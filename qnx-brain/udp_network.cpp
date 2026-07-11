#include "v2v_network.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>

namespace v2v {
namespace {

const char kBroadcastAddress[] = "255.255.255.255";
// Keep this comfortably below the 150 ms proposal-consensus deadline so the
// processing loop can inspect its clock even when no datagrams are arriving.
const long kReceiveTimeoutMicroseconds = 20000L;

class UdpNetwork : public V2VNetwork {
public:
    UdpNetwork()
        : sendSocket_(-1), receiveSocket_(-1), running_(false) {
        std::memset(&broadcastAddress_, 0, sizeof(broadcastAddress_));
    }

    ~UdpNetwork() override {
        shutdown();
    }

    bool initialize() override {
        // Initialization is normally single-threaded, but locking both socket
        // paths also makes an accidental concurrent initialize harmless.
        std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
        std::lock(sendMutex_, receiveMutex_);
        std::lock_guard<std::mutex> sendLock(sendMutex_, std::adopt_lock);
        std::lock_guard<std::mutex> receiveLock(receiveMutex_, std::adopt_lock);

        if (running_.load()) {
            return true;
        }

        closeSocket(sendSocket_);
        closeSocket(receiveSocket_);

        const int sendFd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sendFd < 0) {
            return false;
        }

        int enabled = 1;
        if (::setsockopt(sendFd, SOL_SOCKET, SO_BROADCAST,
                         &enabled, sizeof(enabled)) < 0 ||
            ::setsockopt(sendFd, SOL_SOCKET, SO_REUSEADDR,
                         &enabled, sizeof(enabled)) < 0) {
            ::close(sendFd);
            return false;
        }

        const int receiveFd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (receiveFd < 0) {
            ::close(sendFd);
            return false;
        }

        if (::setsockopt(receiveFd, SOL_SOCKET, SO_REUSEADDR,
                         &enabled, sizeof(enabled)) < 0) {
            ::close(receiveFd);
            ::close(sendFd);
            return false;
        }

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = kReceiveTimeoutMicroseconds;
        if (::setsockopt(receiveFd, SOL_SOCKET, SO_RCVTIMEO,
                         &timeout, sizeof(timeout)) < 0) {
            ::close(receiveFd);
            ::close(sendFd);
            return false;
        }

        sockaddr_in localAddress;
        std::memset(&localAddress, 0, sizeof(localAddress));
        localAddress.sin_family = AF_INET;
        localAddress.sin_port = htons(kUdpPort);
        localAddress.sin_addr.s_addr = htonl(INADDR_ANY);

        if (::bind(receiveFd,
                   reinterpret_cast<const sockaddr*>(&localAddress),
                   sizeof(localAddress)) < 0) {
            ::close(receiveFd);
            ::close(sendFd);
            return false;
        }

        sockaddr_in destination;
        std::memset(&destination, 0, sizeof(destination));
        destination.sin_family = AF_INET;
        destination.sin_port = htons(kUdpPort);
        if (::inet_pton(AF_INET, kBroadcastAddress,
                        &destination.sin_addr) != 1) {
            ::close(receiveFd);
            ::close(sendFd);
            return false;
        }

        sendSocket_ = sendFd;
        receiveSocket_ = receiveFd;
        broadcastAddress_ = destination;
        running_.store(true);
        return true;
    }

    bool broadcast(const std::string& topic,
                   const std::string& payload) override {
        // A comma is the envelope delimiter, so accepting one in the topic would
        // make the received topic ambiguous. This subtraction form also avoids a
        // size_t overflow while checking the encoded packet length.
        if (topic.empty() || topic.find(',') != std::string::npos ||
            topic.size() >= kMaxDatagramSize ||
            payload.size() > kMaxDatagramSize - topic.size() - 1U) {
            return false;
        }

        std::string packet;
        packet.reserve(topic.size() + 1U + payload.size());
        packet.append(topic);
        packet.push_back(',');
        packet.append(payload);

        std::lock_guard<std::mutex> lock(sendMutex_);
        if (!running_.load() || sendSocket_ < 0) {
            return false;
        }

        ssize_t bytesSent;
        do {
            bytesSent = ::sendto(
                sendSocket_, packet.data(), packet.size(), 0,
                reinterpret_cast<const sockaddr*>(&broadcastAddress_),
                sizeof(broadcastAddress_));
        } while (bytesSent < 0 && errno == EINTR && running_.load());

        return bytesSent >= 0 &&
               static_cast<std::size_t>(bytesSent) == packet.size();
    }

    bool listen(std::string& topic, std::string& payload) override {
        topic.clear();
        payload.clear();

        // Only one receiver may consume a datagram at a time. Keeping the mutex
        // through recvfrom also prevents shutdown from closing/reusing the file
        // descriptor mid-call. SO_RCVTIMEO bounds shutdown latency to 20 ms.
        std::lock_guard<std::mutex> lock(receiveMutex_);
        if (!running_.load() || receiveSocket_ < 0) {
            return false;
        }

        char buffer[kMaxDatagramSize + 1U];
        while (running_.load()) {
            const ssize_t bytesReceived =
                ::recvfrom(receiveSocket_, buffer, sizeof(buffer), 0, NULL, NULL);

            if (bytesReceived < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return false;
                }
                return false;
            }

            // Reading one byte beyond the advertised cap makes both a 4097-byte
            // datagram and any larger (truncated) datagram unambiguously invalid.
            if (bytesReceived == 0 ||
                static_cast<std::size_t>(bytesReceived) > kMaxDatagramSize) {
                return false;
            }

            const std::string packet(
                buffer, static_cast<std::size_t>(bytesReceived));
            const std::string::size_type separator = packet.find(',');
            if (separator == std::string::npos || separator == 0U) {
                return false;
            }

            topic.assign(packet, 0U, separator);
            payload.assign(packet, separator + 1U, std::string::npos);
            return true;
        }

        return false;
    }

    void shutdown() override {
        std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);

        // Publish the stop first so a timed-out listen() exits its receive loop.
        running_.store(false);

        {
            std::lock_guard<std::mutex> lock(sendMutex_);
            closeSocket(sendSocket_);
        }
        {
            std::lock_guard<std::mutex> lock(receiveMutex_);
            closeSocket(receiveSocket_);
        }
    }

private:
    static void closeSocket(int& socketFd) {
        if (socketFd >= 0) {
            ::close(socketFd);
            socketFd = -1;
        }
    }

    UdpNetwork(const UdpNetwork&) = delete;
    UdpNetwork& operator=(const UdpNetwork&) = delete;

    int sendSocket_;
    int receiveSocket_;
    sockaddr_in broadcastAddress_;
    std::atomic<bool> running_;
    std::mutex lifecycleMutex_;
    std::mutex sendMutex_;
    std::mutex receiveMutex_;
};

}  // namespace

std::unique_ptr<V2VNetwork> makeUdpNetwork() {
    return std::unique_ptr<V2VNetwork>(new UdpNetwork());
}

}  // namespace v2v
