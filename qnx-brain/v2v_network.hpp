#ifndef V2V_NETWORK_HPP
#define V2V_NETWORK_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace v2v {

// Shared wire-level settings for the PoC. A packet is at most 4096 bytes and
// has the form "topic,payload" (for example, "TELEMETRY,car-1,...").
static const std::uint16_t kUdpPort = 12345;
static const std::size_t kMaxDatagramSize = 4096;

// Transport abstraction kept deliberately small so a DDS-backed implementation
// can replace UDP without changing collision or consensus code.
class V2VNetwork {
public:
    virtual ~V2VNetwork() {}

    // Acquires all transport resources. Calling initialize() more than once is
    // safe; subsequent calls succeed while the transport remains initialized.
    virtual bool initialize() = 0;

    // Sends one packet as topic + ',' + payload. Returns false when the topic is
    // invalid, the encoded packet is too large, or the transport cannot send it.
    virtual bool broadcast(const std::string& topic,
                           const std::string& payload) = 0;

    // Blocks until a valid packet is received, the short receive timeout expires,
    // shutdown is requested, or a socket error occurs. Malformed and oversized
    // datagrams are safely discarded. A false result is therefore nonfatal and
    // lets the caller regularly evaluate its own deadlines and stop condition.
    virtual bool listen(std::string& topic, std::string& payload) = 0;

    // Unblocks listen() within the socket timeout and releases all descriptors.
    // Safe to call repeatedly and from a thread other than the listener.
    virtual void shutdown() = 0;
};

// The returned network is intentionally not initialized, allowing callers to
// construct it before their process/thread lifecycle has started.
std::unique_ptr<V2VNetwork> makeUdpNetwork();

}  // namespace v2v

#endif  // V2V_NETWORK_HPP
