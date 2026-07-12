#ifndef DRIVER_ALERT_HPP
#define DRIVER_ALERT_HPP

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

class DriverAlertService {
public:
    DriverAlertService(const std::string& car_id, bool enabled);
    ~DriverAlertService();

    // Queues a committed action for synthesis and phone delivery. The safety
    // thread never waits for either ElevenLabs or the laptop bridge.
    void enqueue(const std::string& action, double ttc_seconds);
    bool configured() const;
    const std::string& statusMessage() const;

private:
    struct Alert {
        std::string action;
        double ttc_seconds;
    };

    void workerLoop();
    bool synthesize(const Alert& alert,
                    std::string& spoken_text,
                    std::string& audio);
    bool upload(const Alert& alert,
                const std::string& spoken_text,
                const std::string& audio);

    DriverAlertService(const DriverAlertService&);
    DriverAlertService& operator=(const DriverAlertService&);

    std::string car_id_;
    std::string api_key_;
    std::string voice_id_;
    std::string model_id_;
    std::string bridge_url_;
    std::string alert_token_;
    bool configured_;
    bool stopping_;
    std::string status_message_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<Alert> queue_;
    std::thread worker_;
};

#endif  // DRIVER_ALERT_HPP
