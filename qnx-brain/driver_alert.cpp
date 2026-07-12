#include "driver_alert.hpp"

#include <curl/curl.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

const double kDeliveryAllowanceSeconds = 1.0;
const std::size_t kMaximumAudioBytes = 1024U * 1024U;
const std::size_t kMaximumQueuedAlerts = 2U;
const long kSynthesisTimeoutMilliseconds = 4000L;
const long kUploadTimeoutMilliseconds = 2000L;

struct ResponseBuffer {
    std::string bytes;
    std::size_t maximum_size;
};

std::string environment(const char* name) {
    const char* value = std::getenv(name);
    return value == NULL ? std::string() : std::string(value);
}

bool safeIdentifier(const std::string& value) {
    if (value.empty() || value.size() > 128U) {
        return false;
    }
    for (std::string::const_iterator it = value.begin(); it != value.end(); ++it) {
        const char character = *it;
        const bool valid = (character >= 'A' && character <= 'Z') ||
                           (character >= 'a' && character <= 'z') ||
                           (character >= '0' && character <= '9') ||
                           character == '-' || character == '_' ||
                           character == '.';
        if (!valid) {
            return false;
        }
    }
    return true;
}

std::string trimTrailingSlash(const std::string& value) {
    std::string result(value);
    while (!result.empty() && result[result.size() - 1U] == '/') {
        result.erase(result.size() - 1U);
    }
    return result;
}

bool validBridgeUrl(const std::string& value) {
    return value.compare(0U, 7U, "http://") == 0 ||
           value.compare(0U, 8U, "https://") == 0;
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream output;
    for (std::string::const_iterator it = value.begin(); it != value.end(); ++it) {
        const unsigned char character = static_cast<unsigned char>(*it);
        switch (character) {
            case '\"': output << "\\\""; break;
            case '\\': output << "\\\\"; break;
            case '\b': output << "\\b"; break;
            case '\f': output << "\\f"; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default:
                if (character < 0x20U) {
                    output << "\\u" << std::hex << std::setw(4)
                           << std::setfill('0') << static_cast<int>(character)
                           << std::dec;
                } else {
                    output << static_cast<char>(character);
                }
        }
    }
    return output.str();
}

std::string secondsAsWords(double ttc_seconds) {
    static const char* const words[] = {
        "zero", "one", "two", "three", "four", "five",
        "six", "seven", "eight", "nine", "ten"
    };
    const double conservative =
        std::max(0.0, ttc_seconds - kDeliveryAllowanceSeconds);
    if (conservative < 1.25) {
        return std::string();
    }
    int rounded = static_cast<int>(std::floor(conservative + 0.5));
    rounded = std::max(1, std::min(10, rounded));
    return words[rounded];
}

std::string alertText(const std::string& action, double ttc_seconds) {
    std::ostringstream text;
    const std::string seconds = secondsAsWords(ttc_seconds);
    if (seconds.empty()) {
        text << "Collision warning. Impact may be imminent. ";
    } else {
        text << "Collision warning. Potential impact in about "
             << seconds << " seconds. ";
    }

    if (action == "SWERVE_LEFT") {
        text << "Swerving left to prevent collision.";
    } else if (action == "SWERVE_RIGHT") {
        text << "Swerving right to prevent collision.";
    } else if (action == "EMERGENCY_STOP") {
        text << "Emergency stop to prevent collision.";
    } else {
        text << "Braking to prevent collision.";
    }
    return text.str();
}

std::size_t appendResponse(char* data,
                           std::size_t size,
                           std::size_t count,
                           void* user_data) {
    ResponseBuffer* response = static_cast<ResponseBuffer*>(user_data);
    if (count != 0U && size > static_cast<std::size_t>(-1) / count) {
        return 0U;
    }
    const std::size_t byte_count = size * count;
    if (response->bytes.size() > response->maximum_size ||
        byte_count > response->maximum_size - response->bytes.size()) {
        return 0U;
    }
    response->bytes.append(data, byte_count);
    return byte_count;
}

bool performRequest(CURL* curl,
                    long expected_status,
                    const char* operation) {
    char error_buffer[CURL_ERROR_SIZE];
    error_buffer[0] = '\0';
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error_buffer);
    const CURLcode result = curl_easy_perform(curl);
    if (result != CURLE_OK) {
        std::cerr << "Driver alert " << operation << " failed: "
                  << (error_buffer[0] == '\0'
                          ? curl_easy_strerror(result)
                          : error_buffer)
                  << std::endl;
        return false;
    }

    long status = 0L;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    if (status != expected_status) {
        std::cerr << "Driver alert " << operation << " returned HTTP "
                  << status << std::endl;
        return false;
    }
    return true;
}

}  // namespace

DriverAlertService::DriverAlertService(const std::string& car_id, bool enabled)
    : car_id_(car_id),
      api_key_(environment("ELEVENLABS_API_KEY")),
      voice_id_(environment("ELEVENLABS_VOICE_ID")),
      model_id_(environment("ELEVENLABS_MODEL_ID")),
      bridge_url_(trimTrailingSlash(environment("V2V_ALERT_BRIDGE_URL"))),
      alert_token_(environment("V2V_ALERT_TOKEN")),
      configured_(false),
      stopping_(false) {
    if (!enabled) {
        status_message_ = "Driver voice alerts disabled by --no-audio.";
        return;
    }
    if (model_id_.empty()) {
        model_id_ = "eleven_flash_v2_5";
    }
    if (api_key_.empty() || !safeIdentifier(voice_id_) ||
        !validBridgeUrl(bridge_url_) || !safeIdentifier(alert_token_)) {
        status_message_ =
            "Driver voice alerts are not configured. Set ELEVENLABS_API_KEY, "
            "ELEVENLABS_VOICE_ID, V2V_ALERT_BRIDGE_URL, and V2V_ALERT_TOKEN.";
        return;
    }
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        status_message_ = "libcurl global initialization failed; alerts disabled.";
        return;
    }

    configured_ = true;
    status_message_ =
        "Driver voice alerts enabled (ElevenLabs MP3 via laptop phone bridge).";
    worker_ = std::thread(&DriverAlertService::workerLoop, this);
}

DriverAlertService::~DriverAlertService() {
    if (configured_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            queue_.clear();
        }
        wake_.notify_one();
        worker_.join();
        curl_global_cleanup();
    }
}

void DriverAlertService::enqueue(const std::string& action,
                                 double ttc_seconds) {
    if (!configured_) {
        return;
    }
    Alert alert;
    alert.action = action;
    alert.ttc_seconds = ttc_seconds;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) {
            return;
        }
        if (queue_.size() >= kMaximumQueuedAlerts) {
            queue_.pop_front();
        }
        queue_.push_back(alert);
    }
    wake_.notify_one();
}

bool DriverAlertService::configured() const {
    return configured_;
}

const std::string& DriverAlertService::statusMessage() const {
    return status_message_;
}

void DriverAlertService::workerLoop() {
    for (;;) {
        Alert alert;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            while (!stopping_ && queue_.empty()) {
                wake_.wait(lock);
            }
            if (stopping_) {
                return;
            }
            alert = queue_.front();
            queue_.pop_front();
        }

        std::string spoken_text;
        std::string audio;
        if (synthesize(alert, spoken_text, audio) &&
            upload(alert, spoken_text, audio)) {
            std::cout << "Driver alert delivered for " << car_id_ << ": "
                      << spoken_text << std::endl;
        }
    }
}

bool DriverAlertService::synthesize(const Alert& alert,
                                    std::string& spoken_text,
                                    std::string& audio) {
    spoken_text = alertText(alert.action, alert.ttc_seconds);
    const std::string request_body =
        std::string("{\"text\":\"") + jsonEscape(spoken_text) +
        "\",\"model_id\":\"" + jsonEscape(model_id_) +
        "\",\"voice_settings\":{" +
        "\"stability\":0.6,\"similarity_boost\":0.75,\"speed\":1.08}}";
    const std::string request_url =
        "https://api.elevenlabs.io/v1/text-to-speech/" + voice_id_ +
        "?output_format=mp3_44100_128";

    CURL* curl = curl_easy_init();
    if (curl == NULL) {
        std::cerr << "Driver alert synthesis failed: curl_easy_init" << std::endl;
        return false;
    }
    struct curl_slist* headers = NULL;
    const std::string api_header = "xi-api-key: " + api_key_;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "Accept: audio/mpeg");
    headers = curl_slist_append(headers, api_header.c_str());

    ResponseBuffer response;
    response.maximum_size = kMaximumAudioBytes;
    curl_easy_setopt(curl, CURLOPT_URL, request_url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.data());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE,
                     static_cast<curl_off_t>(request_body.size()));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, appendResponse);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 1500L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, kSynthesisTimeoutMilliseconds);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    const bool succeeded = performRequest(curl, 200L, "synthesis") &&
                           !response.bytes.empty();
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    if (succeeded) {
        audio.swap(response.bytes);
    }
    return succeeded;
}

bool DriverAlertService::upload(const Alert& alert,
                                const std::string& spoken_text,
                                const std::string& audio) {
    CURL* curl = curl_easy_init();
    if (curl == NULL) {
        std::cerr << "Driver alert upload failed: curl_easy_init" << std::endl;
        return false;
    }
    const std::string request_url =
        bridge_url_ + "/api/driver-alert/" + car_id_;
    const std::string token_header = "X-V2V-Alert-Token: " + alert_token_;
    const std::string action_header = "X-V2V-Action: " + alert.action;
    std::ostringstream ttc_header;
    ttc_header << "X-V2V-TTC: " << std::fixed << std::setprecision(3)
               << alert.ttc_seconds;
    const std::string text_header =
        "X-V2V-Alert-Text: " + spoken_text;

    struct curl_slist* headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: audio/mpeg");
    headers = curl_slist_append(headers, token_header.c_str());
    headers = curl_slist_append(headers, action_header.c_str());
    headers = curl_slist_append(headers, ttc_header.str().c_str());
    headers = curl_slist_append(headers, text_header.c_str());

    ResponseBuffer response;
    response.maximum_size = 4096U;
    curl_easy_setopt(curl, CURLOPT_URL, request_url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, audio.data());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE,
                     static_cast<curl_off_t>(audio.size()));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, appendResponse);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 750L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, kUploadTimeoutMilliseconds);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    const bool succeeded = performRequest(curl, 202L, "upload");
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return succeeded;
}
