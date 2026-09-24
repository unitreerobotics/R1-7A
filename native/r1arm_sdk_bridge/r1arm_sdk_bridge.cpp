#include "arm_motor_bus.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

struct Bridge {
    ArmMotorBus bus;
};

void setError(char* dst, int capacity, const std::string& message) {
    if (dst == nullptr || capacity <= 0) {
        return;
    }
    const auto count = static_cast<size_t>(capacity - 1);
    std::strncpy(dst, message.c_str(), count);
    dst[count] = '\0';
}

Bridge* asBridge(void* handle) {
    return static_cast<Bridge*>(handle);
}

template <typename Fn>
int guarded(Fn&& fn, char* error, int error_capacity) {
    try {
        fn();
        setError(error, error_capacity, "");
        return 0;
    } catch (const std::exception& exc) {
        setError(error, error_capacity, exc.what());
        return -1;
    } catch (...) {
        setError(error, error_capacity, "unknown C++ exception");
        return -1;
    }
}

}  // namespace

extern "C" {

const char* r1arm_bridge_version() {
    return "r1arm_sdk_bridge_v1";
}

void* r1arm_bridge_create() {
    try {
        return new Bridge();
    } catch (...) {
        return nullptr;
    }
}

void r1arm_bridge_destroy(void* handle) {
    delete asBridge(handle);
}

int r1arm_bridge_open(void* handle,
                      const char* port,
                      int baudrate,
                      const char* calibration_path,
                      char* error,
                      int error_capacity) {
    if (handle == nullptr || port == nullptr || calibration_path == nullptr) {
        setError(error, error_capacity, "null bridge, port, or calibration path");
        return -1;
    }
    return guarded(
        [&]() {
            if (!asBridge(handle)->bus.open(port, baudrate, calibration_path)) {
                throw std::runtime_error(std::string("failed to open R1Arm port ") + port);
            }
        },
        error,
        error_capacity);
}

int r1arm_bridge_is_open(void* handle) {
    return handle != nullptr && asBridge(handle)->bus.isOpen() ? 1 : 0;
}

int r1arm_bridge_probe(void* handle, char* error, int error_capacity) {
    if (handle == nullptr) {
        setError(error, error_capacity, "null bridge");
        return -1;
    }
    return guarded(
        [&]() {
            std::array<ArmJointCmd, kArmJointCount> commands{};
            for (auto& command : commands) {
                command.mode = 1;
                command.timed_out = false;
                command.q = 0.f;
                command.dq = 0.f;
                command.tau = 0.f;
                command.kp = 0.f;
                command.kd = 0.f;
            }
            asBridge(handle)->bus.step(commands);
        },
        error,
        error_capacity);
}

int r1arm_bridge_command_position(void* handle,
                                  const float* q,
                                  float kp,
                                  float kd,
                                  char* error,
                                  int error_capacity) {
    if (handle == nullptr || q == nullptr) {
        setError(error, error_capacity, "null bridge or position command");
        return -1;
    }
    return guarded(
        [&]() {
            std::array<ArmJointCmd, kArmJointCount> commands{};
            for (int i = 0; i < kArmJointCount; ++i) {
                commands[i].mode = 1;
                commands[i].timed_out = false;
                commands[i].q = q[i];
                commands[i].dq = 0.f;
                commands[i].tau = 0.f;
                commands[i].kp = kp;
                commands[i].kd = kd;
            }
            asBridge(handle)->bus.step(commands);
            const auto fresh = asBridge(handle)->bus.lastStepFeedback();
            std::ostringstream missing;
            for (int i = 0; i < kArmJointCount; ++i) {
                if (!fresh[i]) {
                    missing << (missing.tellp() > 0 ? ", J" : "J") << (i + 1);
                }
            }
            if (missing.tellp() > 0) {
                throw std::runtime_error(
                    "R1Arm command cycle missing fresh feedback from " + missing.str());
            }
        },
        error,
        error_capacity);
}

int r1arm_bridge_read_state(void* handle,
                            float* q,
                            float* dq,
                            float* tau_est,
                            uint8_t* mode,
                            uint8_t* timeout,
                            uint32_t* error_code,
                            uint8_t* ready,
                            char* error,
                            int error_capacity) {
    if (handle == nullptr || q == nullptr || dq == nullptr || tau_est == nullptr ||
        mode == nullptr || timeout == nullptr || error_code == nullptr || ready == nullptr) {
        setError(error, error_capacity, "null bridge or state output array");
        return -1;
    }
    return guarded(
        [&]() {
            const auto states = asBridge(handle)->bus.states();
            const auto feedback_ready = asBridge(handle)->bus.feedbackReady();
            for (int i = 0; i < kArmJointCount; ++i) {
                q[i] = states[i].q;
                dq[i] = states[i].dq;
                tau_est[i] = states[i].tau_est;
                mode[i] = states[i].mode;
                timeout[i] = states[i].timeout;
                error_code[i] = states[i].error_code;
                ready[i] = feedback_ready[i] ? 1 : 0;
            }
        },
        error,
        error_capacity);
}

int r1arm_bridge_stop(void* handle, char* error, int error_capacity) {
    if (handle == nullptr) {
        setError(error, error_capacity, "null bridge");
        return -1;
    }
    return guarded(
        [&]() {
            std::array<ArmJointCmd, kArmJointCount> commands{};
            for (auto& command : commands) {
                command.mode = 0;
                command.timed_out = false;
                command.q = 0.f;
                command.dq = 0.f;
                command.tau = 0.f;
                command.kp = 0.f;
                command.kd = 0.f;
            }
            asBridge(handle)->bus.step(commands);
        },
        error,
        error_capacity);
}

int r1arm_bridge_close(void* handle, char* error, int error_capacity) {
    if (handle == nullptr) {
        setError(error, error_capacity, "null bridge");
        return -1;
    }
    return guarded([&]() { asBridge(handle)->bus.close(); }, error, error_capacity);
}

}  // extern "C"
