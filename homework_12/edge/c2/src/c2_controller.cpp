#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;

struct C2Controller::Impl {
    C2State state = C2State::DISARMED;

    FcLink fc;
    UdpSocket udp;
    std::ofstream log_file;
    bool hold_sent = false;
    bool c2_healthy_set = false;

    Impl(uint16_t fc_port)
        : fc(fc_port), udp(STUB_PORT)
    {
        log_file.open("/var/log/c2/c2.log", std::ios::app);
    }

    ~Impl() = default;

    static std::string state_name(C2State s) {
        switch (s) {
            case C2State::DISARMED:
                return "DISARMED";
            case C2State::ARMED_HOLD:
                return "ARMED_HOLD";
            case C2State::ARMED_GUIDED:
                return "ARMED_GUIDED";
            case C2State::ARMED_MANUAL:
                return "ARMED_MANUAL";
        }
        return "UNKNOWN";
    }

    void transition(C2State next) {
        if (next == state) {
            return;
        }
        std::cout << "[C2] state: " << state_name(state) << " -> " << state_name(next) << "\n";
        if (log_file.is_open()) {
            log_file << "[C2] state: " << state_name(state) << " -> " << state_name(next) << "\n";
            log_file.flush();
        }
        state = next;
    }
};

C2Controller::C2Controller(uint16_t fc_port)
    : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    // Healthcheck: create /tmp/c2_healthy after first HEARTBEAT from FC
    if (!impl_->c2_healthy_set && impl_->fc.is_connected()) {
        std::ofstream("/tmp/c2_healthy").close();
        impl_->c2_healthy_set = true;
    }

    // Determine target state based on armed status and flight mode
    C2State target_state = impl_->state;
    if (!impl_->fc.is_armed()) {
        target_state = C2State::DISARMED;
    } else {
        auto fm = impl_->fc.flight_mode();
        switch (fm) {
            case FcLink::FlightMode::Guided:  target_state = C2State::ARMED_GUIDED; break;
            case FcLink::FlightMode::Hold:    target_state = C2State::ARMED_HOLD;   break;
            case FcLink::FlightMode::Manual:  target_state = C2State::ARMED_MANUAL; break;
            default:                          target_state = C2State::DISARMED;     break;
        }
    }

    // Transition to new state if changed
    impl_->transition(target_state);

    // Handle state-specific behavior
    switch (impl_->state) {
        case C2State::ARMED_HOLD:
            // Send hold command once on entry to this state
            if (!impl_->hold_sent) {
                impl_->fc.hold();
                impl_->hold_sent = true;
                std::cout << "[C2] sent hold command\n";
                if (impl_->log_file.is_open()) {
                    impl_->log_file << "[C2] sent hold command\n";
                    impl_->log_file.flush();
                }
            }
            break;
        default:
            // Reset hold_sent when leaving ARMED_HOLD
            impl_->hold_sent = false;
            break;
    }

    // Read UDP from auto_stub (non-blocking)
    char buf[512];
    sockaddr_in sender{};
    ssize_t n = impl_->udp.recv(buf, sizeof(buf), sender);
    if (n > 0) {
        buf[n] = '\0';
        std::string json_str(buf, static_cast<std::size_t>(n));

        try {
            auto json = nlohmann::json::parse(json_str);
            float north = json.value("north_m", 0.0f);
            float east  = json.value("east_m", 0.0f);

            switch (impl_->state) {
                case C2State::ARMED_GUIDED:
                    impl_->fc.go_to_ned(north, east);
                    std::cout << "[C2] fwd: north=" << north << " east=" << east << "\n";
                    if (impl_->log_file.is_open()) {
                        impl_->log_file << "[C2] fwd: north=" << north << " east=" << east << "\n";
                        impl_->log_file.flush();
                    }
                    break;
                case C2State::DISARMED:
                    std::cout << "[C2] blocked: waypoint in DISARMED\n";
                    if (impl_->log_file.is_open()) {
                        impl_->log_file << "[C2] blocked: waypoint in DISARMED\n";
                        impl_->log_file.flush();
                    }
                    break;
                case C2State::ARMED_HOLD:
                    std::cout << "[C2] blocked: waypoint in ARMED_HOLD\n";
                    if (impl_->log_file.is_open()) {
                        impl_->log_file << "[C2] blocked: waypoint in ARMED_HOLD\n";
                        impl_->log_file.flush();
                    }
                    break;
                case C2State::ARMED_MANUAL:
                    std::cout << "[C2] blocked: waypoint in ARMED_MANUAL\n";
                    if (impl_->log_file.is_open()) {
                        impl_->log_file << "[C2] blocked: waypoint in ARMED_MANUAL\n";
                        impl_->log_file.flush();
                    }
                    break;
            }
        } catch (const std::exception& e) {
            std::cerr << "[C2] error parsing JSON: " << e.what() << "\n";
        }
    }
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
