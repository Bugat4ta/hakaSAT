#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

namespace cromulence::attitude {

class AttitudeApp {
public:
    AttitudeApp();
    ~AttitudeApp();

    void initialize();
    void shutdown();

    void noop();
    void reset();
    void config(int maxBlind, bool enabled, size_t frameCount);
    void attitude_request(double qx, double qy, double qz, double qs);
    void on_heartbeat();

private:
    void attitude_cmd(double qx, double qy, double qz, double qS);
    
    bool enabled_;
    bool blocked_;
    double maxBlind_;
    size_t counter_;
    size_t framesPerRequest_;
    static constexpr double MAX_BLIND_DEFAULT = 1000000000.0;
    static const size_t FRAMES = 10;
    static const size_t STABILIZATION_COUNT = 5;
    static constexpr double TOLERANCE = 0.02;
};

AttitudeApp::AttitudeApp() 
    : enabled_(false), blocked_(false), maxBlind_(MAX_BLIND_DEFAULT), counter_(0), framesPerRequest_(FRAMES) {}

AttitudeApp::~AttitudeApp() {}

void AttitudeApp::initialize() {
    std::cout << "[INFO] Starting AttitudeApp" << std::endl;
    maxBlind_ = MAX_BLIND_DEFAULT;
    counter_ = 0;
    framesPerRequest_ = FRAMES;
    enabled_ = false;
    blocked_ = false;
    std::cout << "[INFO] Initialization complete" << std::endl;
}

void AttitudeApp::shutdown() {
    std::cout << "[INFO] Shutting down" << std::endl;
}

void AttitudeApp::noop() {
    std::cout << "[INFO] No operation command received." << std::endl;
}

void AttitudeApp::reset() {
    std::cout << "[INFO] Resetting. Polling disabled" << std::endl;
    maxBlind_ = MAX_BLIND_DEFAULT;
    framesPerRequest_ = FRAMES;
    blocked_ = false;
    enabled_ = false;
    counter_ = 0;
}

void AttitudeApp::config(int maxBlind, bool enabled, size_t frameCount) {
    std::cout << "[INFO] Configuring AttitudeApp" << std::endl;
    maxBlind_ = maxBlind;
    enabled_ = enabled;
    framesPerRequest_ = frameCount;
}

void AttitudeApp::attitude_request(double qx, double qy, double qz, double qs) {
    if (enabled_) {
        attitude_cmd(qx, qy, qz, qs);
    } else {
        std::cout << "[INFO] Ignoring pointing. Enabled: " << enabled_ << std::endl;
    }
}

void AttitudeApp::on_heartbeat() {
    counter_++;
    std::cout << "[INFO] Heartbeat received. Counter incremented: " << counter_ << std::endl;
}

void AttitudeApp::attitude_cmd(double qx, double qy, double qz, double qS) {
    double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qS * qS);
    std::cout << "[INFO] Processing command with qx: " << qx << ", qy: " << qy << ", qz: " << qz << ", qs: " << qS << std::endl;
    std::cout << "[INFO] Quaternion norm: " << norm << std::endl;

    if (std::abs(norm - 1.0) < TOLERANCE) {
        std::cout << "[INFO] Satellite is becoming more stable!" << std::endl;
        counter_++;

        if (counter_ >= STABILIZATION_COUNT) {
            std::cout << "[INFO] Satellite is stable. Challenge completed!" << std::endl;
            enabled_ = false;
        }
    } else {
        std::cout << "[INFO] Satellite is still spinning. Current quaternion: [" << qx << ", " << qy << ", " << qz << ", " << qS << "]" << std::endl;
        counter_ = 0;
    }
}

} 

int main() {
    cromulence::attitude::AttitudeApp app;
    app.initialize();
    
    app.noop();
    app.reset();
    app.config(200, true, 10);
    
    app.attitude_request(0.9, 0.1, 0.1, 0.1);
    app.attitude_request(0.1, 0.1, 0.1, 0.9);
    app.attitude_request(0.0, 0.0, 0.0, 1.0);
    app.attitude_request(0.0, 0.0, 0.0, 1.0);
    app.attitude_request(0.0, 0.0, 0.0, 1.0);
    app.attitude_request(0.0, 0.0, 0.0, 1.0);
    app.attitude_request(0.0, 0.0, 0.0, 1.0);
    
    app.on_heartbeat();
    app.shutdown();
    
    return 0;
}
