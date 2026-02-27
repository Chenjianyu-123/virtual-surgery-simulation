#include "haptic/adaptive_filter.h"
#include <cmath>

namespace vss {

// ==================== AdaptiveFilter ====================

AdaptiveFilter::AdaptiveFilter(const Config& config) 
    : config_(config) {
    reset();
}

void AdaptiveFilter::reset() {
    state_ = FilterState::NORMAL;
    current_cutoff_ = config_.normal_cutoff;
    prev_output_ = 0.0;
    prev_output_vec_ = Vector3d::Zero();
    prev_force_ = 0.0;
    event_timer_ = 0.0;
    updateAlpha();
}

void AdaptiveFilter::updateAlpha() {
    // 一阶低通滤波器系数: alpha = 1 / (1 + 2*pi*fc*dt)
    double dt = 1.0 / config_.sampling_rate;
    double rc = 1.0 / (2.0 * M_PI * current_cutoff_);
    alpha_ = dt / (rc + dt);
    
    // 限制alpha范围
    if (alpha_ > 1.0) alpha_ = 1.0;
    if (alpha_ < 0.0) alpha_ = 0.0;
}

bool AdaptiveFilter::detectEvent(double force, double dt) {
    // 计算力变化率 (N/s)
    double force_derivative = (force - prev_force_) / dt;
    
    // 转换为 N/ms 进行比较
    double derivative_ms = std::abs(force_derivative) / 1000.0;
    
    prev_force_ = force;
    
    return derivative_ms > config_.event_threshold;
}

void AdaptiveFilter::updateState(double dt) {
    if (state_ == FilterState::EVENT) {
        // 事件状态衰减计时
        event_timer_ += dt;
        if (event_timer_ >= config_.event_decay_time) {
            // 切换回正常状态
            state_ = FilterState::NORMAL;
            current_cutoff_ = config_.normal_cutoff;
            updateAlpha();
        }
    }
}

double AdaptiveFilter::filter(double force) {
    double dt = 1.0 / config_.sampling_rate;
    
    // 检测事件
    bool event_detected = detectEvent(force, dt);
    
    if (event_detected && state_ == FilterState::NORMAL) {
        // 切换到事件状态
        state_ = FilterState::EVENT;
        current_cutoff_ = config_.event_cutoff;
        event_timer_ = 0.0;
        updateAlpha();
    }
    
    // 更新状态机
    updateState(dt);
    
    // 应用一阶低通滤波
    double output = alpha_ * force + (1.0 - alpha_) * prev_output_;
    prev_output_ = output;
    
    return output;
}

Vector3d AdaptiveFilter::filter(const Vector3d& force) {
    double dt = 1.0 / config_.sampling_rate;
    
    // 使用力向量的模来检测事件
    double force_magnitude = force.norm();
    bool event_detected = detectEvent(force_magnitude, dt);
    
    if (event_detected && state_ == FilterState::NORMAL) {
        state_ = FilterState::EVENT;
        current_cutoff_ = config_.event_cutoff;
        event_timer_ = 0.0;
        updateAlpha();
    }
    
    updateState(dt);
    
    // 对每个分量应用滤波
    Vector3d output;
    for (int i = 0; i < 3; ++i) {
        output[i] = alpha_ * force[i] + (1.0 - alpha_) * prev_output_vec_[i];
        prev_output_vec_[i] = output[i];
    }
    
    return output;
}

void AdaptiveFilter::triggerEvent() {
    state_ = FilterState::EVENT;
    current_cutoff_ = config_.event_cutoff;
    event_timer_ = 0.0;
    updateAlpha();
}

void AdaptiveFilter::setConfig(const Config& config) {
    config_ = config;
    reset();
}

// ==================== LowPassFilter ====================

LowPassFilter::LowPassFilter(double cutoff_hz, double sample_rate_hz)
    : sample_rate_(sample_rate_hz), cutoff_(cutoff_hz) {
    setCutoff(cutoff_hz);
}

void LowPassFilter::reset() {
    prev_output_ = 0.0;
}

double LowPassFilter::filter(double input) {
    double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
    prev_output_ = output;
    return output;
}

void LowPassFilter::setCutoff(double cutoff_hz) {
    cutoff_ = cutoff_hz;
    double dt = 1.0 / sample_rate_;
    double rc = 1.0 / (2.0 * M_PI * cutoff_);
    alpha_ = dt / (rc + dt);
    
    if (alpha_ > 1.0) alpha_ = 1.0;
    if (alpha_ < 0.0) alpha_ = 0.0;
}

} // namespace vss
