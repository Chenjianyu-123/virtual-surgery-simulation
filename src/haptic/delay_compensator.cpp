#include "haptic/delay_compensator.h"
#include <numeric>
#include <cmath>

namespace vss {

// ==================== DelayCompensator ====================

DelayCompensator::DelayCompensator(const Config& config)
    : config_(config) {
    reset();
}

void DelayCompensator::reset() {
    ar_coeffs_.assign(config_.ar_order, 0.0);
    history_.clear();
    prediction_error_ = 0.0;
    last_prediction_ = 0.0;
    
    // 初始化向量版本
    history_vec_.clear();
    ar_coeffs_vec_.assign(config_.ar_order, Vector3d::Zero());
    last_prediction_vec_ = Vector3d::Zero();
    
    // 初始化AR系数（简单平均）
    for (int i = 0; i < config_.ar_order; ++i) {
        ar_coeffs_[i] = 1.0 / config_.ar_order;
    }
}

void DelayCompensator::addToHistory(double value) {
    history_.push_back(value);
    while (history_.size() > static_cast<size_t>(config_.window_size)) {
        history_.pop_front();
    }
}

void DelayCompensator::addToHistoryVector(const Vector3d& value) {
    history_vec_.push_back(value);
    while (history_vec_.size() > static_cast<size_t>(config_.window_size)) {
        history_vec_.pop_front();
    }
}

double DelayCompensator::predict() const {
    if (history_.size() < static_cast<size_t>(config_.ar_order)) {
        return history_.empty() ? 0.0 : history_.back();
    }
    
    double prediction = 0.0;
    auto it = history_.rbegin();
    for (int i = 0; i < config_.ar_order && it != history_.rend(); ++i, ++it) {
        prediction += ar_coeffs_[i] * (*it);
    }
    
    return prediction;
}

Vector3d DelayCompensator::predictVector() const {
    if (history_vec_.size() < static_cast<size_t>(config_.ar_order)) {
        return history_vec_.empty() ? Vector3d::Zero() : history_vec_.back();
    }
    
    Vector3d prediction = Vector3d::Zero();
    auto it = history_vec_.rbegin();
    for (int i = 0; i < config_.ar_order && it != history_vec_.rend(); ++i, ++it) {
        prediction += ar_coeffs_vec_[i].cwiseProduct(*it);
    }
    
    return prediction;
}

void DelayCompensator::updateARCoefficients(double actual, double predicted) {
    prediction_error_ = actual - predicted;
    
    // 梯度下降更新AR系数
    auto it = history_.rbegin();
    for (int i = 0; i < config_.ar_order && it != history_.rend(); ++i, ++it) {
        double gradient = -2.0 * prediction_error_ * (*it);
        ar_coeffs_[i] -= config_.learning_rate * gradient;
    }
    
    // 归一化系数，确保稳定性
    double sum = 0.0;
    for (double coeff : ar_coeffs_) {
        sum += std::abs(coeff);
    }
    if (sum > 1.5) {
        for (double& coeff : ar_coeffs_) {
            coeff /= sum;
        }
    }
}

void DelayCompensator::updateARCoefficientsVector(const Vector3d& actual, 
                                                   const Vector3d& predicted) {
    Vector3d error = actual - predicted;
    
    auto it = history_vec_.rbegin();
    for (int i = 0; i < config_.ar_order && it != history_vec_.rend(); ++i, ++it) {
        Vector3d gradient = -2.0 * error.cwiseProduct(*it);
        ar_coeffs_vec_[i] -= config_.learning_rate * gradient;
    }
    
    // 限制系数范围
    for (auto& coeff : ar_coeffs_vec_) {
        for (int j = 0; j < 3; ++j) {
            coeff[j] = std::max(-1.0, std::min(1.0, coeff[j]));
        }
    }
}

double DelayCompensator::update(double measured_force) {
    // 首先进行预测（基于历史数据）
    double prediction = predict();
    
    // 更新历史窗口
    addToHistory(measured_force);
    
    // 更新AR系数（使用实际值与预测值的误差）
    if (history_.size() >= static_cast<size_t>(config_.ar_order)) {
        updateARCoefficients(measured_force, last_prediction_);
    }
    
    last_prediction_ = prediction;
    
    // 返回预测值（补偿延迟）
    return prediction;
}

Vector3d DelayCompensator::update(const Vector3d& measured_force) {
    Vector3d prediction = predictVector();
    
    addToHistoryVector(measured_force);
    
    if (history_vec_.size() >= static_cast<size_t>(config_.ar_order)) {
        updateARCoefficientsVector(measured_force, last_prediction_vec_);
    }
    
    last_prediction_vec_ = prediction;
    
    return prediction;
}

void DelayCompensator::setConfig(const Config& config) {
    config_ = config;
    reset();
}

// ==================== LinearPredictor ====================

LinearPredictor::LinearPredictor(int window_size)
    : window_size_(window_size) {
    reset();
}

void LinearPredictor::reset() {
    window_.clear();
    slope_ = 0.0;
    intercept_ = 0.0;
}

void LinearPredictor::fitLinearModel() {
    if (window_.size() < 2) return;
    
    int n = window_.size();
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    
    int x = 0;
    for (double y : window_) {
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
        ++x;
    }
    
    double denom = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denom) > 1e-10) {
        slope_ = (n * sum_xy - sum_x * sum_y) / denom;
        intercept_ = (sum_y - slope_ * sum_x) / n;
    }
}

double LinearPredictor::predict(double new_value) {
    window_.push_back(new_value);
    while (window_.size() > static_cast<size_t>(window_size_)) {
        window_.pop_front();
    }
    
    fitLinearModel();
    
    // 预测下一个值
    int next_x = static_cast<int>(window_.size());
    return slope_ * next_x + intercept_;
}

} // namespace vss
