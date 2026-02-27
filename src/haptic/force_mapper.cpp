#include "haptic/force_mapper.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace vss {

// ==================== ForceMapper ====================

ForceMapper::ForceMapper(const Config& config)
    : config_(config) {
    // 添加默认校准点
    addCalibrationPoint(0.0, 0.0);
    addCalibrationPoint(config.input_max, config.output_max);
}

double ForceMapper::map(double input_force) const {
    // 限制输入范围
    input_force = clamp(input_force, config_.input_min, config_.input_max);
    
    switch (config_.mapping_type) {
        case MappingType::LINEAR:
            return mapLinear(input_force);
        case MappingType::SQUARE_ROOT:
            return mapSquareRoot(input_force);
        case MappingType::LOGARITHMIC:
            return mapLogarithmic(input_force);
        case MappingType::PIECEWISE:
        case MappingType::USER_DEFINED:
            return mapPiecewise(input_force);
        default:
            return mapLinear(input_force);
    }
}

Vector3d ForceMapper::map(const Vector3d& input_force) const {
    Vector3d output;
    for (int i = 0; i < 3; ++i) {
        output[i] = map(input_force[i]);
    }
    return output;
}

double ForceMapper::mapLinear(double input) const {
    return lerp(input, config_.input_min, config_.input_max,
                config_.output_min, config_.output_max);
}

double ForceMapper::mapSquareRoot(double input) const {
    // 归一化到[0,1]
    double normalized = (input - config_.input_min) / 
                        (config_.input_max - config_.input_min);
    normalized = clamp(normalized, 0.0, 1.0);
    
    // 应用平方根映射
    double mapped = std::sqrt(normalized);
    
    // 混合线性和平方根映射
    double factor = config_.nonlinearity_factor;
    double result = factor * mapped + (1.0 - factor) * normalized;
    
    // 映射回输出范围
    return config_.output_min + result * (config_.output_max - config_.output_min);
}

double ForceMapper::mapLogarithmic(double input) const {
    // 避免log(0)
    double epsilon = 1e-6;
    double adjusted_input = input + epsilon;
    double adjusted_min = config_.input_min + epsilon;
    double adjusted_max = config_.input_max + epsilon;
    
    // 对数映射
    double log_input = std::log(adjusted_input);
    double log_min = std::log(adjusted_min);
    double log_max = std::log(adjusted_max);
    
    double normalized = (log_input - log_min) / (log_max - log_min);
    normalized = clamp(normalized, 0.0, 1.0);
    
    return config_.output_min + normalized * (config_.output_max - config_.output_min);
}

double ForceMapper::mapPiecewise(double input) const {
    if (calibration_points_.empty()) {
        return mapLinear(input);
    }
    
    if (cache_dirty_) {
        updatePiecewiseCache();
    }
    
    // 找到相邻的校准点
    const CalibrationPoint* lower = nullptr;
    const CalibrationPoint* upper = nullptr;
    
    for (size_t i = 0; i < calibration_points_.size(); ++i) {
        if (calibration_points_[i].input_force <= input) {
            lower = &calibration_points_[i];
        }
        if (calibration_points_[i].input_force >= input && !upper) {
            upper = &calibration_points_[i];
        }
    }
    
    if (!lower) {
        return calibration_points_.front().output_force;
    }
    if (!upper) {
        return calibration_points_.back().output_force;
    }
    if (lower == upper) {
        return lower->output_force;
    }
    
    // 线性插值
    return lerp(input, lower->input_force, upper->input_force,
                lower->output_force, upper->output_force);
}

void ForceMapper::updatePiecewiseCache() const {
    piecewise_cache_.clear();
    for (const auto& point : calibration_points_) {
        piecewise_cache_[point.input_force] = point.output_force;
    }
    cache_dirty_ = false;
}

void ForceMapper::addCalibrationPoint(double input, double output) {
    addCalibrationPoint(CalibrationPoint(input, output));
}

void ForceMapper::addCalibrationPoint(const CalibrationPoint& point) {
    calibration_points_.push_back(point);
    
    // 按输入力排序
    std::sort(calibration_points_.begin(), calibration_points_.end(),
              [](const CalibrationPoint& a, const CalibrationPoint& b) {
                  return a.input_force < b.input_force;
              });
    
    cache_dirty_ = true;
}

void ForceMapper::clearCalibrationPoints() {
    calibration_points_.clear();
    cache_dirty_ = true;
}

void ForceMapper::fitMappingCurve() {
    if (calibration_points_.size() < 2) {
        return;
    }
    
    // 确保有起点和终点
    if (calibration_points_.front().input_force > config_.input_min) {
        addCalibrationPoint(config_.input_min, config_.output_min);
    }
    if (calibration_points_.back().input_force < config_.input_max) {
        addCalibrationPoint(config_.input_max, config_.output_max);
    }
    
    cache_dirty_ = true;
}

void ForceMapper::setConfig(const Config& config) {
    config_ = config;
    cache_dirty_ = true;
}

void ForceMapper::setMappingType(MappingType type) {
    config_.mapping_type = type;
}

double ForceMapper::lerp(double x, double x0, double x1, double y0, double y1) {
    if (std::abs(x1 - x0) < 1e-10) {
        return y0;
    }
    double t = (x - x0) / (x1 - x0);
    t = clamp(t, 0.0, 1.0);
    return y0 + t * (y1 - y0);
}

double ForceMapper::clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

bool ForceMapper::saveCalibration(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }
    
    // 写入配置
    file << "# Force Mapper Calibration Data\n";
    file << "# Format: input_force output_force\n";
    file << "CONFIG input_min " << config_.input_min << "\n";
    file << "CONFIG input_max " << config_.input_max << "\n";
    file << "CONFIG output_min " << config_.output_min << "\n";
    file << "CONFIG output_max " << config_.output_max << "\n";
    file << "\n";
    
    // 写入校准点
    for (const auto& point : calibration_points_) {
        file << point.input_force << " " << point.output_force << "\n";
    }
    
    file.close();
    return true;
}

bool ForceMapper::loadCalibration(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }
    
    calibration_points_.clear();
    
    std::string line;
    while (std::getline(file, line)) {
        // 跳过注释和空行
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        
        if (token == "CONFIG") {
            std::string key;
            double value;
            iss >> key >> value;
            if (key == "input_min") config_.input_min = value;
            else if (key == "input_max") config_.input_max = value;
            else if (key == "output_min") config_.output_min = value;
            else if (key == "output_max") config_.output_max = value;
        } else {
            // 解析校准点
            double input = std::stod(token);
            double output;
            iss >> output;
            calibration_points_.emplace_back(input, output);
        }
    }
    
    file.close();
    cache_dirty_ = true;
    return true;
}

// ==================== ForceScaler ====================

ForceScaler::ForceScaler(double scale_factor)
    : scale_factor_(scale_factor) {
}

} // namespace vss
