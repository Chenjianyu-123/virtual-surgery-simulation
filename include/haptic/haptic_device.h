#pragma once

#include "types.h"
#include <string>
#include <functional>

namespace vss {

// 力反馈设备类型
enum class HapticDeviceType {
    Unknown,
    PhantomOmni,        // Geomagic Touch (Phantom Omni)
    PhantomDesktop,     // Geomagic Touch Desktop
    Falcon,             // Novint Falcon
    Omega,              // Force Dimension Omega
n    Custom              // 自定义设备
};

// 设备状态
struct HapticDeviceState {
    Vector3d position;          // 位置 (m)
    Quaterniond orientation;    // 姿态
    Vector3d velocity;          // 线速度 (m/s)
    Vector3d angularVelocity;   // 角速度 (rad/s)
    
    bool button1;               // 按钮1状态
    bool button2;               // 按钮2状态
    bool button3;               // 按钮3状态
    
    double timestamp;           // 时间戳
    
    HapticDeviceState() : button1(false), button2(false), button3(false), timestamp(0.0) {
        position = velocity = angularVelocity = Vector3d::Zero();
        orientation = Quaterniond::Identity();
    }
};

// 设备能力
struct HapticDeviceCapabilities {
    double maxForce;            // 最大输出力 (N)
    double maxTorque;           // 最大输出力矩 (N·m)
    double maxStiffness;        // 最大刚度 (N/m)
    double maxDamping;          // 最大阻尼 (N·s/m)
    double workspaceRadius;     // 工作空间半径 (m)
    double positionResolution;  // 位置分辨率 (m)
    double forceResolution;     // 力分辨率 (N)
    double updateRate;          // 更新率 (Hz)
    int numButtons;             // 按钮数量
    bool supportsForce;         // 是否支持力输出
    bool supportsTorque;        // 是否支持力矩输出
    bool supportsOrientation;   // 是否支持姿态输入
    
    HapticDeviceCapabilities()
        : maxForce(10.0), maxTorque(1.0), maxStiffness(1000.0),
          maxDamping(100.0), workspaceRadius(0.1), positionResolution(1e-4),
          forceResolution(0.01), updateRate(1000.0), numButtons(2),
          supportsForce(true), supportsTorque(false), supportsOrientation(false) {}
};

// 力反馈设备基类
class HapticDevice {
public:
    HapticDevice();
    virtual ~HapticDevice();
    
    // 禁止拷贝，允许移动
    HapticDevice(const HapticDevice&) = delete;
    HapticDevice& operator=(const HapticDevice&) = delete;
    HapticDevice(HapticDevice&&) = default;
    HapticDevice& operator=(HapticDevice&&) = default;
    
    // 初始化
    virtual bool initialize(int deviceIndex = 0) = 0;
    virtual void shutdown() = 0;
    virtual bool isInitialized() const = 0;
    
    // 获取设备信息
    virtual HapticDeviceType getType() const = 0;
    virtual std::string getDeviceName() const = 0;
    virtual HapticDeviceCapabilities getCapabilities() const = 0;
    
    // 状态读取
    virtual bool getState(HapticDeviceState& state) = 0;
    virtual bool getPosition(Vector3d& position) = 0;
    virtual bool getOrientation(Quaterniond& orientation) = 0;
    virtual bool getVelocity(Vector3d& velocity) = 0;
    virtual bool getButtonState(int buttonIndex, bool& pressed) = 0;
    
    // 力反馈输出
    virtual bool setForce(const Vector3d& force) = 0;
    virtual bool setTorque(const Vector3d& torque) = 0;
    virtual bool setForceAndTorque(const Vector3d& force, const Vector3d& torque) = 0;
    
    // 零位校准
    virtual bool calibrate() = 0;
    virtual bool setZeroPosition() = 0;
    
    // 安全限制
    virtual void setForceLimit(double maxForce) { maxForceLimit_ = maxForce; }
    virtual void setStiffnessLimit(double maxStiffness) { maxStiffnessLimit_ = maxStiffness; }
    
    double getForceLimit() const { return maxForceLimit_; }
    double getStiffnessLimit() const { return maxStiffnessLimit_; }
    
    // 紧急停止
    virtual void emergencyStop() = 0;
    virtual void resume() = 0;
    virtual bool isInEmergencyStop() const = 0;
    
    // 回调函数
    using StateCallback = std::function<void(const HapticDeviceState&)>;
    using ButtonCallback = std::function<void(int, bool)>;
    
    void setStateCallback(StateCallback callback) { stateCallback_ = callback; }
    void setButtonCallback(ButtonCallback callback) { buttonCallback_ = callback; }
    
protected:
    double maxForceLimit_ = 10.0;
    double maxStiffnessLimit_ = 1000.0;
    
    StateCallback stateCallback_;
    ButtonCallback buttonCallback_;
};

// 虚拟力反馈设备 (用于测试)
class VirtualHapticDevice : public HapticDevice {
public:
    VirtualHapticDevice();
    ~VirtualHapticDevice() override;
    
    bool initialize(int deviceIndex = 0) override;
    void shutdown() override;
    bool isInitialized() const override { return initialized_; }
    
    HapticDeviceType getType() const override { return HapticDeviceType::Custom; }
    std::string getDeviceName() const override { return "Virtual Haptic Device"; }
    HapticDeviceCapabilities getCapabilities() const override { return capabilities_; }
    
    bool getState(HapticDeviceState& state) override;
    bool getPosition(Vector3d& position) override;
    bool getOrientation(Quaterniond& orientation) override;
    bool getVelocity(Vector3d& velocity) override;
    bool getButtonState(int buttonIndex, bool& pressed) override;
    
    bool setForce(const Vector3d& force) override;
    bool setTorque(const Vector3d& torque) override;
    bool setForceAndTorque(const Vector3d& force, const Vector3d& torque) override;
    
    bool calibrate() override;
    bool setZeroPosition() override;
    
    void emergencyStop() override;
    void resume() override;
    bool isInEmergencyStop() const override { return emergencyStop_; }
    
    // 虚拟设备特有：设置输入状态
    void setVirtualPosition(const Vector3d& position);
    void setVirtualOrientation(const Quaterniond& orientation);
    void setVirtualButton(int buttonIndex, bool pressed);
    
    // 获取输出的力
    Vector3d getOutputForce() const { return outputForce_; }
    Vector3d getOutputTorque() const { return outputTorque_; }
    
private:
    bool initialized_ = false;
    bool emergencyStop_ = false;
    
    HapticDeviceState state_;
    HapticDeviceCapabilities capabilities_;
    
    Vector3d outputForce_;
    Vector3d outputTorque_;
    
    Vector3d zeroPosition_;
    Quaterniond zeroOrientation_;
};

// 力反馈设备管理器
class HapticDeviceManager {
public:
    HapticDeviceManager();
    ~HapticDeviceManager();
    
    // 创建设备
    std::shared_ptr<HapticDevice> createDevice(HapticDeviceType type);
    
    // 枚举可用设备
    std::vector<std::pair<int, std::string>> enumerateDevices();
    
    // 获取默认设备
    std::shared_ptr<HapticDevice> getDefaultDevice();
    
    // 设置活动设备
    void setActiveDevice(std::shared_ptr<HapticDevice> device);
    std::shared_ptr<HapticDevice> getActiveDevice() const { return activeDevice_; }
    
private:
    std::shared_ptr<HapticDevice> activeDevice_;
    std::vector<std::shared_ptr<HapticDevice>> devices_;
};

} // namespace vss
