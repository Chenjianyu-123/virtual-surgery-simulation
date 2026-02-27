#pragma once

#include "types.h"
#include "instrument.h"

namespace vss {

// 刚体求解器配置
struct RigidBodySolverConfig {
    double linearDamping = 0.99;    // 线性阻尼
    double angularDamping = 0.99;   // 角阻尼
    bool useGravity = false;        // 是否使用重力
    Vector3d gravity = Vector3d(0, -9.81, 0);
};

// 刚体求解器
class RigidBodySolver {
public:
    RigidBodySolver();
    ~RigidBodySolver();
    
    // 设置配置
    void setConfig(const RigidBodySolverConfig& config) { config_ = config; }
    const RigidBodySolverConfig& getConfig() const { return config_; }
    
    // 求解一个时间步
    void solve(Instrument& instrument, double dt);
    
    // 积分刚体状态
    void integrate(RigidBodyState& state, double dt);
    
    // 半隐式欧拉积分
    void integrateSemiImplicit(RigidBodyState& state, double dt);
    
    // 四阶Runge-Kutta积分
    void integrateRK4(RigidBodyState& state, double dt);
    
    // 更新惯性张量
    void updateInertiaTensor(RigidBodyState& state);
    
    // 应用力
    void applyForce(RigidBodyState& state, const Vector3d& force, 
                    const Vector3d& applicationPoint);
    
    // 应用局部力
    void applyLocalForce(RigidBodyState& state, const Vector3d& force,
                         const Vector3d& localPoint);
    
    // 应用力矩
    void applyTorque(RigidBodyState& state, const Vector3d& torque);
    
    // 清除力和力矩
    void clearForces(RigidBodyState& state);
    
private:
    RigidBodySolverConfig config_;
    
    // 内部辅助函数
    void integratePosition(RigidBodyState& state, double dt);
    void integrateOrientation(RigidBodyState& state, double dt);
    void integrateVelocity(RigidBodyState& state, double dt);
    void integrateAngularVelocity(RigidBodyState& state, double dt);
    
    // 计算角动量
    Vector3d computeAngularMomentum(const RigidBodyState& state);
    
    // 归一化四元数
    void normalizeOrientation(RigidBodyState& state);
};

} // namespace vss
