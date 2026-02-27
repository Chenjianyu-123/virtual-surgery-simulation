#include "rigid_body_solver.h"
#include "math_utils.h"

namespace vss {

// RigidBodySolver 实现

RigidBodySolver::RigidBodySolver() = default;
RigidBodySolver::~RigidBodySolver() = default;

void RigidBodySolver::solve(Instrument& instrument, double dt) {
    auto& state = instrument.getState();
    
    // 应用重力
    if (config_.useGravity) {
        state.force += state.mass * config_.gravity;
    }
    
    // 积分
    integrateSemiImplicit(state, dt);
    
    // 更新夹持点
    instrument.updateGraspPoints();
    
    // 清除力
    clearForces(state);
}

void RigidBodySolver::integrate(RigidBodyState& state, double dt) {
    integrateSemiImplicit(state, dt);
}

void RigidBodySolver::integrateSemiImplicit(RigidBodyState& state, double dt) {
    if (dt <= 0.0) {
        return;
    }
    
    // 更新世界坐标系下的逆惯性张量
    updateInertiaTensor(state);
    
    // 积分线速度
    integrateVelocity(state, dt);
    
    // 积分位置
    integratePosition(state, dt);
    
    // 积分角速度
    integrateAngularVelocity(state, dt);
    
    // 积分姿态
    integrateOrientation(state, dt);
    
    // 归一化四元数
    normalizeOrientation(state);
}

void RigidBodySolver::integrateRK4(RigidBodyState& state, double dt) {
    // 简化的RK4实现
    // 保存初始状态
    RigidBodyState k1 = state;
    
    // k1
    Vector3d v1 = k1.linearVelocity;
    Vector3d omega1 = k1.angularVelocity;
    Vector3d f1 = k1.force / k1.mass;
    Vector3d tau1 = k1.invInertiaWorld * k1.torque;
    
    // k2
    RigidBodyState k2 = state;
    k2.position += 0.5 * dt * v1;
    k2.linearVelocity += 0.5 * dt * f1;
    k2.orientation.coeffs() += 0.5 * dt * math::quaternion::derivative(k2.orientation, omega1).coeffs();
    k2.angularVelocity += 0.5 * dt * tau1;
    
    Vector3d v2 = k2.linearVelocity;
    Vector3d omega2 = k2.angularVelocity;
    
    // k3
    RigidBodyState k3 = state;
    k3.position += 0.5 * dt * v2;
    k3.linearVelocity += 0.5 * dt * f1;
    k3.orientation.coeffs() += 0.5 * dt * math::quaternion::derivative(k3.orientation, omega2).coeffs();
    k3.angularVelocity += 0.5 * dt * tau1;
    
    Vector3d v3 = k3.linearVelocity;
    Vector3d omega3 = k3.angularVelocity;
    
    // k4
    RigidBodyState k4 = state;
    k4.position += dt * v3;
    k4.linearVelocity += dt * f1;
    k4.orientation.coeffs() += dt * math::quaternion::derivative(k4.orientation, omega3).coeffs();
    k4.angularVelocity += dt * tau1;
    
    Vector3d v4 = k4.linearVelocity;
    Vector3d omega4 = k4.angularVelocity;
    
    // 组合
    state.position += (dt / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4);
    state.linearVelocity += dt * f1;
    state.linearVelocity *= config_.linearDamping;
    
    // 姿态更新 (使用角速度平均值)
    Vector3d avgOmega = (omega1 + 2.0 * omega2 + 2.0 * omega3 + omega4) / 6.0;
    state.orientation.coeffs() += dt * math::quaternion::derivative(state.orientation, avgOmega).coeffs();
    state.angularVelocity += dt * tau1;
    state.angularVelocity *= config_.angularDamping;
    
    normalizeOrientation(state);
}

void RigidBodySolver::integratePosition(RigidBodyState& state, double dt) {
    state.position += dt * state.linearVelocity;
}

void RigidBodySolver::integrateOrientation(RigidBodyState& state, double dt) {
    // 使用角速度更新四元数
    // dq/dt = 0.5 * omega * q
    
    Vector3d omega = state.angularVelocity;
    double omegaNorm = omega.norm();
    
    if (omegaNorm < 1e-10) {
        return;
    }
    
    // 计算旋转增量
    double halfAngle = 0.5 * omegaNorm * dt;
    double sinHalfAngle = std::sin(halfAngle);
    double cosHalfAngle = std::cos(halfAngle);
    
    Quaterniond deltaQ;
    deltaQ.w() = cosHalfAngle;
    deltaQ.x() = sinHalfAngle * omega.x() / omegaNorm;
    deltaQ.y() = sinHalfAngle * omega.y() / omegaNorm;
    deltaQ.z() = sinHalfAngle * omega.z() / omegaNorm;
    
    // 更新姿态
    state.orientation = deltaQ * state.orientation;
}

void RigidBodySolver::integrateVelocity(RigidBodyState& state, double dt) {
    // v = v + (F / m) * dt
    state.linearVelocity += (state.force / state.mass) * dt;
    
    // 应用阻尼
    state.linearVelocity *= config_.linearDamping;
}

void RigidBodySolver::integrateAngularVelocity(RigidBodyState& state, double dt) {
    // 计算角动量
    Vector3d angularMomentum = state.inertia * state.angularVelocity;
    
    // 计算力矩 (包括陀螺力矩)
    Vector3d gyroscopicTorque = state.angularVelocity.cross(angularMomentum);
    Vector3d totalTorque = state.torque - gyroscopicTorque;
    
    // omega = omega + I^{-1} * tau * dt
    state.angularVelocity += state.invInertiaWorld * totalTorque * dt;
    
    // 应用阻尼
    state.angularVelocity *= config_.angularDamping;
}

void RigidBodySolver::updateInertiaTensor(RigidBodyState& state) {
    // 更新世界坐标系下的逆惯性张量
    Matrix3d R = state.orientation.toRotationMatrix();
    state.invInertiaWorld = R * state.invInertia * R.transpose();
}

void RigidBodySolver::applyForce(RigidBodyState& state, const Vector3d& force,
                                  const Vector3d& applicationPoint) {
    state.force += force;
    
    // 计算力矩: tau = r x F
    Vector3d r = applicationPoint - state.position;
    state.torque += r.cross(force);
}

void RigidBodySolver::applyLocalForce(RigidBodyState& state, const Vector3d& force,
                                       const Vector3d& localPoint) {
    // 转换到世界坐标
    Vector3d worldForce = state.orientation * force;
    Vector3d worldPoint = state.localToWorld(localPoint);
    
    applyForce(state, worldForce, worldPoint);
}

void RigidBodySolver::applyTorque(RigidBodyState& state, const Vector3d& torque) {
    state.torque += torque;
}

void RigidBodySolver::clearForces(RigidBodyState& state) {
    state.force = Vector3d::Zero();
    state.torque = Vector3d::Zero();
}

Vector3d RigidBodySolver::computeAngularMomentum(const RigidBodyState& state) {
    return state.inertia * state.angularVelocity;
}

void RigidBodySolver::normalizeOrientation(RigidBodyState& state) {
    state.orientation.normalize();
}

} // namespace vss
