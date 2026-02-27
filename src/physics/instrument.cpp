#include "instrument.h"
#include <algorithm>

namespace vss {

Instrument::Instrument() : feedbackForce_(Vector3d::Zero()), feedbackTorque_(Vector3d::Zero()) {}

Instrument::~Instrument() = default;

void Instrument::initialize(const Vector3d& position, const Quaterniond& orientation,
                            double mass, const Matrix3d& inertia) {
    state_.position = position;
    state_.orientation = orientation;
    state_.mass = mass;
    state_.inertia = inertia;
    state_.invInertia = inertia.inverse();
    state_.updateInvInertiaWorld();
    
    state_.linearVelocity = Vector3d::Zero();
    state_.angularVelocity = Vector3d::Zero();
    state_.force = Vector3d::Zero();
    state_.torque = Vector3d::Zero();
}

void Instrument::setGeometry(const std::vector<Vector3d>& vertices,
                              const std::vector<std::array<int, 3>>& faces) {
    vertices_ = vertices;
    faces_ = faces;
}

void Instrument::addGraspPoint(const Vector3d& localPosition, double maxForce) {
    GraspPoint point;
    point.index = static_cast<int>(graspPoints_.size());
    point.localPosition = localPosition;
    point.worldPosition = state_.localToWorld(localPosition);
    point.isActive = true;
    point.state = GraspState::Open;
    point.maxGraspForce = maxForce;
    
    graspPoints_.push_back(point);
}

void Instrument::startGrasp(int graspPointIndex) {
    if (graspPointIndex < 0 || graspPointIndex >= static_cast<int>(graspPoints_.size())) {
        return;
    }
    
    GraspPoint& point = graspPoints_[graspPointIndex];
    if (point.state == GraspState::Open) {
        point.state = GraspState::Closing;
    }
}

void Instrument::releaseGrasp(int graspPointIndex) {
    if (graspPointIndex < 0 || graspPointIndex >= static_cast<int>(graspPoints_.size())) {
        return;
    }
    
    GraspPoint& point = graspPoints_[graspPointIndex];
    if (point.state == GraspState::Grasping || point.state == GraspState::Closing) {
        point.state = GraspState::Opening;
        point.graspedSutureParticle = -1;
        point.graspForce = 0.0;
    }
}

void Instrument::setGraspForce(int graspPointIndex, double force) {
    if (graspPointIndex < 0 || graspPointIndex >= static_cast<int>(graspPoints_.size())) {
        return;
    }
    
    GraspPoint& point = graspPoints_[graspPointIndex];
    point.graspForce = std::max(0.0, std::min(force, point.maxGraspForce));
    
    if (point.state == GraspState::Closing && point.graspForce > 0.1) {
        point.state = GraspState::Grasping;
    } else if (point.state == GraspState::Opening && point.graspForce < 0.01) {
        point.state = GraspState::Open;
    }
}

bool Instrument::isGrasping(int graspPointIndex) const {
    if (graspPointIndex < 0 || graspPointIndex >= static_cast<int>(graspPoints_.size())) {
        return false;
    }
    return graspPoints_[graspPointIndex].state == GraspState::Grasping;
}

int Instrument::getGraspedParticle(int graspPointIndex) const {
    if (graspPointIndex < 0 || graspPointIndex >= static_cast<int>(graspPoints_.size())) {
        return -1;
    }
    return graspPoints_[graspPointIndex].graspedSutureParticle;
}

void Instrument::updateGraspPoints() {
    for (auto& point : graspPoints_) {
        point.worldPosition = state_.localToWorld(point.localPosition);
    }
}

void Instrument::applyExternalForce(const Vector3d& force, const Vector3d& applicationPoint) {
    state_.force += force;
    state_.torque += (applicationPoint - state_.position).cross(force);
}

void Instrument::applyExternalForceLocal(const Vector3d& force, const Vector3d& localPoint) {
    Vector3d worldPoint = state_.localToWorld(localPoint);
    applyExternalForce(force, worldPoint);
}

void Instrument::applyExternalTorque(const Vector3d& torque) {
    state_.torque += torque;
}

void Instrument::integrate(double dt) {
    if (dt <= 0.0) {
        return;
    }
    
    // 更新世界坐标系下的逆惯性张量
    updateInertiaWorld();
    
    // 半隐式欧拉积分 - 线速度
    state_.linearVelocity += dt * state_.force / state_.mass;
    state_.linearVelocity *= linearDamping_;
    
    // 更新位置
    state_.position += dt * state_.linearVelocity;
    
    // 半隐式欧拉积分 - 角速度
    Vector3d angularMomentum = state_.inertia * state_.angularVelocity;
    Vector3d torque = state_.torque - state_.angularVelocity.cross(angularMomentum);
    state_.angularVelocity += dt * state_.invInertiaWorld * torque;
    state_.angularVelocity *= angularDamping_;
    
    // 更新姿态 (四元数)
    integrateOrientation(dt);
    
    // 更新夹持点位置
    updateGraspPoints();
}

void Instrument::integrateOrientation(double dt) {
    // 使用四元数微分更新姿态
    // dq/dt = 0.5 * omega * q
    
    Vector3d omega = state_.angularVelocity;
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
    state_.orientation = deltaQ * state_.orientation;
    state_.orientation.normalize();
}

void Instrument::clearForces() {
    state_.force = Vector3d::Zero();
    state_.torque = Vector3d::Zero();
}

AABB Instrument::getBoundingBox() const {
    if (vertices_.empty()) {
        return AABB(state_.position, state_.position);
    }
    
    AABB bbox;
    bool first = true;
    
    for (const auto& vertex : vertices_) {
        Vector3d worldVertex = state_.localToWorld(vertex);
        if (first) {
            bbox.min = bbox.max = worldVertex;
            first = false;
        } else {
            bbox.expand(worldVertex);
        }
    }
    
    return bbox;
}

void Instrument::reset() {
    state_.linearVelocity = Vector3d::Zero();
    state_.angularVelocity = Vector3d::Zero();
    state_.force = Vector3d::Zero();
    state_.torque = Vector3d::Zero();
    
    for (auto& point : graspPoints_) {
        point.state = GraspState::Open;
        point.graspedSutureParticle = -1;
        point.graspForce = 0.0;
    }
    
    clearFeedback();
}

void Instrument::syncWithHapticDevice(const Vector3d& devicePosition, const Quaterniond& deviceOrientation) {
    // 将设备位置和姿态同步到器械
    // 这里可以实现阻抗控制或导纳控制
    
    // 计算位置误差
    Vector3d positionError = devicePosition - state_.position;
    
    // 计算姿态误差
    Quaterniond orientationError = deviceOrientation * state_.orientation.inverse();
    
    // 简单的PD控制
    double kp = 1000.0;  // 位置刚度
    double kd = 50.0;    // 位置阻尼
    
    Vector3d desiredForce = kp * positionError - kd * state_.linearVelocity;
    
    // 设置力反馈输出
    feedbackForce_ = -desiredForce;
    
    // 更新器械状态
    state_.position = devicePosition;
    state_.orientation = deviceOrientation;
    state_.linearVelocity = Vector3d::Zero();
    state_.angularVelocity = Vector3d::Zero();
    
    updateGraspPoints();
}

void Instrument::getHapticOutput(Vector3d& force, Vector3d& torque) const {
    force = feedbackForce_;
    torque = feedbackTorque_;
}

void Instrument::updateInertiaWorld() {
    state_.updateInvInertiaWorld();
}

} // namespace vss
