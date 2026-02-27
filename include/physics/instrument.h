#pragma once

#include "types.h"
#include <vector>
#include <memory>

namespace vss {

// 夹持点状态
enum class GraspState {
    Open,       // 打开
    Closing,    // 正在关闭
    Grasping,   // 夹持中
    Opening     // 正在打开
};

// 夹持点
struct GraspPoint {
    int index;
    Vector3d localPosition;     // 局部坐标
    Vector3d worldPosition;     // 世界坐标
    bool isActive;              // 是否激活
    GraspState state;           // 夹持状态
    int graspedSutureParticle;  // 夹持的缝合线质点索引 (-1表示无)
    double graspForce;          // 夹持力
    double maxGraspForce;       // 最大夹持力
    
    GraspPoint() : index(-1), isActive(false), state(GraspState::Open),
                   graspedSutureParticle(-1), graspForce(0.0), maxGraspForce(10.0) {
        localPosition = worldPosition = Vector3d::Zero();
    }
};

// 器械刚体状态
struct RigidBodyState {
    Vector3d position;          // 位置
    Quaterniond orientation;    // 姿态 (四元数)
    Vector3d linearVelocity;    // 线速度
    Vector3d angularVelocity;   // 角速度
    
    Vector3d force;             // 合力
    Vector3d torque;            // 合力矩
    
    double mass;                // 质量
    Matrix3d inertia;           // 惯性张量 (局部坐标)
    Matrix3d invInertia;        // 逆惯性张量
    Matrix3d invInertiaWorld;   // 世界坐标系下的逆惯性张量
    
    RigidBodyState() : mass(1.0) {
        position = linearVelocity = angularVelocity = Vector3d::Zero();
        orientation = Quaterniond::Identity();
        force = torque = Vector3d::Zero();
        inertia = Matrix3d::Identity();
        invInertia = Matrix3d::Identity();
        invInertiaWorld = Matrix3d::Identity();
    }
    
    // 更新世界坐标系下的逆惯性张量
    void updateInvInertiaWorld() {
        Matrix3d R = orientation.toRotationMatrix();
        invInertiaWorld = R * invInertia * R.transpose();
    }
    
    // 局部坐标转世界坐标
    Vector3d localToWorld(const Vector3d& localPoint) const {
        return position + orientation * localPoint;
    }
    
    // 世界坐标转局部坐标
    Vector3d worldToLocal(const Vector3d& worldPoint) const {
        return orientation.inverse() * (worldPoint - position);
    }
    
    // 获取变换矩阵
    Matrix4d getTransformMatrix() const {
        Matrix4d T = Matrix4d::Identity();
        T.block<3, 3>(0, 0) = orientation.toRotationMatrix();
        T.block<3, 1>(0, 3) = position;
        return T;
    }
};

// 器械类
class Instrument {
public:
    Instrument();
    ~Instrument();
    
    // 禁用拷贝，允许移动
    Instrument(const Instrument&) = delete;
    Instrument& operator=(const Instrument&) = delete;
    Instrument(Instrument&&) = default;
    Instrument& operator=(Instrument&&) = default;
    
    // 初始化
    void initialize(const Vector3d& position, const Quaterniond& orientation,
                    double mass, const Matrix3d& inertia);
    
    // 几何设置
    void setGeometry(const std::vector<Vector3d>& vertices,
                     const std::vector<std::array<int, 3>>& faces);
    void addGraspPoint(const Vector3d& localPosition, double maxForce = 10.0);
    
    // 状态获取
    RigidBodyState& getState() { return state_; }
    const RigidBodyState& getState() const { return state_; }
    
    // 夹持点操作
    GraspPoint& getGraspPoint(int index) { return graspPoints_[index]; }
    const GraspPoint& getGraspPoint(int index) const { return graspPoints_[index]; }
    int getGraspPointCount() const { return static_cast<int>(graspPoints_.size()); }
    std::vector<GraspPoint>& getGraspPoints() { return graspPoints_; }
    
    // 夹持控制
    void startGrasp(int graspPointIndex);
    void releaseGrasp(int graspPointIndex);
    void setGraspForce(int graspPointIndex, double force);
    bool isGrasping(int graspPointIndex) const;
    int getGraspedParticle(int graspPointIndex) const;
    
    // 更新夹持点世界坐标
    void updateGraspPoints();
    
    // 力反馈
    void applyExternalForce(const Vector3d& force, const Vector3d& applicationPoint);
    void applyExternalForceLocal(const Vector3d& force, const Vector3d& localPoint);
    void applyExternalTorque(const Vector3d& torque);
    
    // 力反馈输出
    Vector3d getFeedbackForce() const { return feedbackForce_; }
    Vector3d getFeedbackTorque() const { return feedbackTorque_; }
    void setFeedbackForce(const Vector3d& force) { feedbackForce_ = force; }
    void setFeedbackTorque(const Vector3d& torque) { feedbackTorque_ = torque; }
    void clearFeedback() { feedbackForce_ = feedbackTorque_ = Vector3d::Zero(); }
    
    // 物理更新 (半隐式欧拉)
    void integrate(double dt);
    
    // 清除力和力矩
    void clearForces();
    
    // 获取几何信息
    const std::vector<Vector3d>& getVertices() const { return vertices_; }
    const std::vector<std::array<int, 3>>& getFaces() const { return faces_; }
    
    // 获取AABB
    AABB getBoundingBox() const;
    
    // 设置/获取参数
    void setLinearDamping(double damping) { linearDamping_ = damping; }
    void setAngularDamping(double damping) { angularDamping_ = damping; }
    double getLinearDamping() const { return linearDamping_; }
    double getAngularDamping() const { return angularDamping_; }
    
    // 重置
    void reset();
    
    // 与力反馈设备同步
    void syncWithHapticDevice(const Vector3d& devicePosition, const Quaterniond& deviceOrientation);
    void getHapticOutput(Vector3d& force, Vector3d& torque) const;
    
private:
    RigidBodyState state_;
    std::vector<GraspPoint> graspPoints_;
    
    // 几何信息
    std::vector<Vector3d> vertices_;
    std::vector<std::array<int, 3>> faces_;
    
    // 力反馈
    Vector3d feedbackForce_;
    Vector3d feedbackTorque_;
    
    // 阻尼
    double linearDamping_ = 0.99;
    double angularDamping_ = 0.99;
    
    // 内部辅助函数
    void updateInertiaWorld();
    void integrateOrientation(double dt);
};

} // namespace vss
