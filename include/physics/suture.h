#pragma once

#include "types.h"
#include <vector>
#include <memory>

namespace vss {

// 前向声明
class Constraint;

// 缝合线质点
struct SutureParticle {
    int index;
    Vector3d position;
    Vector3d prevPosition;      // 上一帧位置 (用于XPBD)
    Vector3d velocity;
    Vector3d force;
    double mass;
    double radius;
    double inverseMass;
    bool isFixed;
    
    SutureParticle() : index(-1), mass(1.0), radius(0.001), 
                       inverseMass(1.0), isFixed(false) {
        position = prevPosition = velocity = force = Vector3d::Zero();
    }
};

// 约束基类
class Constraint {
public:
    virtual ~Constraint() = default;
    
    // 求解约束，返回lambda增量
    virtual double solve(std::vector<SutureParticle>& particles, double dt) = 0;
    
    // 评估约束违反程度
    virtual double evaluate(const std::vector<SutureParticle>& particles) const = 0;
    
    // 获取约束类型
    virtual const char* getType() const = 0;
    
    // 获取涉及的粒子索引
    virtual std::vector<int> getParticleIndices() const = 0;
    
    double stiffness = 1.0;
    double compliance = 0.0;  // 柔度 = 1/stiffness
};

// 距离约束
class DistanceConstraint : public Constraint {
public:
    DistanceConstraint(int p1, int p2, double restLength, double stiffness = 1.0);
    
    double solve(std::vector<SutureParticle>& particles, double dt) override;
    double evaluate(const std::vector<SutureParticle>& particles) const override;
    const char* getType() const override { return "Distance"; }
    std::vector<int> getParticleIndices() const override { return {particle1, particle2}; }
    
    int particle1, particle2;
    double restLength;
};

// 弯曲约束
class BendingConstraint : public Constraint {
public:
    BendingConstraint(int p1, int p2, int p3, double restAngle, double stiffness = 1.0);
    
    double solve(std::vector<SutureParticle>& particles, double dt) override;
    double evaluate(const std::vector<SutureParticle>& particles) const override;
    const char* getType() const override { return "Bending"; }
    std::vector<int> getParticleIndices() const override { return {particle1, particle2, particle3}; }
    
    int particle1, particle2, particle3;
    double restAngle;
};

// 锚定约束 (将粒子固定到世界空间某点)
class AnchorConstraint : public Constraint {
public:
    AnchorConstraint(int particle, const Vector3d& anchorPos, double stiffness = 1.0);
    
    double solve(std::vector<SutureParticle>& particles, double dt) override;
    double evaluate(const std::vector<SutureParticle>& particles) const override;
    const char* getType() const override { return "Anchor"; }
    std::vector<int> getParticleIndices() const override { return {particle}; }
    
    int particle;
    Vector3d anchorPosition;
};

// 碰撞约束 (动态生成)
class CollisionConstraint : public Constraint {
public:
    CollisionConstraint(int particle, const Vector3d& contactPoint, 
                        const Vector3d& normal, double penetration, double stiffness = 1.0);
    
    double solve(std::vector<SutureParticle>& particles, double dt) override;
    double evaluate(const std::vector<SutureParticle>& particles) const override;
    const char* getType() const override { return "Collision"; }
    std::vector<int> getParticleIndices() const override { return {particle}; }
    
    int particle;
    Vector3d contactPoint;
    Vector3d normal;
    double penetration;
};

// 缝合线类
class Suture {
public:
    Suture();
    ~Suture();
    
    // 禁用拷贝，允许移动
    Suture(const Suture&) = delete;
    Suture& operator=(const Suture&) = delete;
    Suture(Suture&&) = default;
    Suture& operator=(Suture&&) = default;
    
    // 质点管理
    int addParticle(const Vector3d& position, double mass = 0.01, double radius = 0.001);
    void removeParticle(int index);
    SutureParticle& getParticle(int index) { return particles_[index]; }
    const SutureParticle& getParticle(int index) const { return particles_[index]; }
    int getParticleCount() const { return static_cast<int>(particles_.size()); }
    std::vector<SutureParticle>& getParticles() { return particles_; }
    const std::vector<SutureParticle>& getParticles() const { return particles_; }
    
    // 约束管理
    void addDistanceConstraint(int p1, int p2, double restLength = -1.0, double stiffness = 1.0);
    void addBendingConstraint(int p1, int p2, int p3, double restAngle = 0.0, double stiffness = 0.5);
    void addAnchorConstraint(int particle, const Vector3d& anchorPos, double stiffness = 1.0);
    void addCollisionConstraint(int particle, const Vector3d& contactPoint,
                                 const Vector3d& normal, double penetration);
    void removeConstraint(int index);
    void clearConstraints();
    void clearCollisionConstraints();  // 只清除碰撞约束
    
    std::shared_ptr<Constraint> getConstraint(int index) { return constraints_[index]; }
    std::shared_ptr<Constraint> getConstraint(int index) const { return constraints_[index]; }
    int getConstraintCount() const { return static_cast<int>(constraints_.size()); }
    const std::vector<std::shared_ptr<Constraint>>& getConstraints() const { return constraints_; }
    
    // 创建链式结构
    void createChain(const std::vector<Vector3d>& positions, double totalMass = 0.1, 
                     double radius = 0.001);
    
    // XPBD求解
    void simulate(double dt);
    void solveConstraints(double dt);
    void updateVelocities(double dt);
    
    // 物理计算
    void applyForce(int particleIndex, const Vector3d& force);
    void applyGlobalForce(const Vector3d& force);  // 如重力
    void dampVelocities(double damping);
    
    // 获取系统信息
    double getTotalLength() const;
    double getTotalMass() const;
    Vector3d getCenterOfMass() const;
    AABB getBoundingBox() const;
    
    // 锚定操作
    void anchorParticle(int particleIndex, const Vector3d& position);
    void unanchorParticle(int particleIndex);
    void moveAnchor(int particleIndex, const Vector3d& newPosition);
    
    // 重置
    void reset();
    
    // 参数设置
    void setSolverIterations(int iterations) { solverIterations_ = iterations; }
    void setDamping(double damping) { damping_ = damping; }
    void setGravity(const Vector3d& gravity) { gravity_ = gravity; }
    
    int getSolverIterations() const { return solverIterations_; }
    double getDamping() const { return damping_; }
    Vector3d getGravity() const { return gravity_; }
    
private:
    std::vector<SutureParticle> particles_;
    std::vector<std::shared_ptr<Constraint>> constraints_;
    
    // XPBD参数
    int solverIterations_ = 10;
    double damping_ = 0.99;
    Vector3d gravity_ = Vector3d(0, -9.81, 0);
    
    // 内部辅助函数
    void updateInverseMasses();
    bool validateParticleIndex(int index) const;
};

} // namespace vss
