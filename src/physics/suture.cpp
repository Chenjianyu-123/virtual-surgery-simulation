#include "suture.h"
#include <algorithm>
#include <cmath>

namespace vss {

// DistanceConstraint 实现
DistanceConstraint::DistanceConstraint(int p1, int p2, double restLength, double stiffness)
    : particle1(p1), particle2(p2), restLength(restLength) {
    this->stiffness = stiffness;
    this->compliance = stiffness > 0 ? 1.0 / stiffness : 0.0;
}

double DistanceConstraint::solve(std::vector<SutureParticle>& particles, double dt) {
    if (particle1 < 0 || particle1 >= static_cast<int>(particles.size()) ||
        particle2 < 0 || particle2 >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    SutureParticle& p1 = particles[particle1];
    SutureParticle& p2 = particles[particle2];
    
    if (p1.isFixed && p2.isFixed) {
        return 0.0;
    }
    
    Vector3d delta = p2.position - p1.position;
    double dist = delta.norm();
    
    if (dist < 1e-10) {
        return 0.0;
    }
    
    double C = dist - restLength;
    Vector3d n = delta / dist;
    
    double w1 = p1.inverseMass;
    double w2 = p2.inverseMass;
    double wSum = w1 + w2;
    
    if (wSum < 1e-10) {
        return 0.0;
    }
    
    // XPBD: 考虑柔度
    double alpha = compliance / (dt * dt);
    double lambda = -C / (wSum + alpha);
    
    Vector3d correction = lambda * n;
    
    if (!p1.isFixed) {
        p1.position -= w1 * correction;
    }
    if (!p2.isFixed) {
        p2.position += w2 * correction;
    }
    
    return std::abs(C);
}

double DistanceConstraint::evaluate(const std::vector<SutureParticle>& particles) const {
    if (particle1 < 0 || particle1 >= static_cast<int>(particles.size()) ||
        particle2 < 0 || particle2 >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    const SutureParticle& p1 = particles[particle1];
    const SutureParticle& p2 = particles[particle2];
    
    double dist = (p2.position - p1.position).norm();
    return std::abs(dist - restLength);
}

// BendingConstraint 实现
BendingConstraint::BendingConstraint(int p1, int p2, int p3, double restAngle, double stiffness)
    : particle1(p1), particle2(p2), particle3(p3), restAngle(restAngle) {
    this->stiffness = stiffness;
    this->compliance = stiffness > 0 ? 1.0 / stiffness : 0.0;
}

double BendingConstraint::solve(std::vector<SutureParticle>& particles, double dt) {
    if (particle1 < 0 || particle1 >= static_cast<int>(particles.size()) ||
        particle2 < 0 || particle2 >= static_cast<int>(particles.size()) ||
        particle3 < 0 || particle3 >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    SutureParticle& p1 = particles[particle1];
    SutureParticle& p2 = particles[particle2];
    SutureParticle& p3 = particles[particle3];
    
    if (p1.isFixed && p2.isFixed && p3.isFixed) {
        return 0.0;
    }
    
    Vector3d e1 = p2.position - p1.position;
    Vector3d e2 = p3.position - p2.position;
    
    double len1 = e1.norm();
    double len2 = e2.norm();
    
    if (len1 < 1e-10 || len2 < 1e-10) {
        return 0.0;
    }
    
    e1 /= len1;
    e2 /= len2;
    
    double dot = e1.dot(e2);
    dot = std::max(-1.0, std::min(1.0, dot));
    double angle = std::acos(dot);
    
    double C = angle - restAngle;
    
    if (std::abs(C) < 1e-10) {
        return 0.0;
    }
    
    // 简化处理：直接调整中间粒子
    Vector3d axis = e1.cross(e2);
    if (axis.norm() < 1e-10) {
        return 0.0;
    }
    axis.normalize();
    
    double wSum = p1.inverseMass + 2.0 * p2.inverseMass + p3.inverseMass;
    if (wSum < 1e-10) {
        return 0.0;
    }
    
    double alpha = compliance / (dt * dt);
    double lambda = -C / (wSum + alpha);
    
    // 应用修正
    if (!p1.isFixed) {
        p1.position += 0.5 * lambda * axis * len1 * p1.inverseMass;
    }
    if (!p2.isFixed) {
        p2.position -= lambda * axis * len1 * p2.inverseMass;
        p2.position += lambda * axis * len2 * p2.inverseMass;
    }
    if (!p3.isFixed) {
        p3.position -= 0.5 * lambda * axis * len2 * p3.inverseMass;
    }
    
    return std::abs(C);
}

double BendingConstraint::evaluate(const std::vector<SutureParticle>& particles) const {
    if (particle1 < 0 || particle1 >= static_cast<int>(particles.size()) ||
        particle2 < 0 || particle2 >= static_cast<int>(particles.size()) ||
        particle3 < 0 || particle3 >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    const SutureParticle& p1 = particles[particle1];
    const SutureParticle& p2 = particles[particle2];
    const SutureParticle& p3 = particles[particle3];
    
    Vector3d e1 = (p2.position - p1.position).normalized();
    Vector3d e2 = (p3.position - p2.position).normalized();
    
    double dot = std::max(-1.0, std::min(1.0, e1.dot(e2)));
    double angle = std::acos(dot);
    
    return std::abs(angle - restAngle);
}

// AnchorConstraint 实现
AnchorConstraint::AnchorConstraint(int particle, const Vector3d& anchorPos, double stiffness)
    : particle(particle), anchorPosition(anchorPos) {
    this->stiffness = stiffness;
    this->compliance = stiffness > 0 ? 1.0 / stiffness : 0.0;
}

double AnchorConstraint::solve(std::vector<SutureParticle>& particles, double dt) {
    if (particle < 0 || particle >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    SutureParticle& p = particles[particle];
    
    if (p.isFixed) {
        p.position = anchorPosition;
        return 0.0;
    }
    
    Vector3d delta = anchorPosition - p.position;
    double dist = delta.norm();
    
    if (dist < 1e-10) {
        return 0.0;
    }
    
    double alpha = compliance / (dt * dt);
    double lambda = dist / (p.inverseMass + alpha);
    
    p.position += lambda * p.inverseMass * delta / dist;
    
    return dist;
}

double AnchorConstraint::evaluate(const std::vector<SutureParticle>& particles) const {
    if (particle < 0 || particle >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    return (particles[particle].position - anchorPosition).norm();
}

// CollisionConstraint 实现
CollisionConstraint::CollisionConstraint(int particle, const Vector3d& contactPoint,
                                          const Vector3d& normal, double penetration, double stiffness)
    : particle(particle), contactPoint(contactPoint), normal(normal), penetration(penetration) {
    this->stiffness = stiffness;
    this->compliance = 0.0;  // 碰撞约束通常是硬约束
}

double CollisionConstraint::solve(std::vector<SutureParticle>& particles, double dt) {
    if (particle < 0 || particle >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    SutureParticle& p = particles[particle];
    
    if (p.isFixed) {
        return 0.0;
    }
    
    // 计算粒子到接触平面的距离
    double dist = (p.position - contactPoint).dot(normal);
    
    if (dist >= 0) {
        return 0.0;  // 没有穿透
    }
    
    // 将粒子推出穿透区域
    Vector3d correction = -dist * normal;
    p.position += correction;
    
    return -dist;
}

double CollisionConstraint::evaluate(const std::vector<SutureParticle>& particles) const {
    if (particle < 0 || particle >= static_cast<int>(particles.size())) {
        return 0.0;
    }
    
    const SutureParticle& p = particles[particle];
    double dist = (p.position - contactPoint).dot(normal);
    
    return std::max(0.0, -dist);  // 返回穿透深度
}

// Suture 类实现
Suture::Suture() = default;
Suture::~Suture() = default;

int Suture::addParticle(const Vector3d& position, double mass, double radius) {
    SutureParticle particle;
    particle.index = static_cast<int>(particles_.size());
    particle.position = position;
    particle.prevPosition = position;
    particle.velocity = Vector3d::Zero();
    particle.force = Vector3d::Zero();
    particle.mass = mass;
    particle.radius = radius;
    particle.inverseMass = mass > 0 ? 1.0 / mass : 0.0;
    particle.isFixed = false;
    
    particles_.push_back(particle);
    return particle.index;
}

void Suture::removeParticle(int index) {
    if (index < 0 || index >= static_cast<int>(particles_.size())) {
        return;
    }
    
    particles_.erase(particles_.begin() + index);
    
    // 更新剩余粒子的索引
    for (int i = index; i < static_cast<int>(particles_.size()); ++i) {
        particles_[i].index = i;
    }
    
    // 移除涉及该粒子的约束
    constraints_.erase(
        std::remove_if(constraints_.begin(), constraints_.end(),
            [index](const std::shared_ptr<Constraint>& c) {
                auto indices = c->getParticleIndices();
                return std::find(indices.begin(), indices.end(), index) != indices.end();
            }),
        constraints_.end()
    );
    
    // 更新约束中的粒子索引
    for (auto& constraint : constraints_) {
        auto indices = constraint->getParticleIndices();
        for (int& idx : indices) {
            if (idx > index) {
                idx--;
            }
        }
    }
}

void Suture::addDistanceConstraint(int p1, int p2, double restLength, double stiffness) {
    if (!validateParticleIndex(p1) || !validateParticleIndex(p2)) {
        return;
    }
    
    if (restLength < 0) {
        // 使用当前距离作为静止长度
        restLength = (particles_[p2].position - particles_[p1].position).norm();
    }
    
    constraints_.push_back(std::make_shared<DistanceConstraint>(p1, p2, restLength, stiffness));
}

void Suture::addBendingConstraint(int p1, int p2, int p3, double restAngle, double stiffness) {
    if (!validateParticleIndex(p1) || !validateParticleIndex(p2) || !validateParticleIndex(p3)) {
        return;
    }
    
    constraints_.push_back(std::make_shared<BendingConstraint>(p1, p2, p3, restAngle, stiffness));
}

void Suture::addAnchorConstraint(int particle, const Vector3d& anchorPos, double stiffness) {
    if (!validateParticleIndex(particle)) {
        return;
    }
    
    constraints_.push_back(std::make_shared<AnchorConstraint>(particle, anchorPos, stiffness));
}

void Suture::addCollisionConstraint(int particle, const Vector3d& contactPoint,
                                     const Vector3d& normal, double penetration) {
    if (!validateParticleIndex(particle)) {
        return;
    }
    
    constraints_.push_back(std::make_shared<CollisionConstraint>(
        particle, contactPoint, normal, penetration));
}

void Suture::removeConstraint(int index) {
    if (index < 0 || index >= static_cast<int>(constraints_.size())) {
        return;
    }
    
    constraints_.erase(constraints_.begin() + index);
}

void Suture::clearConstraints() {
    constraints_.clear();
}

void Suture::clearCollisionConstraints() {
    constraints_.erase(
        std::remove_if(constraints_.begin(), constraints_.end(),
            [](const std::shared_ptr<Constraint>& c) {
                return std::string(c->getType()) == "Collision";
            }),
        constraints_.end()
    );
}

void Suture::createChain(const std::vector<Vector3d>& positions, double totalMass, double radius) {
    // 清空现有数据
    particles_.clear();
    constraints_.clear();
    
    if (positions.empty()) {
        return;
    }
    
    double particleMass = totalMass / positions.size();
    
    // 创建粒子
    for (const auto& pos : positions) {
        addParticle(pos, particleMass, radius);
    }
    
    // 创建距离约束（链式连接）
    for (int i = 0; i < static_cast<int>(positions.size()) - 1; ++i) {
        double restLength = (positions[i + 1] - positions[i]).norm();
        addDistanceConstraint(i, i + 1, restLength, 1.0);
    }
    
    // 创建弯曲约束（每三个连续粒子）
    for (int i = 0; i < static_cast<int>(positions.size()) - 2; ++i) {
        addBendingConstraint(i, i + 1, i + 2, 0.0, 0.5);
    }
}

void Suture::simulate(double dt) {
    // 预测位置
    for (auto& particle : particles_) {
        if (!particle.isFixed) {
            particle.prevPosition = particle.position;
            
            // 应用重力
            particle.velocity += gravity_ * dt;
            
            // 应用外力
            particle.velocity += particle.force * particle.inverseMass * dt;
            
            // 阻尼
            particle.velocity *= damping_;
            
            // 更新位置
            particle.position += particle.velocity * dt;
            
            // 清除外力
            particle.force = Vector3d::Zero();
        }
    }
    
    // 求解约束
    solveConstraints(dt);
    
    // 更新速度
    updateVelocities(dt);
}

void Suture::solveConstraints(double dt) {
    for (int iter = 0; iter < solverIterations_; ++iter) {
        for (auto& constraint : constraints_) {
            constraint->solve(particles_, dt);
        }
    }
}

void Suture::updateVelocities(double dt) {
    if (dt < 1e-10) {
        return;
    }
    
    for (auto& particle : particles_) {
        if (!particle.isFixed) {
            particle.velocity = (particle.position - particle.prevPosition) / dt;
        }
    }
}

void Suture::applyForce(int particleIndex, const Vector3d& force) {
    if (validateParticleIndex(particleIndex)) {
        particles_[particleIndex].force += force;
    }
}

void Suture::applyGlobalForce(const Vector3d& force) {
    for (auto& particle : particles_) {
        particle.force += force;
    }
}

void Suture::dampVelocities(double damping) {
    for (auto& particle : particles_) {
        particle.velocity *= damping;
    }
}

double Suture::getTotalLength() const {
    double length = 0.0;
    for (size_t i = 0; i < particles_.size() - 1; ++i) {
        length += (particles_[i + 1].position - particles_[i].position).norm();
    }
    return length;
}

double Suture::getTotalMass() const {
    double mass = 0.0;
    for (const auto& particle : particles_) {
        mass += particle.mass;
    }
    return mass;
}

Vector3d Suture::getCenterOfMass() const {
    Vector3d center = Vector3d::Zero();
    double totalMass = 0.0;
    
    for (const auto& particle : particles_) {
        center += particle.mass * particle.position;
        totalMass += particle.mass;
    }
    
    if (totalMass > 0.0) {
        center /= totalMass;
    }
    
    return center;
}

AABB Suture::getBoundingBox() const {
    if (particles_.empty()) {
        return AABB();
    }
    
    AABB bbox;
    bbox.min = bbox.max = particles_[0].position;
    
    for (const auto& particle : particles_) {
        bbox.expand(particle.position);
    }
    
    // 考虑粒子半径
    Vector3d radiusVec(particles_[0].radius, particles_[0].radius, particles_[0].radius);
    bbox.min -= radiusVec;
    bbox.max += radiusVec;
    
    return bbox;
}

void Suture::anchorParticle(int particleIndex, const Vector3d& position) {
    if (!validateParticleIndex(particleIndex)) {
        return;
    }
    
    particles_[particleIndex].isFixed = true;
    particles_[particleIndex].position = position;
    particles_[particleIndex].velocity = Vector3d::Zero();
    
    // 添加或更新锚定约束
    bool found = false;
    for (auto& constraint : constraints_) {
        if (std::string(constraint->getType()) == "Anchor") {
            auto anchor = std::static_pointer_cast<AnchorConstraint>(constraint);
            if (anchor->particle == particleIndex) {
                anchor->anchorPosition = position;
                found = true;
                break;
            }
        }
    }
    
    if (!found) {
        addAnchorConstraint(particleIndex, position, 1.0);
    }
}

void Suture::unanchorParticle(int particleIndex) {
    if (!validateParticleIndex(particleIndex)) {
        return;
    }
    
    particles_[particleIndex].isFixed = false;
    
    // 移除对应的锚定约束
    constraints_.erase(
        std::remove_if(constraints_.begin(), constraints_.end(),
            [particleIndex](const std::shared_ptr<Constraint>& c) {
                if (std::string(c->getType()) == "Anchor") {
                    auto anchor = std::static_pointer_cast<AnchorConstraint>(c);
                    return anchor->particle == particleIndex;
                }
                return false;
            }),
        constraints_.end()
    );
}

void Suture::moveAnchor(int particleIndex, const Vector3d& newPosition) {
    if (!validateParticleIndex(particleIndex)) {
        return;
    }
    
    particles_[particleIndex].position = newPosition;
    
    // 更新锚定约束
    for (auto& constraint : constraints_) {
        if (std::string(constraint->getType()) == "Anchor") {
            auto anchor = std::static_pointer_cast<AnchorConstraint>(constraint);
            if (anchor->particle == particleIndex) {
                anchor->anchorPosition = newPosition;
                break;
            }
        }
    }
}

void Suture::reset() {
    for (auto& particle : particles_) {
        particle.velocity = Vector3d::Zero();
        particle.force = Vector3d::Zero();
    }
}

void Suture::updateInverseMasses() {
    for (auto& particle : particles_) {
        particle.inverseMass = particle.mass > 0 ? 1.0 / particle.mass : 0.0;
    }
}

bool Suture::validateParticleIndex(int index) const {
    return index >= 0 && index < static_cast<int>(particles_.size());
}

} // namespace vss
