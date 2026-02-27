#include "constraint_projector.h"
#include <algorithm>

namespace vss {

// ConstraintProjector 实现

ConstraintProjector::ConstraintProjector() = default;
ConstraintProjector::~ConstraintProjector() = default;

void ConstraintProjector::projectConstraints(
    Tissue& tissue,
    Suture& suture,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    // 1. 关键约束迭代至收敛
    for (int i = 0; i < config_.criticalIterations; ++i) {
        double error = projectCriticalConstraints(tissue, suture, instrument, contacts, dt);
        if (error < config_.tolerance) {
            break;
        }
    }
    
    // 2. 非关键约束投影
    projectNonCriticalConstraints(suture, dt);
    
    // 3. 局部修正关键组
    for (int i = 0; i < config_.localCorrectionPasses; ++i) {
        localCorrection(tissue, suture, instrument, contacts, dt);
    }
}

double ConstraintProjector::projectCriticalConstraints(
    Tissue& tissue,
    Suture& suture,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    double totalError = 0.0;
    
    // 1. 处理接触约束
    projectContactConstraints(tissue, suture, instrument, contacts, dt);
    
    // 2. 处理夹持约束
    projectGraspConstraints(suture, instrument, dt);
    
    // 3. 求解缝合线的关键约束 (距离约束、自碰撞)
    std::vector<int> criticalConstraints;
    for (int i = 0; i < suture.getConstraintCount(); ++i) {
        auto constraint = suture.getConstraint(i);
        const char* type = constraint->getType();
        
        // 距离约束和碰撞约束是关键约束
        if (std::string(type) == "Distance" || std::string(type) == "Collision") {
            criticalConstraints.push_back(i);
        }
    }
    
    if (!criticalConstraints.empty()) {
        totalError = solveConstraintGroup(suture, criticalConstraints, dt);
    }
    
    lastError_ = totalError;
    return totalError;
}

double ConstraintProjector::projectNonCriticalConstraints(
    Suture& suture,
    double dt
) {
    std::vector<int> nonCriticalConstraints;
    
    for (int i = 0; i < suture.getConstraintCount(); ++i) {
        auto constraint = suture.getConstraint(i);
        const char* type = constraint->getType();
        
        // 弯曲约束和锚定约束是非关键约束
        if (std::string(type) == "Bending" || std::string(type) == "Anchor") {
            nonCriticalConstraints.push_back(i);
        }
    }
    
    if (nonCriticalConstraints.empty()) {
        return 0.0;
    }
    
    return solveConstraintGroup(suture, nonCriticalConstraints, dt);
}

void ConstraintProjector::localCorrection(
    Tissue& tissue,
    Suture& suture,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    // 局部修正：只处理与接触相关的约束
    
    // 1. 修正接触点附近的约束
    for (const auto& contact : contacts) {
        if (!contact.isValid()) continue;
        
        // 根据接触类型进行修正
        switch (contact.type) {
            case ContactType::SutureTissue:
                // 修正缝合线质点位置
                if (contact.object1Index >= 0 && contact.object1Index < suture.getParticleCount()) {
                    auto& particle = suture.getParticle(contact.object1Index);
                    if (!particle.isFixed) {
                        // 将质点推出穿透区域
                        particle.position += contact.normal * contact.penetration;
                    }
                }
                break;
                
            case ContactType::InstrumentSuture:
                // 修正被夹持的质点
                if (contact.object2Index >= 0 && contact.object2Index < suture.getParticleCount()) {
                    auto& particle = suture.getParticle(contact.object2Index);
                    if (!particle.isFixed) {
                        particle.position += contact.normal * contact.penetration;
                    }
                }
                break;
                
            default:
                break;
        }
    }
    
    // 2. 修正夹持约束
    projectGraspConstraints(suture, instrument, dt);
}

void ConstraintProjector::projectContactConstraints(
    Tissue& tissue,
    Suture& suture,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    // 应用接触力到物体
    applyContactForces(tissue, suture, instrument, contacts);
    
    // 处理穿透修正
    for (const auto& contact : contacts) {
        if (!contact.isValid()) continue;
        
        // 根据接触类型处理
        switch (contact.type) {
            case ContactType::SutureTissue: {
                // 缝合线-组织接触
                int particleIdx = contact.object1Index;
                if (particleIdx >= 0 && particleIdx < suture.getParticleCount()) {
                    auto& particle = suture.getParticle(particleIdx);
                    if (!particle.isFixed) {
                        // 位置修正
                        Vector3d correction = contact.normal * contact.penetration;
                        particle.position += correction;
                        
                        // 速度修正 (反弹)
                        double restitution = 0.1;  // 恢复系数
                        double vn = particle.velocity.dot(contact.normal);
                        if (vn < 0) {
                            particle.velocity -= (1.0 + restitution) * vn * contact.normal;
                        }
                    }
                }
                break;
            }
            
            case ContactType::InstrumentSuture: {
                // 器械-缝合线接触
                int particleIdx = contact.object2Index;
                if (particleIdx >= 0 && particleIdx < suture.getParticleCount()) {
                    auto& particle = suture.getParticle(particleIdx);
                    if (!particle.isFixed) {
                        Vector3d correction = contact.normal * contact.penetration;
                        particle.position += correction;
                    }
                }
                break;
            }
            
            case ContactType::InstrumentTissue: {
                // 器械-组织接触
                // 这里可以添加对组织的修正
                break;
            }
            
            default:
                break;
        }
    }
}

void ConstraintProjector::projectGraspConstraints(
    Suture& suture,
    Instrument& instrument,
    double dt
) {
    // 更新夹持约束
    updateGraspConstraints(suture, instrument);
    
    // 对于每个夹持点
    for (int i = 0; i < instrument.getGraspPointCount(); ++i) {
        const auto& graspPoint = instrument.getGraspPoint(i);
        
        if (graspPoint.state != GraspState::Grasping) {
            continue;
        }
        
        int graspedParticle = graspPoint.graspedSutureParticle;
        if (graspedParticle < 0 || graspedParticle >= suture.getParticleCount()) {
            continue;
        }
        
        auto& particle = suture.getParticle(graspedParticle);
        
        // 将质点移动到夹持点位置
        particle.position = graspPoint.worldPosition;
        
        // 同步速度
        // 计算夹持点的速度
        Vector3d graspVelocity = instrument.getState().linearVelocity +
            instrument.getState().angularVelocity.cross(
                graspPoint.worldPosition - instrument.getState().position);
        
        particle.velocity = graspVelocity;
    }
}

void ConstraintProjector::classifyConstraints(
    const Suture& suture,
    std::vector<int>& critical,
    std::vector<int>& high,
    std::vector<int>& normal,
    std::vector<int>& low
) {
    critical.clear();
    high.clear();
    normal.clear();
    low.clear();
    
    for (int i = 0; i < suture.getConstraintCount(); ++i) {
        auto constraint = suture.getConstraint(i);
        const char* type = constraint->getType();
        
        if (std::string(type) == "Distance" || std::string(type) == "Collision") {
            critical.push_back(i);
        } else if (std::string(type) == "Anchor") {
            high.push_back(i);
        } else if (std::string(type) == "Bending") {
            normal.push_back(i);
        } else {
            low.push_back(i);
        }
    }
}

double ConstraintProjector::solveConstraintGroup(
    Suture& suture,
    const std::vector<int>& constraintIndices,
    double dt
) {
    double totalError = 0.0;
    
    for (int idx : constraintIndices) {
        auto constraint = suture.getConstraint(idx);
        double error = constraint->solve(suture.getParticles(), dt);
        totalError += error * error;
    }
    
    return std::sqrt(totalError);
}

void ConstraintProjector::applyContactForces(
    Tissue& tissue,
    Suture& suture,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts
) {
    for (const auto& contact : contacts) {
        if (!contact.isValid()) continue;
        
        Vector3d force = contact.totalForce;
        
        switch (contact.type) {
            case ContactType::SutureTissue: {
                // 应用到缝合线质点
                int particleIdx = contact.object1Index;
                if (particleIdx >= 0 && particleIdx < suture.getParticleCount()) {
                    suture.applyForce(particleIdx, force);
                }
                
                // 应用到组织节点
                int tetIdx = contact.object2Index;
                if (tetIdx >= 0 && tetIdx < tissue.getTetrahedronCount()) {
                    // 使用重心坐标将力分配到节点
                    const auto& elem = tissue.getTetrahedron(tetIdx);
                    for (int i = 0; i < 4; ++i) {
                        int nodeIdx = elem.nodeIndices[i];
                        double weight = contact.barycentricCoords[i];
                        tissue.getNode(nodeIdx).force -= force * weight;
                    }
                }
                break;
            }
            
            case ContactType::InstrumentSuture: {
                // 应用到器械
                instrument.applyExternalForce(force, contact.contactPoint);
                
                // 应用到缝合线
                int particleIdx = contact.object2Index;
                if (particleIdx >= 0 && particleIdx < suture.getParticleCount()) {
                    suture.applyForce(particleIdx, -force);
                }
                break;
            }
            
            case ContactType::InstrumentTissue: {
                // 应用到器械
                instrument.applyExternalForce(force, contact.contactPoint);
                break;
            }
            
            default:
                break;
        }
    }
}

void ConstraintProjector::updateGraspConstraints(
    Suture& suture,
    Instrument& instrument
) {
    // 对于每个夹持点，检查是否需要更新夹持的质点
    for (int i = 0; i < instrument.getGraspPointCount(); ++i) {
        auto& graspPoint = instrument.getGraspPoint(i);
        
        if (graspPoint.state == GraspState::Closing) {
            // 查找最近的质点
            double minDist = std::numeric_limits<double>::max();
            int closestParticle = -1;
            
            for (int j = 0; j < suture.getParticleCount(); ++j) {
                const auto& particle = suture.getParticle(j);
                double dist = (particle.position - graspPoint.worldPosition).norm();
                
                // 检查是否在夹持范围内
                if (dist < particle.radius * 2.0 && dist < minDist) {
                    minDist = dist;
                    closestParticle = j;
                }
            }
            
            if (closestParticle >= 0) {
                graspPoint.graspedSutureParticle = closestParticle;
                graspPoint.state = GraspState::Grasping;
            }
        } else if (graspPoint.state == GraspState::Opening) {
            graspPoint.graspedSutureParticle = -1;
            graspPoint.state = GraspState::Open;
        }
    }
}

// CouplingSolver 实现

CouplingSolver::CouplingSolver() = default;
CouplingSolver::~CouplingSolver() = default;

void CouplingSolver::solve(
    Tissue& tissue,
    Suture& suture,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    // 迭代耦合求解
    for (int iter = 0; iter < iterations_; ++iter) {
        // 1. 软组织-缝合线耦合
        solveTissueSutureCoupling(tissue, suture, contacts, dt);
        
        // 2. 缝合线-器械耦合
        solveSutureInstrumentCoupling(suture, instrument, dt);
        
        // 3. 软组织-器械耦合
        solveTissueInstrumentCoupling(tissue, instrument, contacts, dt);
    }
}

void CouplingSolver::solveTissueSutureCoupling(
    Tissue& tissue,
    Suture& suture,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    // 处理缝合线-组织接触
    for (const auto& contact : contacts) {
        if (contact.type != ContactType::SutureTissue) continue;
        if (!contact.isValid()) continue;
        
        int particleIdx = contact.object1Index;
        int tetIdx = contact.object2Index;
        
        if (particleIdx < 0 || particleIdx >= suture.getParticleCount()) continue;
        if (tetIdx < 0 || tetIdx >= tissue.getTetrahedronCount()) continue;
        
        auto& particle = suture.getParticle(particleIdx);
        const auto& elem = tissue.getTetrahedron(tetIdx);
        
        // 计算质点在组织中的嵌入位置
        Vector3d embeddedPos = Vector3d::Zero();
        for (int i = 0; i < 4; ++i) {
            embeddedPos += contact.barycentricCoords[i] * 
                          tissue.getNode(elem.nodeIndices[i]).position;
        }
        
        // 耦合修正
        Vector3d correction = (embeddedPos - particle.position) * 0.5;
        
        if (!particle.isFixed) {
            particle.position += correction;
        }
        
        // 修正组织节点
        for (int i = 0; i < 4; ++i) {
            int nodeIdx = elem.nodeIndices[i];
            auto& node = tissue.getNode(nodeIdx);
            if (!node.isFixed) {
                node.position -= correction * contact.barycentricCoords[i];
            }
        }
    }
}

void CouplingSolver::solveSutureInstrumentCoupling(
    Suture& suture,
    Instrument& instrument,
    double dt
) {
    // 处理夹持耦合
    for (int i = 0; i < instrument.getGraspPointCount(); ++i) {
        const auto& graspPoint = instrument.getGraspPoint(i);
        
        if (graspPoint.state != GraspState::Grasping) continue;
        
        int graspedParticle = graspPoint.graspedSutureParticle;
        if (graspedParticle < 0 || graspedParticle >= suture.getParticleCount()) {
            continue;
        }
        
        auto& particle = suture.getParticle(graspedParticle);
        
        // 强耦合：质点位置完全跟随夹持点
        particle.position = graspPoint.worldPosition;
        
        // 速度同步
        Vector3d graspVelocity = instrument.getState().linearVelocity +
            instrument.getState().angularVelocity.cross(
                graspPoint.worldPosition - instrument.getState().position);
        
        particle.velocity = graspVelocity;
    }
}

void CouplingSolver::solveTissueInstrumentCoupling(
    Tissue& tissue,
    Instrument& instrument,
    const std::vector<ContactPair>& contacts,
    double dt
) {
    // 处理器械-组织接触
    for (const auto& contact : contacts) {
        if (contact.type != ContactType::InstrumentTissue) continue;
        if (!contact.isValid()) continue;
        
        // 应用接触力到器械
        instrument.applyExternalForce(contact.totalForce, contact.contactPoint);
        
        // 应用接触力到组织
        int tetIdx = contact.object2Index;
        if (tetIdx < 0 || tetIdx >= tissue.getTetrahedronCount()) continue;
        
        const auto& elem = tissue.getTetrahedron(tetIdx);
        for (int i = 0; i < 4; ++i) {
            int nodeIdx = elem.nodeIndices[i];
            double weight = contact.barycentricCoords[i];
            tissue.getNode(nodeIdx).force += contact.totalForce * weight;
        }
    }
}

} // namespace vss
