#include "xpbd_solver.h"
#include <chrono>
#include <unordered_set>

namespace vss {

// XPBDSolver 实现

XPBDSolver::XPBDSolver() = default;
XPBDSolver::~XPBDSolver() = default;

void XPBDSolver::solve(Suture& suture, double dt) {
    auto start = std::chrono::high_resolution_clock::now();
    
    // 计算子步长
    double subDt = dt / config_.substepCount;
    
    for (int substep = 0; substep < config_.substepCount; ++substep) {
        solveSubstep(suture, subDt);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    solveTime_ = std::chrono::duration<double>(end - start).count();
}

void XPBDSolver::solveSubstep(Suture& suture, double dt) {
    // 1. 预测位置
    predictPositions(suture, dt);
    
    // 2. 求解约束
    solveConstraints(suture, dt);
    
    // 3. 更新速度
    updateVelocities(suture, dt);
    
    // 4. 应用阻尼
    applyDamping(suture);
}

void XPBDSolver::predictPositions(Suture& suture, double dt) {
    for (auto& particle : suture.getParticles()) {
        if (!particle.isFixed) {
            particle.prevPosition = particle.position;
            
            // 应用外力 (重力等)
            // v = v + f_ext * dt * invMass
            particle.velocity += particle.force * particle.inverseMass * dt;
            
            // x = x + v * dt
            particle.position += particle.velocity * dt;
            
            // 清除外力
            particle.force = Vector3d::Zero();
        }
    }
}

void XPBDSolver::solveConstraints(Suture& suture, double dt) {
    constraintError_ = 0.0;
    
    // 对约束进行分组以支持并行求解
    groupConstraints(suture);
    
    // 迭代求解
    for (int iter = 0; iter < config_.iterations; ++iter) {
        double iterError = 0.0;
        
        // 按组求解约束
        for (const auto& group : constraintGroups_) {
            iterError += solveConstraintGroup(suture, group, dt);
        }
        
        constraintError_ = iterError;
        lastIterations_ = iter + 1;
        
        // 检查收敛
        if (iterError < 1e-6) {
            break;
        }
    }
}

void XPBDSolver::groupConstraints(const Suture& suture) {
    constraintGroups_.clear();
    
    int numConstraints = suture.getConstraintCount();
    if (numConstraints == 0) {
        return;
    }
    
    // 简单的贪心分组算法
    std::vector<bool> assigned(numConstraints, false);
    
    while (true) {
        std::vector<int> group;
        std::unordered_set<int> particlesInGroup;
        
        for (int i = 0; i < numConstraints; ++i) {
            if (assigned[i]) continue;
            
            // 检查是否与组内约束共享粒子
            auto constraint = suture.getConstraint(i);
            auto particleIndices = constraint->getParticleIndices();
            
            bool sharesParticles = false;
            for (int pIdx : particleIndices) {
                if (particlesInGroup.count(pIdx) > 0) {
                    sharesParticles = true;
                    break;
                }
            }
            
            if (!sharesParticles) {
                group.push_back(i);
                assigned[i] = true;
                for (int pIdx : particleIndices) {
                    particlesInGroup.insert(pIdx);
                }
            }
        }
        
        if (group.empty()) {
            break;
        }
        
        constraintGroups_.push_back(group);
    }
}

bool XPBDSolver::constraintsShareParticles(int c1, int c2, const Suture& suture) {
    auto constraint1 = suture.getConstraint(c1);
    auto constraint2 = suture.getConstraint(c2);
    
    auto particles1 = constraint1->getParticleIndices();
    auto particles2 = constraint2->getParticleIndices();
    
    for (int p1 : particles1) {
        for (int p2 : particles2) {
            if (p1 == p2) {
                return true;
            }
        }
    }
    
    return false;
}

double XPBDSolver::solveConstraintGroup(Suture& suture, 
                                         const std::vector<int>& group, 
                                         double dt) {
    double totalError = 0.0;
    
    for (int constraintIdx : group) {
        auto constraint = suture.getConstraint(constraintIdx);
        double error = constraint->solve(suture.getParticles(), dt);
        totalError += error * error;
    }
    
    return std::sqrt(totalError);
}

void XPBDSolver::updateVelocities(Suture& suture, double dt) {
    if (dt < 1e-10) {
        return;
    }
    
    for (auto& particle : suture.getParticles()) {
        if (!particle.isFixed) {
            particle.velocity = (particle.position - particle.prevPosition) / dt;
        }
    }
}

void XPBDSolver::applyDamping(Suture& suture) {
    for (auto& particle : suture.getParticles()) {
        if (!particle.isFixed) {
            particle.velocity *= config_.damping;
        }
    }
}

// PBDSolver 实现

PBDSolver::PBDSolver() = default;
PBDSolver::~PBDSolver() = default;

void PBDSolver::solve(Suture& suture, double dt) {
    // PBD求解 (简化版本)
    
    // 1. 预测位置
    for (auto& particle : suture.getParticles()) {
        if (!particle.isFixed) {
            particle.prevPosition = particle.position;
            particle.position += particle.velocity * dt;
        }
    }
    
    // 2. 求解约束
    for (int iter = 0; iter < iterations_; ++iter) {
        for (int i = 0; i < suture.getConstraintCount(); ++i) {
            auto constraint = suture.getConstraint(i);
            
            // PBD使用刚度直接作为混合因子
            double oldStiffness = constraint->stiffness;
            constraint->stiffness = stiffness_;
            constraint->compliance = 0.0;
            
            constraint->solve(suture.getParticles(), dt);
            
            constraint->stiffness = oldStiffness;
        }
    }
    
    // 3. 更新速度
    for (auto& particle : suture.getParticles()) {
        if (!particle.isFixed) {
            particle.velocity = (particle.position - particle.prevPosition) / dt;
        }
    }
}

} // namespace vss
