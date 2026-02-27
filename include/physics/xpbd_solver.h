#pragma once

#include "types.h"
#include "suture.h"

namespace vss {

// XPBD求解器配置
struct XPBDSolverConfig {
    int iterations = 10;            // 约束求解迭代次数
    int substepCount = 1;           // 子步数
    double damping = 0.99;          // 速度阻尼
    double compliance = 0.0;        // 全局柔度
    bool useGPU = false;            // 是否使用GPU
};

// XPBD求解器
class XPBDSolver {
public:
    XPBDSolver();
    ~XPBDSolver();
    
    // 设置配置
    void setConfig(const XPBDSolverConfig& config) { config_ = config; }
    const XPBDSolverConfig& getConfig() const { return config_; }
    
    // 求解一个时间步
    void solve(Suture& suture, double dt);
    
    // 子步求解
    void solveSubstep(Suture& suture, double dt);
    
    // 预测位置
    void predictPositions(Suture& suture, double dt);
    
    // 求解约束
    void solveConstraints(Suture& suture, double dt);
    
    // 更新速度
    void updateVelocities(Suture& suture, double dt);
    
    // 应用阻尼
    void applyDamping(Suture& suture);
    
    // 获取求解统计
    int getLastIterations() const { return lastIterations_; }
    double getConstraintError() const { return constraintError_; }
    double getSolveTime() const { return solveTime_; }
    
private:
    XPBDSolverConfig config_;
    
    // 统计信息
    int lastIterations_ = 0;
    double constraintError_ = 0.0;
    double solveTime_ = 0.0;
    
    // 约束分组 (用于并行求解)
    std::vector<std::vector<int>> constraintGroups_;
    
    // 内部辅助函数
    void groupConstraints(const Suture& suture);
    bool constraintsShareParticles(int c1, int c2, const Suture& suture);
    double solveConstraintGroup(Suture& suture, const std::vector<int>& group, double dt);
};

// 位置基动力学求解器 (PBD的扩展)
class PBDSolver {
public:
    PBDSolver();
    ~PBDSolver();
    
    // 求解
    void solve(Suture& suture, double dt);
    
    // 设置迭代次数
    void setIterations(int iterations) { iterations_ = iterations; }
    void setStiffness(double stiffness) { stiffness_ = stiffness; }
    
private:
    int iterations_ = 10;
    double stiffness_ = 1.0;
};

} // namespace vss
