#pragma once

#include "types.h"
#include "tissue.h"
#include "suture.h"
#include "instrument.h"
#include "contact.h"

namespace vss {

// 约束组优先级
enum class ConstraintPriority {
    Critical,       // 关键约束：距离、自碰撞、夹持
    High,           // 高优先级：接触
    Normal,         // 普通优先级：弯曲、扭转
    Low             // 低优先级：锚定（可弹性）
};

// 约束投影器配置
struct ConstraintProjectorConfig {
    int maxIterations = 10;         // 最大迭代次数
    int criticalIterations = 5;     // 关键约束迭代次数
    double tolerance = 1e-6;        // 收敛容差
    int localCorrectionPasses = 3;  // 局部修正次数
    bool enableParallel = false;    // 是否启用并行
};

// 约束投影器
class ConstraintProjector {
public:
    ConstraintProjector();
    ~ConstraintProjector();
    
    // 设置配置
    void setConfig(const ConstraintProjectorConfig& config) { config_ = config; }
    const ConstraintProjectorConfig& getConfig() const { return config_; }
    
    // 投影所有约束
    void projectConstraints(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts,
        double dt
    );
    
    // 投影关键约束
    double projectCriticalConstraints(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts,
        double dt
    );
    
    // 投影非关键约束
    double projectNonCriticalConstraints(
        Suture& suture,
        double dt
    );
    
    // 局部修正
    void localCorrection(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts,
        double dt
    );
    
    // 处理接触约束
    void projectContactConstraints(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts,
        double dt
    );
    
    // 处理夹持约束
    void projectGraspConstraints(
        Suture& suture,
        Instrument& instrument,
        double dt
    );
    
    // 获取统计信息
    int getLastIterations() const { return lastIterations_; }
    double getLastError() const { return lastError_; }
    
private:
    ConstraintProjectorConfig config_;
    
    // 统计信息
    int lastIterations_ = 0;
    double lastError_ = 0.0;
    
    // 内部辅助函数
    void classifyConstraints(
        const Suture& suture,
        std::vector<int>& critical,
        std::vector<int>& high,
        std::vector<int>& normal,
        std::vector<int>& low
    );
    
    double solveConstraintGroup(
        Suture& suture,
        const std::vector<int>& constraintIndices,
        double dt
    );
    
    void applyContactForces(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts
    );
    
    void updateGraspConstraints(
        Suture& suture,
        Instrument& instrument
    );
};

// 耦合求解器 (用于多体耦合)
class CouplingSolver {
public:
    CouplingSolver();
    ~CouplingSolver();
    
    // 求解多体耦合
    void solve(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts,
        double dt
    );
    
    // 设置迭代次数
    void setIterations(int iterations) { iterations_ = iterations; }
    void setTolerance(double tolerance) { tolerance_ = tolerance; }
    
private:
    int iterations_ = 10;
    double tolerance_ = 1e-6;
    
    // 内部辅助函数
    void solveTissueSutureCoupling(
        Tissue& tissue,
        Suture& suture,
        const std::vector<ContactPair>& contacts,
        double dt
    );
    
    void solveSutureInstrumentCoupling(
        Suture& suture,
        Instrument& instrument,
        double dt
    );
    
    void solveTissueInstrumentCoupling(
        Tissue& tissue,
        Instrument& instrument,
        const std::vector<ContactPair>& contacts,
        double dt
    );
};

} // namespace vss
