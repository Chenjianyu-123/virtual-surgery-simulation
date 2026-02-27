#pragma once

#include "types.h"
#include "tissue.h"
#include "suture.h"
#include "instrument.h"
#include <vector>

namespace vss {

// 接触类型
enum class ContactType {
    None,
    NeedleTissue,       // 针-组织
    SutureTissue,       // 线-组织
    InstrumentSuture,   // 器械-线
    InstrumentTissue,   // 器械-组织
    SutureSelf,         // 线自碰撞
    TissueSelf          // 组织自碰撞
};

// 接触对结构
struct ContactPair {
    ContactType type;
    
    // 接触几何信息
    Vector3d contactPoint;      // 接触点 (世界坐标)
    Vector3d normal;            // 法向量 (指向物体1)
    double penetration;         // 穿透深度 (>0表示穿透)
    
    // 物理属性
    double frictionCoeff;       // 摩擦系数
    double stiffnessEnhance;    // 刚度增强系数 (各向异性)
    double restitution;         // 恢复系数
    
    // 接触力
    Vector3d normalForce;       // 法向力
    Vector3d frictionForce;     // 摩擦力
    Vector3d totalForce;        // 总接触力
    
    // 参与接触的物体索引
    int object1Index;
    int object2Index;
    
    // 局部坐标信息
    std::array<double, 4> barycentricCoords;  // 四面体重心坐标 (用于组织)
    int faceIndex;                            // 三角形面索引
    
    // 接触状态
    bool isPersistent;          // 是否持续接触
    int persistentId;           // 持续接触ID
    double contactTime;         // 接触持续时间
    
    ContactPair() : type(ContactType::None), penetration(0.0),
                    frictionCoeff(0.3), stiffnessEnhance(1.0), restitution(0.0),
                    object1Index(-1), object2Index(-1), faceIndex(-1),
                    isPersistent(false), persistentId(-1), contactTime(0.0) {
        contactPoint = normal = normalForce = frictionForce = totalForce = Vector3d::Zero();
        barycentricCoords = {0.0, 0.0, 0.0, 0.0};
    }
    
    // 计算总力
    void computeTotalForce() {
        totalForce = normalForce + frictionForce;
    }
    
    // 检查是否有效接触
    bool isValid() const {
        return type != ContactType::None && penetration > 0.0;
    }
};

// 接触检测管理器
class ContactDetector {
public:
    ContactDetector();
    ~ContactDetector();
    
    // 设置容差
    void setContactOffset(double offset) { contactOffset_ = offset; }
    void setFrictionCoefficient(double friction) { defaultFriction_ = friction; }
    
    // 接触检测入口
    std::vector<ContactPair> detectContacts(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        bool enableSelfCollision = false
    );
    
    // 特定类型的接触检测
    std::vector<ContactPair> detectSutureTissueContacts(
        const Suture& suture,
        const Tissue& tissue
    );
    
    std::vector<ContactPair> detectInstrumentTissueContacts(
        const Instrument& instrument,
        const Tissue& tissue
    );
    
    std::vector<ContactPair> detectInstrumentSutureContacts(
        const Instrument& instrument,
        const Suture& suture
    );
    
    std::vector<ContactPair> detectSutureSelfContacts(
        const Suture& suture
    );
    
    // 计算接触力
    void computeContactForces(
        std::vector<ContactPair>& contacts,
        double dt,
        const SimulationParams& params
    );
    
    // 应用接触力到物体
    void applyContactForces(
        const std::vector<ContactPair>& contacts,
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument
    );
    
    // 清除接触状态
    void clear() { persistentContacts_.clear(); }
    
private:
    double contactOffset_ = 0.001;
    double defaultFriction_ = 0.3;
    
    // 持续接触跟踪
    std::vector<ContactPair> persistentContacts_;
    int nextPersistentId_ = 0;
    
    // 内部辅助函数
    ContactPair createSutureTissueContact(
        int particleIdx,
        int tetIdx,
        const SutureParticle& particle,
        const TetrahedronElement& tet,
        const std::vector<TissueNode>& nodes
    );
    
    void updatePersistentContacts(std::vector<ContactPair>& newContacts);
    
    // 计算摩擦力的辅助函数
    Vector3d computeFrictionForce(
        const Vector3d& relativeVelocity,
        const Vector3d& normal,
        double normalForceMag,
        double frictionCoeff
    );
};

// 接触求解器 (用于迭代求解接触约束)
class ContactSolver {
public:
    ContactSolver();
    ~ContactSolver();
    
    // 求解接触约束
    void solve(
        std::vector<ContactPair>& contacts,
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        int iterations = 10
    );
    
    // 设置求解参数
    void setIterations(int iterations) { iterations_ = iterations; }
    void setTolerance(double tolerance) { tolerance_ = tolerance; }
    
private:
    int iterations_ = 10;
    double tolerance_ = 1e-6;
};

} // namespace vss
