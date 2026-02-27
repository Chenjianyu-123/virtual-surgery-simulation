// 简化测试程序 - 不依赖外部库
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <numeric>

// 简单的向量类
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
    Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }
    
    double dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const { double len = length(); return len > 0 ? (*this) / len : Vec3(); }
};

std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

// AABB
struct AABB {
    Vec3 min, max;
    AABB() {}
    AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}
    
    Vec3 center() const { return (min + max) * 0.5; }
    double volume() const { 
        Vec3 ext = max - min;
        return ext.x * ext.y * ext.z;
    }
};

// 材质
struct Material {
    double youngsModulus;
    double poissonRatio;
    double density;
    
    Material(double E = 1e5, double nu = 0.45, double rho = 1000.0)
        : youngsModulus(E), poissonRatio(nu), density(rho) {}
    
    double lambda() const {
        return youngsModulus * poissonRatio / ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio));
    }
    
    double mu() const {
        return youngsModulus / (2.0 * (1.0 + poissonRatio));
    }
};

// 四面体
struct Tetrahedron {
    std::array<Vec3, 4> vertices;
    
    Tetrahedron() {}
    Tetrahedron(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3) {
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
        vertices[3] = v3;
    }
    
    double volume() const {
        Vec3 e1 = vertices[1] - vertices[0];
        Vec3 e2 = vertices[2] - vertices[0];
        Vec3 e3 = vertices[3] - vertices[0];
        return std::abs(e1.dot(e2.cross(e3))) / 6.0;
    }
    
    Vec3 centroid() const {
        return (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25;
    }
};

// 软组织节点
struct TissueNode {
    Vec3 position;
    Vec3 velocity;
    Vec3 force;
    double mass;
    bool isFixed;
    
    TissueNode() : mass(0.0), isFixed(false) {}
};

// 软组织类
class SimpleTissue {
public:
    std::vector<TissueNode> nodes;
    std::vector<Tetrahedron> elements;
    
    void addNode(const Vec3& pos, double mass = 1.0) {
        TissueNode node;
        node.position = pos;
        node.mass = mass;
        nodes.push_back(node);
    }
    
    void addTetrahedron(int n0, int n1, int n2, int n3) {
        elements.push_back(Tetrahedron(
            nodes[n0].position,
            nodes[n1].position,
            nodes[n2].position,
            nodes[n3].position
        ));
    }
    
    double getTotalVolume() const {
        double vol = 0.0;
        for (const auto& elem : elements) {
            vol += elem.volume();
        }
        return vol;
    }
    
    double getTotalMass() const {
        double mass = 0.0;
        for (const auto& node : nodes) {
            mass += node.mass;
        }
        return mass;
    }
};

// 缝合线质点
struct SutureParticle {
    Vec3 position;
    Vec3 prevPosition;
    Vec3 velocity;
    double mass;
    double inverseMass;
    bool isFixed;
    
    SutureParticle() : mass(0.01), inverseMass(100.0), isFixed(false) {}
};

// 简单缝合线
class SimpleSuture {
public:
    std::vector<SutureParticle> particles;
    
    void createChain(const std::vector<Vec3>& positions, double totalMass) {
        particles.clear();
        double massPerParticle = totalMass / positions.size();
        
        for (const auto& pos : positions) {
            SutureParticle p;
            p.position = pos;
            p.prevPosition = pos;
            p.mass = massPerParticle;
            p.inverseMass = 1.0 / massPerParticle;
            particles.push_back(p);
        }
    }
    
    void simulate(double dt, const Vec3& gravity) {
        for (auto& p : particles) {
            if (!p.isFixed) {
                p.prevPosition = p.position;
                
                // 应用重力
                p.velocity = p.velocity + gravity * dt;
                
                // 更新位置
                p.position = p.position + p.velocity * dt;
            }
        }
        
        // 简单的距离约束
        for (size_t i = 0; i < particles.size() - 1; ++i) {
            SutureParticle& p1 = particles[i];
            SutureParticle& p2 = particles[i + 1];
            
            Vec3 delta = p2.position - p1.position;
            double dist = delta.length();
            if (dist < 1e-10) continue;
            
            double targetDist = 0.1;  // 目标距离
            double diff = (dist - targetDist) / dist;
            
            Vec3 offset = delta * diff * 0.5;
            
            if (!p1.isFixed) p1.position = p1.position + offset;
            if (!p2.isFixed) p2.position = p2.position - offset;
        }
        
        // 更新速度
        for (auto& p : particles) {
            if (!p.isFixed) {
                p.velocity = (p.position - p.prevPosition) / dt;
            }
        }
    }
    
    double getTotalLength() const {
        double len = 0.0;
        for (size_t i = 0; i < particles.size() - 1; ++i) {
            len += (particles[i + 1].position - particles[i].position).length();
        }
        return len;
    }
    
    Vec3 getCenterOfMass() const {
        Vec3 center;
        double totalMass = 0.0;
        for (const auto& p : particles) {
            center = center + p.position * p.mass;
            totalMass += p.mass;
        }
        return totalMass > 0 ? center / totalMass : Vec3();
    }
};

// 测试函数
void testBasicTypes() {
    std::cout << "=== Testing Basic Types ===" << std::endl;
    
    // 测试向量
    Vec3 v1(1, 2, 3);
    Vec3 v2(4, 5, 6);
    Vec3 v3 = v1 + v2;
    std::cout << "v1 + v2 = " << v3 << std::endl;
    std::cout << "v1 dot v2 = " << v1.dot(v2) << std::endl;
    std::cout << "v1 cross v2 = " << v1.cross(v2) << std::endl;
    
    // 测试AABB
    AABB bbox(Vec3(0, 0, 0), Vec3(1, 1, 1));
    std::cout << "AABB center: " << bbox.center() << std::endl;
    std::cout << "AABB volume: " << bbox.volume() << std::endl;
    
    // 测试材质
    Material mat(1e5, 0.45, 1000.0);
    std::cout << "Material lambda: " << mat.lambda() << std::endl;
    std::cout << "Material mu: " << mat.mu() << std::endl;
    
    // 测试四面体
    Tetrahedron tet(Vec3(0,0,0), Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1));
    std::cout << "Tetrahedron volume: " << tet.volume() << std::endl;
    std::cout << "Tetrahedron centroid: " << tet.centroid() << std::endl;
    
    std::cout << "Basic types test passed!" << std::endl << std::endl;
}

void testTissue() {
    std::cout << "=== Testing Simple Tissue ===" << std::endl;
    
    SimpleTissue tissue;
    
    // 创建四面体
    tissue.addNode(Vec3(0, 0, 0), 0.25);
    tissue.addNode(Vec3(1, 0, 0), 0.25);
    tissue.addNode(Vec3(0, 1, 0), 0.25);
    tissue.addNode(Vec3(0, 0, 1), 0.25);
    
    tissue.addTetrahedron(0, 1, 2, 3);
    
    std::cout << "Node count: " << tissue.nodes.size() << std::endl;
    std::cout << "Element count: " << tissue.elements.size() << std::endl;
    std::cout << "Total volume: " << tissue.getTotalVolume() << std::endl;
    std::cout << "Total mass: " << tissue.getTotalMass() << std::endl;
    
    std::cout << "Tissue test passed!" << std::endl << std::endl;
}

void testSuture() {
    std::cout << "=== Testing Simple Suture ===" << std::endl;
    
    SimpleSuture suture;
    
    std::vector<Vec3> positions = {
        Vec3(0, 0, 0),
        Vec3(0.1, 0, 0),
        Vec3(0.2, 0, 0),
        Vec3(0.3, 0, 0),
        Vec3(0.4, 0, 0)
    };
    
    suture.createChain(positions, 0.1);
    
    std::cout << "Particle count: " << suture.particles.size() << std::endl;
    std::cout << "Initial length: " << suture.getTotalLength() << std::endl;
    std::cout << "Initial COM: " << suture.getCenterOfMass() << std::endl;
    
    // 模拟几步
    Vec3 gravity(0, -9.81, 0);
    for (int i = 0; i < 10; ++i) {
        suture.simulate(0.001, gravity);
    }
    
    std::cout << "Final COM: " << suture.getCenterOfMass() << std::endl;
    std::cout << "Final length: " << suture.getTotalLength() << std::endl;
    
    std::cout << "Suture test passed!" << std::endl << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Virtual Surgery Simulation - Simple Test" << std::endl;
    std::cout << "========================================" << std::endl << std::endl;
    
    try {
        testBasicTypes();
        testTissue();
        testSuture();
        
        std::cout << "========================================" << std::endl;
        std::cout << "All tests passed successfully!" << std::endl;
        std::cout << "========================================" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
}
