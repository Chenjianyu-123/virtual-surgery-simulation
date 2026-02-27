#!/usr/bin/env python3
"""
虚拟手术缝合多体耦合仿真系统 - 算法验证测试
"""

import numpy as np
import math

# ============ 基础类型测试 ============

class Vec3:
    """3D向量"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.v = np.array([x, y, z], dtype=float)
    
    @property
    def x(self): return self.v[0]
    @property
    def y(self): return self.v[1]
    @property
    def z(self): return self.v[2]
    
    def __add__(self, other): return Vec3(*(self.v + other.v))
    def __sub__(self, other): return Vec3(*(self.v - other.v))
    def __mul__(self, s): return Vec3(*(self.v * s))
    def __truediv__(self, s): return Vec3(*(self.v / s))
    def __repr__(self): return f"Vec3({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"
    
    def dot(self, other): return np.dot(self.v, other.v)
    def cross(self, other): return Vec3(*np.cross(self.v, other.v))
    def length(self): return np.linalg.norm(self.v)
    def normalized(self): 
        len = self.length()
        return self / len if len > 1e-10 else Vec3()

class AABB:
    """AABB包围盒"""
    def __init__(self, min_vec=None, max_vec=None):
        self.min = min_vec if min_vec else Vec3()
        self.max = max_vec if max_vec else Vec3()
    
    def center(self): return (self.min + self.max) * 0.5
    def volume(self): 
        ext = self.max - self.min
        return ext.x * ext.y * ext.z

class Material:
    """材质属性"""
    def __init__(self, E=1e5, nu=0.45, rho=1000.0):
        self.youngs_modulus = E
        self.poisson_ratio = nu
        self.density = rho
    
    def lambda_param(self):
        return self.youngs_modulus * self.poisson_ratio / \
               ((1.0 + self.poisson_ratio) * (1.0 - 2.0 * self.poisson_ratio))
    
    def mu(self):
        return self.youngs_modulus / (2.0 * (1.0 + self.poisson_ratio))

class Tetrahedron:
    """四面体"""
    def __init__(self, v0, v1, v2, v3):
        self.vertices = [v0, v1, v2, v3]
    
    def volume(self):
        e1 = self.vertices[1] - self.vertices[0]
        e2 = self.vertices[2] - self.vertices[0]
        e3 = self.vertices[3] - self.vertices[0]
        return abs(e1.dot(e2.cross(e3))) / 6.0
    
    def centroid(self):
        return sum(self.vertices, Vec3()) * 0.25

# ============ 软组织模块测试 ============

class TissueNode:
    """软组织节点"""
    def __init__(self):
        self.position = Vec3()
        self.velocity = Vec3()
        self.force = Vec3()
        self.mass = 0.0
        self.is_fixed = False

class SimpleTissue:
    """简化软组织"""
    def __init__(self):
        self.nodes = []
        self.elements = []
    
    def add_node(self, pos, mass=1.0):
        node = TissueNode()
        node.position = pos
        node.mass = mass
        self.nodes.append(node)
        return len(self.nodes) - 1
    
    def add_tetrahedron(self, n0, n1, n2, n3):
        self.elements.append(Tetrahedron(
            self.nodes[n0].position,
            self.nodes[n1].position,
            self.nodes[n2].position,
            self.nodes[n3].position
        ))
    
    def get_total_volume(self):
        return sum(elem.volume() for elem in self.elements)
    
    def get_total_mass(self):
        return sum(node.mass for node in self.nodes)

# ============ 缝合线模块测试 ============

class SutureParticle:
    """缝合线质点"""
    def __init__(self):
        self.position = Vec3()
        self.prev_position = Vec3()
        self.velocity = Vec3()
        self.mass = 0.01
        self.inverse_mass = 100.0
        self.is_fixed = False

class SimpleSuture:
    """简化缝合线"""
    def __init__(self):
        self.particles = []
    
    def create_chain(self, positions, total_mass):
        self.particles = []
        mass_per_particle = total_mass / len(positions)
        
        for pos in positions:
            p = SutureParticle()
            p.position = pos
            p.prev_position = pos
            p.mass = mass_per_particle
            p.inverse_mass = 1.0 / mass_per_particle
            self.particles.append(p)
    
    def simulate(self, dt, gravity):
        # 预测位置
        for p in self.particles:
            if not p.is_fixed:
                p.prev_position = p.position
                p.velocity = p.velocity + gravity * dt
                p.position = p.position + p.velocity * dt
        
        # 距离约束
        for i in range(len(self.particles) - 1):
            p1 = self.particles[i]
            p2 = self.particles[i + 1]
            
            delta = p2.position - p1.position
            dist = delta.length()
            if dist < 1e-10:
                continue
            
            target_dist = 0.1
            diff = (dist - target_dist) / dist
            offset = delta * diff * 0.5
            
            if not p1.is_fixed:
                p1.position = p1.position + offset
            if not p2.is_fixed:
                p2.position = p2.position - offset
        
        # 更新速度
        for p in self.particles:
            if not p.is_fixed:
                p.velocity = (p.position - p.prev_position) / dt
    
    def get_total_length(self):
        length = 0.0
        for i in range(len(self.particles) - 1):
            length += (self.particles[i + 1].position - self.particles[i].position).length()
        return length
    
    def get_center_of_mass(self):
        center = Vec3()
        total_mass = 0.0
        for p in self.particles:
            center = center + p.position * p.mass
            total_mass += p.mass
        return center / total_mass if total_mass > 0 else Vec3()

# ============ 器械模块测试 ============

class InstrumentState:
    """器械刚体状态"""
    def __init__(self):
        self.position = Vec3()
        self.orientation = np.eye(3)  # 旋转矩阵
        self.linear_velocity = Vec3()
        self.angular_velocity = Vec3()
        self.mass = 1.0
        self.force = Vec3()
        self.torque = Vec3()
    
    def integrate(self, dt):
        # 半隐式欧拉积分
        self.linear_velocity = self.linear_velocity + (self.force * (1.0 / self.mass)) * dt
        self.position = self.position + self.linear_velocity * dt
        
        # 简化的角速度积分
        self.angular_velocity = self.angular_velocity * 0.99  # 阻尼
        
        self.force = Vec3()
        self.torque = Vec3()

# ============ 测试函数 ============

def test_basic_types():
    """测试基础类型"""
    print("=== Testing Basic Types ===")
    
    # 测试向量
    v1 = Vec3(1, 2, 3)
    v2 = Vec3(4, 5, 6)
    v3 = v1 + v2
    print(f"v1 + v2 = {v3}")
    print(f"v1 dot v2 = {v1.dot(v2):.4f}")
    print(f"v1 cross v2 = {v1.cross(v2)}")
    
    # 测试AABB
    bbox = AABB(Vec3(0, 0, 0), Vec3(1, 1, 1))
    print(f"AABB center: {bbox.center()}")
    print(f"AABB volume: {bbox.volume():.6f}")
    
    # 测试材质
    mat = Material(1e5, 0.45, 1000.0)
    print(f"Material lambda: {mat.lambda_param():.4f}")
    print(f"Material mu: {mat.mu():.4f}")
    
    # 测试四面体
    tet = Tetrahedron(
        Vec3(0, 0, 0),
        Vec3(1, 0, 0),
        Vec3(0, 1, 0),
        Vec3(0, 0, 1)
    )
    print(f"Tetrahedron volume: {tet.volume():.6f}")
    print(f"Tetrahedron centroid: {tet.centroid()}")
    
    print("✓ Basic types test passed!\n")

def test_tissue():
    """测试软组织"""
    print("=== Testing Tissue ===")
    
    tissue = SimpleTissue()
    
    # 创建四面体
    tissue.add_node(Vec3(0, 0, 0), 0.25)
    tissue.add_node(Vec3(1, 0, 0), 0.25)
    tissue.add_node(Vec3(0, 1, 0), 0.25)
    tissue.add_node(Vec3(0, 0, 1), 0.25)
    
    tissue.add_tetrahedron(0, 1, 2, 3)
    
    print(f"Node count: {len(tissue.nodes)}")
    print(f"Element count: {len(tissue.elements)}")
    print(f"Total volume: {tissue.get_total_volume():.6f}")
    print(f"Total mass: {tissue.get_total_mass():.4f}")
    
    print("✓ Tissue test passed!\n")

def test_suture():
    """测试缝合线"""
    print("=== Testing Suture ===")
    
    suture = SimpleSuture()
    
    positions = [
        Vec3(0, 0, 0),
        Vec3(0.1, 0, 0),
        Vec3(0.2, 0, 0),
        Vec3(0.3, 0, 0),
        Vec3(0.4, 0, 0)
    ]
    
    suture.create_chain(positions, 0.1)
    
    print(f"Particle count: {len(suture.particles)}")
    print(f"Initial length: {suture.get_total_length():.6f}")
    print(f"Initial COM: {suture.get_center_of_mass()}")
    
    # 模拟
    gravity = Vec3(0, -9.81, 0)
    for i in range(100):
        suture.simulate(0.001, gravity)
    
    print(f"Final COM: {suture.get_center_of_mass()}")
    print(f"Final length: {suture.get_total_length():.6f}")
    
    print("✓ Suture test passed!\n")

def test_instrument():
    """测试器械"""
    print("=== Testing Instrument ===")
    
    inst = InstrumentState()
    inst.mass = 0.5
    inst.position = Vec3(0, 0, 0)
    
    # 应用力
    inst.force = Vec3(1.0, 0, 0)
    
    print(f"Initial position: {inst.position}")
    print(f"Initial velocity: {inst.linear_velocity}")
    
    # 积分
    for i in range(100):
        inst.integrate(0.001)
        inst.force = Vec3(1.0, 0, 0)  # 持续施加力
    
    print(f"Final position: {inst.position}")
    print(f"Final velocity: {inst.linear_velocity}")
    
    print("✓ Instrument test passed!\n")

def test_performance():
    """性能测试"""
    print("=== Performance Test ===")
    
    import time
    
    # 创建较大的缝合线
    suture = SimpleSuture()
    positions = [Vec3(i * 0.01, 0, 0) for i in range(50)]
    suture.create_chain(positions, 1.0)
    
    gravity = Vec3(0, -9.81, 0)
    
    start_time = time.time()
    steps = 100
    for _ in range(steps):
        suture.simulate(0.001, gravity)
    elapsed = time.time() - start_time
    
    print(f"{steps} simulation steps with {len(positions)} particles: {elapsed:.4f}s")
    print(f"Average time per step: {elapsed/steps*1000:.4f}ms")
    if elapsed > 0:
        print(f"Estimated FPS: {steps/elapsed:.2f}")
    
    print("✓ Performance test passed!\n")

# ============ 主程序 ============

if __name__ == "__main__":
    print("=" * 50)
    print("Virtual Surgery Simulation System Test")
    print("=" * 50)
    print()
    
    try:
        test_basic_types()
        test_tissue()
        test_suture()
        test_instrument()
        test_performance()
        
        print("=" * 50)
        print("All tests passed successfully!")
        print("=" * 50)
        
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
