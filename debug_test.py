#!/usr/bin/env python3
"""调试测试 - 找出NaN问题"""

import math

class Vec3:
    __slots__ = ['x', 'y', 'z']
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __mul__(self, s):
        return Vec3(self.x * s, self.y * s, self.z * s)
    def __truediv__(self, s):
        return Vec3(self.x / s, self.y / s, self.z / s)
    def length_sq(self):
        return self.x * self.x + self.y * self.y + self.z * self.z
    def length(self):
        return math.sqrt(self.length_sq())
    def copy(self):
        return Vec3(self.x, self.y, self.z)
    def __repr__(self):
        return f"Vec3({self.x}, {self.y}, {self.z})"

class Particle:
    def __init__(self):
        self.position = Vec3()
        self.prev_position = Vec3()
        self.velocity = Vec3()
        self.mass = 0.01
        self.inverse_mass = 100.0
        self.is_fixed = False

# 模拟缝合线
particles = []
positions = [Vec3(0, 1.0 - i*0.1, 0) for i in range(5)]
for pos in positions:
    p = Particle()
    p.position = pos.copy()
    p.prev_position = pos.copy()
    particles.append(p)

# 固定第一个
particles[0].is_fixed = True

print("初始状态:")
for i, p in enumerate(particles):
    print(f"  p{i}: {p.position}, fixed={p.is_fixed}")

# 计算初始长度
initial_length = 0.0
for i in range(len(particles) - 1):
    d = particles[i+1].position - particles[i].position
    initial_length += d.length()
print(f"\n初始长度: {initial_length}")

# 模拟
dt = 0.001
gravity = Vec3(0, -9.81, 0)
damping = 0.995

for step in range(10):
    # 预测
    for p in particles:
        if not p.is_fixed:
            p.prev_position = p.position.copy()
            total_force = gravity * p.mass
            p.velocity = p.velocity + total_force * p.inverse_mass * dt
            p.velocity = p.velocity * damping
            p.position = p.position + p.velocity * dt
    
    # 约束求解
    for iteration in range(5):
        for i in range(len(particles) - 1):
            p1 = particles[i]
            p2 = particles[i+1]
            
            if p1.is_fixed and p2.is_fixed:
                continue
            
            delta = p2.position - p1.position
            dist_sq = delta.length_sq()
            
            if dist_sq < 1e-10:
                print(f"Step {step}, iter {iteration}: dist_sq too small!")
                continue
            
            dist = math.sqrt(dist_sq)
            rest_length = 0.1  # 0.1m
            
            C = dist - rest_length
            w1 = p1.inverse_mass
            w2 = p2.inverse_mass
            w_sum = w1 + w2
            
            if w_sum < 1e-10:
                continue
            
            lambda_val = -C / w_sum
            
            if dist < 1e-10:
                print(f"Step {step}, iter {iteration}: dist too small!")
                continue
            
            # 修正
            scale = lambda_val / dist
            dx = delta.x * scale
            dy = delta.y * scale
            dz = delta.z * scale
            
            if not p1.is_fixed:
                p1.position.x += dx * w1
                p1.position.y += dy * w1
                p1.position.z += dz * w1
            if not p2.is_fixed:
                p2.position.x -= dx * w2
                p2.position.y -= dy * w2
                p2.position.z -= dz * w2
    
    # 更新速度
    for p in particles:
        if not p.is_fixed:
            p.velocity = (p.position - p.prev_position) / dt
    
    # 检查NaN
    for i, p in enumerate(particles):
        if math.isnan(p.position.x) or math.isnan(p.position.y) or math.isnan(p.position.z):
            print(f"Step {step}: p{i} position is NaN!")

# 计算最终长度
final_length = 0.0
for i in range(len(particles) - 1):
    d = particles[i+1].position - particles[i].position
    final_length += d.length()

print(f"\n最终长度: {final_length}")
print(f"长度误差: {abs(final_length - initial_length)}")
print(f"长度误差是NaN: {math.isnan(abs(final_length - initial_length))}")

print("\n最终状态:")
for i, p in enumerate(particles):
    print(f"  p{i}: {p.position}")
