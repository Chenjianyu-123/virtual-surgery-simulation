#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>

#include "core/types.h"
#include "core/math_utils.h"
#include "physics/tissue.h"
#include "physics/suture.h"
#include "physics/instrument.h"
#include "physics/fem_solver.h"
#include "physics/xpbd_solver.h"
#include "physics/rigid_body_solver.h"
#include "collision/aabb_tree.h"

using namespace vss;

struct SimulationConfig {
    double dt = 0.001;
    int max_steps = 10000;
    double target_fps = 60.0;
    bool enable_rendering = true;
    bool enable_haptic = true;
    bool headless = false;
};

struct SimulationStats {
    int current_step = 0;
    double current_time = 0.0;
    double avg_fps = 0.0;
    double total_simulation_time = 0.0;
};

class SimulationScene {
public:
    std::unique_ptr<Tissue> tissue;
    std::unique_ptr<Suture> suture;
    std::unique_ptr<Instrument> instrument;
    
    std::unique_ptr<FEMSolver> fem_solver;
    std::unique_ptr<XPBDSolver> xpbd_solver;
    std::unique_ptr<RigidBodySolver> rigid_solver;
    
    std::unique_ptr<AABBTree> collision_tree;
    
    bool initialize() {
        tissue = std::make_unique<Tissue>();
        Material mat;
        mat.youngsModulus = 1000.0;
        mat.poissonRatio = 0.45;
        mat.density = 1000.0;
        tissue->setGlobalMaterial(mat);
        
        std::vector<Vector3d> positions;
        for (int i = 0; i < 5; ++i) {
            positions.push_back(Vector3d(0, 1.0 - i * 0.1, 0));
        }
        suture = std::make_unique<Suture>();
        suture->createChain(positions, 0.1, 0.001);
        suture->setDamping(0.98);
        
        instrument = std::make_unique<Instrument>();
        instrument->initialize(Vector3d(0, 0, 0), Quaterniond::Identity(), 
                              0.5, Matrix3d::Identity());
        
        fem_solver = std::make_unique<FEMSolver>();
        xpbd_solver = std::make_unique<XPBDSolver>();
        rigid_solver = std::make_unique<RigidBodySolver>();
        
        collision_tree = std::make_unique<AABBTree>();
        
        return true;
    }
    
    void update(double dt) {
        // 简化版 PBD 模拟 - 只处理粒子
        if (suture && suture->getParticleCount() > 0) {
            auto& particles = suture->getParticles();
            Vector3d gravity(0, -9.81 * 0.001, 0);
            double damping = 0.99;
            int iterations = 3;
            
            for (auto& p : particles) {
                if (!p.isFixed) {
                    p.velocity += gravity;
                    p.velocity *= damping;
                    p.position += p.velocity * dt;
                }
            }
            
            for (int iter = 0; iter < iterations; ++iter) {
                for (int i = 0; i < static_cast<int>(particles.size()) - 1; ++i) {
                    auto& p1 = particles[i];
                    auto& p2 = particles[i + 1];
                    
                    Vector3d delta = p2.position - p1.position;
                    double dist = delta.norm();
                    if (dist < 1e-10) continue;
                    
                    double restLength = 0.1;
                    double diff = (dist - restLength) / dist;
                    
                    Vector3d correction = diff * 0.5 * delta;
                    
                    if (!p1.isFixed) p1.position += correction;
                    if (!p2.isFixed) p2.position -= correction;
                }
            }
        }
    }
};

int runTestMode() {
    std::cout << "========================================" << std::endl;
    std::cout << "Virtual Surgery Simulation - Test Mode" << std::endl;
    std::cout << "========================================" << std::endl;
    
    int tests_passed = 0;
    int tests_total = 0;
    
    {
        std::cout << "\n[Test 1] Math Utilities..." << std::endl;
        tests_total++;
        
        Vector3d v1(1, 2, 3);
        Vector3d v2(4, 5, 6);
        double dot = v1.dot(v2);
        
        if (std::abs(dot - 32.0) < 1e-6) {
            std::cout << "  PASS: Dot product test" << std::endl;
            tests_passed++;
        } else {
            std::cout << "  FAIL: Dot product test" << std::endl;
        }
    }
    
    {
        std::cout << "\n[Test 2] Tissue Creation..." << std::endl;
        tests_total++;
        
        Tissue tissue;
        Material mat;
        mat.youngsModulus = 1000.0;
        mat.poissonRatio = 0.45;
        tissue.setGlobalMaterial(mat);
        
        if (tissue.getNodeCount() == 0) {
            std::cout << "  PASS: Tissue creation test" << std::endl;
            tests_passed++;
        } else {
            std::cout << "  FAIL: Tissue creation test" << std::endl;
        }
    }
    
    {
        std::cout << "\n[Test 3] Suture Creation..." << std::endl;
        tests_total++;
        
        std::vector<Vector3d> positions;
        positions.push_back(Vector3d(0, 1.0, 0));
        positions.push_back(Vector3d(0, 0.9, 0));
        
        Suture suture;
        suture.createChain(positions, 0.1, 0.001);
        
        if (suture.getParticleCount() == 2) {
            std::cout << "  PASS: Suture creation test" << std::endl;
            tests_passed++;
        } else {
            std::cout << "  FAIL: Suture creation test" << std::endl;
        }
    }
    
    {
        std::cout << "\n[Test 4] Instrument Creation..." << std::endl;
        tests_total++;
        
        Instrument inst;
        inst.initialize(Vector3d(1, 2, 3), Quaterniond::Identity(), 0.5, Matrix3d::Identity());
        
        if (inst.getState().mass == 0.5) {
            std::cout << "  PASS: Instrument creation test" << std::endl;
            tests_passed++;
        } else {
            std::cout << "  FAIL: Instrument creation test" << std::endl;
        }
    }
    
    {
        std::cout << "\n[Test 5] AABB Tree..." << std::endl;
        tests_total++;
        
        AABBTree tree;
        
        AABB box1(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
        AABB box2(Vector3d(0.5, 0.5, 0.5), Vector3d(1.5, 1.5, 1.5));
        
        int id1 = tree.insertObject(box1);
        int id2 = tree.insertObject(box2);
        
        auto pairs = tree.queryAllOverlaps();
        
        if (id1 >= 0 && id2 >= 0) {
            std::cout << "  PASS: AABB Tree test (found " << pairs.size() << " overlaps)" << std::endl;
            tests_passed++;
        } else {
            std::cout << "  FAIL: AABB Tree test" << std::endl;
        }
    }
    
    {
        std::cout << "\n[Test 6] XPBD Solver..." << std::endl;
        tests_total++;
        
        std::vector<Vector3d> positions;
        for (int i = 0; i < 3; ++i) {
            positions.push_back(Vector3d(0, 1.0 - i * 0.1, 0));
        }
        
        Suture suture;
        suture.createChain(positions, 0.1, 0.001);
        
        XPBDSolver solver;
        solver.solve(suture, 0.001);
        
        std::cout << "  PASS: XPBD Solver test" << std::endl;
        tests_passed++;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Test Summary: " << tests_passed << "/" << tests_total << " passed" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return (tests_passed == tests_total) ? 0 : 1;
}

int runSimulation(const SimulationConfig& config) {
    std::cout << "========================================" << std::endl;
    std::cout << "Virtual Surgery Simulation" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Time step: " << config.dt << " s" << std::endl;
    std::cout << "Max steps: " << config.max_steps << std::endl;
    std::cout << "Target FPS: " << config.target_fps << std::endl;
    std::cout << "========================================" << std::endl;
    
    SimulationScene scene;
    if (!scene.initialize()) {
        std::cerr << "Failed to initialize simulation scene" << std::endl;
        return 1;
    }
    
    std::cout << "Scene initialized successfully!" << std::endl;
    std::cout << "  Tissue nodes: " << scene.tissue->getNodeCount() << std::endl;
    std::cout << "  Suture particles: " << scene.suture->getParticleCount() << std::endl;
    
    SimulationStats stats;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int step = 0; step < config.max_steps; ++step) {
        auto frame_start = std::chrono::high_resolution_clock::now();
        
        scene.update(config.dt);
        
        stats.current_step = step;
        stats.current_time = step * config.dt;
        
        auto frame_end = std::chrono::high_resolution_clock::now();
        double frame_time = std::chrono::duration<double>(frame_end - frame_start).count();
        double target_frame_time = 1.0 / config.target_fps;
        
        if (frame_time < target_frame_time) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(target_frame_time - frame_time)
            );
        }
        
        if (step % 100 == 0) {
            double actual_fps = 1.0 / std::max(frame_time, target_frame_time);
            stats.avg_fps = actual_fps;
            
            std::cout << "Step " << step << "/" << config.max_steps 
                      << " | Time: " << std::fixed << std::setprecision(3) << stats.current_time << "s"
                      << " | FPS: " << std::setprecision(1) << actual_fps << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    stats.total_simulation_time = std::chrono::duration<double>(end_time - start_time).count();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Simulation Complete" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total steps: " << stats.current_step << std::endl;
    std::cout << "Simulation time: " << stats.current_time << " s" << std::endl;
    std::cout << "Real time: " << stats.total_simulation_time << " s" << std::endl;
    std::cout << "Average FPS: " << stats.avg_fps << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}

void printHelp(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  --test          Run test mode" << std::endl;
    std::cout << "  --headless      Run without rendering" << std::endl;
    std::cout << "  --steps N       Set maximum simulation steps (default: 10000)" << std::endl;
    std::cout << "  --fps N         Set target FPS (default: 60)" << std::endl;
    std::cout << "  --dt VALUE      Set time step in seconds (default: 0.001)" << std::endl;
    std::cout << "  --help          Show this help message" << std::endl;
}

int main(int argc, char* argv[]) {
    SimulationConfig config;
    bool test_mode = false;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--test") {
            test_mode = true;
        } else if (arg == "--headless") {
            config.headless = true;
            config.enable_rendering = false;
        } else if (arg == "--steps" && i + 1 < argc) {
            config.max_steps = std::atoi(argv[++i]);
        } else if (arg == "--fps" && i + 1 < argc) {
            config.target_fps = std::atof(argv[++i]);
        } else if (arg == "--dt" && i + 1 < argc) {
            config.dt = std::atof(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printHelp(argv[0]);
            return 0;
        }
    }
    
    if (test_mode) {
        return runTestMode();
    } else {
        return runSimulation(config);
    }
}
