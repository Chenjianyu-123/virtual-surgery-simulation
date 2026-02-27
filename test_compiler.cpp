#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Vector3d v(1, 2, 3);
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    
    std::cout << "========================================" << std::endl;
    std::cout << "C++ Compiler Test - SUCCESS!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Eigen version: " << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << "Vector: " << v.transpose() << std::endl;
    std::cout << "Matrix:\n" << m << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
