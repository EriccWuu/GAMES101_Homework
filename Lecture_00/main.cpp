#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define PI acos(-1)

int main(){

    // // Basic Example of cpp
    // std::cout << "Example of cpp \n";
    // float a = 1.0, b = 2.0;
    // std::cout << a << std::endl;
    // std::cout << a/b << std::endl;
    // std::cout << std::sqrt(b) << std::endl;
    // std::cout << std::acos(-1) << std::endl;
    // std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    // std::cout << "Example of vector \n";
    // vector definition
    // Eigen::Vector3f v(1.0f,2.0f,3.0f);
    // Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // // vector output
    // std::cout << "Example of output \n";
    // std::cout << v << std::endl;
    // // vector add
    // std::cout << "Example of add \n";
    // std::cout << v + w << std::endl;
    // // vector scalar multiply
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v * 3.0f << std::endl;
    // std::cout << 2.0f * v << std::endl;
    // // vector-vector multiply
    // std::cout << "Example of scalar multiply \n";
    // std::cout << v.adjoint() * w << std::endl; // P.adjoint() transpose the vector
    // std::cout << v.norm() << std::endl; // mod of v
    // std::cout << v.dot(w) / v.norm() / w.norm() << std::endl; // angel between v and w

    // // Example of matrix
    // std::cout << "Example of matrix \n";
    // // matrix definition
    // Eigen::Matrix3f i,j;
    // i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    // j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // // matrix output
    // std::cout << "Example of output \n";
    // std::cout << i << std::endl;
    // std::cout << i + j << std::endl;
    // std::cout << i * 2.0 << std::endl;
    // std::cout << i * j << std::endl;
    // std::cout << i * w << std::endl;
    
    // // matrix add i + j
    // // matrix scalar multiply i * 2.0
    // // matrix multiply i * j
    // // matrix multiply vector i * v
    Eigen::Matrix<float, 3, 3> m_rota, n_trans, t;
    Eigen::Vector3f p(2.0f, 1.0f, 1.0f);

    m_rota << cos(PI/4), -sin(PI/4), 0,
              sin(PI/4), cos(PI/4), 0,
              0, 0, 1.0f;
    n_trans << 0, 0, 1,
               0, 0, 2,
               0, 0, 1;
    
    std::cout << m_rota * n_trans * p << std::endl;

    return 0;
}