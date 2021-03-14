/*
    fk-3link-planar.cc
        Sample c++ source code of forward kinematics of 3-link planar robot arm
        Author: Keitaro Naruse
        Date:   2021-03-13
        Required external library: Eigen
        The MIT License
*/
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

/*
    fk()
        forward kinematics function of 3-link planar robot arm
        input: q, a joint angle vector ( q(0), q(1), q(2) )'
        output: p, a pose matrix(3, 4)
        a pose vector ( p(0), p(1), p(2), p(3) )
            p(0) = (x0, y0, q0)': Pose of the first link = the base
            p(1) = (x1, y1, q1)': Pose of the second link = the end of the first link 
            p(2) = (x2, y2, q2)': Pose of the third link = the end of the second link 
            p(3) = (x3, y3, q3)': Pose of the hand tip = the end of the third link 
*/
Eigen::Matrix<double, 3, 4> fk(const Eigen::Vector3d& q)
{
    //  Robot arm link parameters
    const double L1 = 1.0, L2 = 1.0, L3 = 1.0;

    //  A pose vector of joints and hand
    Eigen::Vector3d p0, p1, p2, p3;
    
    //  A vector of the pose vectors as a matrix
    //  p = (p0, p1, p2, p3)
    Eigen::Matrix<double, 3, 4> p;

    //  Forward kinematics calculation 
    //  p0: A pose of the first joint = the base
    //  Set as (x0, y0, q0)' = (0,0,0)'
    p0 << 0.0, 0.0, 0.0;
    //  Assign p0 to the 0th column of the matrix p
    p.col(0) = p0;

    //  p1: A pose of the second joint = the end of the first link
    //  Set p1 = p0 + (L1*cos(q(1)), L1*sin(q(1)), q1)'
    p1 << L1 * std::cos(q(0)), L1 * std::sin(q(0)), q(0);
    p1 = p0 + p1;
    //  Assign p1 to the 1st column of the matrix p
    p.col(1) = p1;

    //  p2: A pose of the third joint = the end of the second link
    //  Set p2 = p1 + (L2*cos(q(1)+q(2)), L2*sin(q(1)+q(2)), q2)'
    p2 << L2 * std::cos(q(0)+q(1)), L2 * std::sin(q(0)+q(1)), q(1);
    p2 = p1 + p2;
    //  Assign p2 to the 2nd column of the matrix p
    p.col(2) = p2;

    //  p3: A pose of the hand tip = the end of the third link
    //  Set p3 = p2 + (L3*cos(q(0)+q(1)+q(2)), L3*sin(q(0)+q(1)+q(2)), q(2))'
    p3 << L3 * std::cos(q(0)+q(1)+q(2)), L3 * std::sin(q(0)+q(1)+q(2)), q(2);
    p3 = p2 + p3;
    //  Assign p3 to the 3rd column of the matrix p
    p.col(3) = p3;

    return(p);
}

int main()
{
    //  A joint angle vector (q1, q2, q3)
    Eigen::Vector3d q;
    //  Initial value is (0.1, 0.4, 0.9) [rad]
    q << 0.1, 0.4, 0.9;

    //  A set of poses as a matrix
    //  p = (p0, p1, p2, p3)
    Eigen::Matrix<double, 3, 4> p;

    //  Forward kinematics solution
    p = fk(q);

    //  Console out the joint angle vector
    std::cout << q << std::endl;

    //  Console out the pose matrix = a vector of poses
    std::cout << p << std::endl;
    return(0);
}
