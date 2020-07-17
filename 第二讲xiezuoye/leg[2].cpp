//
// Created by flk on 19-11-5.
//
#include <iostream>
#include <Core>
#include <Dense>

int main()
{
    Eigen::Vector3d p1(0.5,-0.1,0.2);
    Eigen::Vector3d t1(0.7,1.1,0.2);
    Eigen::Quaterniond Q1(0.55,0.3,0.2,0.2);
    Q1=Q1.normalized();
    Eigen::Matrix3d R1=Q1.toRotationMatrix();

    std::cout<<Q1.vec()<<std::endl;
    std::cout<<R1<<std::endl;
    Eigen::Quaterniond Q2(-0.1,0.3,-0.7,0.2);
    Q2=Q2.normalized();
    Eigen::Matrix3d R2=Q2.toRotationMatrix();
    std::cout<<Q2.vec()<<std::endl;
    std::cout<<R2<<std::endl;
    Eigen::Vector3d pw=R1.inverse()*(p1-t1);
    std::cout<<pw<<std::endl;
    Eigen::Vector3d t2(-0.1,0.4,0.8);
    Eigen::Vector3d p2=R2*pw+t2;
    std::cout<<p2<<std::endl;
    return 0;
}
