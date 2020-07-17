#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>

using namespace std;


int main(int argv, char ** argc)
{
    cv::Mat image_left, image_right;
    image_left=cv::imread("./left.png",0);
    image_right=cv::imread("./right.png",0);

    if(image_left.empty()||image_right.empty())
    {
        cout<<"not found image"<<endl;
        return 1;
    }
    cv::imshow("left",image_left);
    cv::imshow("right",image_right);
    cv::waitKey(0);

    vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector =cv::GFTTDetector::create(500,0.01,20);
    detector->detect(image_left,kp1);
    cout<<"number of kp1:"<<kp1.size();

    cv::Mat image_left_CV;
    cv::cvtColor(image_left, image_left_CV,CV_GRAY2BGR);
    for(int i=0;i<kp1.size();i++)
    {
        cv::circle(image_left_CV,kp1[i].pt,2,cv::Scalar(0,255,0),2);
    }
    cv::imshow("image_left_CV",image_left_CV);
    cv::waitKey(0);

    return 0;
}
