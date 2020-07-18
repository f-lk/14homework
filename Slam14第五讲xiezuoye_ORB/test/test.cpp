#include<iostream>
#include <Dense>
#include <Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;


int main()
{
    vector<cv::KeyPoint> keypoint_1,keypoint_2;
    cv::Mat image_1,descriptor_1,descriptor_2,image_2;
    image_1=cv::imread("./1.png",cv::IMREAD_COLOR);
    image_2=cv::imread("./2.png",cv::IMREAD_COLOR);

    cv::imshow("image_1",image_1);
    cv::imshow("image_2",image_2);

    cv::waitKey(0);

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher=cv::DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(image_1,keypoint_1);
    descriptor->compute(image_1,keypoint_1,descriptor_1);

    detector->detect(image_2,keypoint_2);
    descriptor->compute(image_2,keypoint_2,descriptor_2);

    cv::Mat out1,out2;
    cv::drawKeypoints(image_1,keypoint_1,out1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(image_2,keypoint_2,out2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    cv::imshow("ORB2",out2);
    cv::imshow("ORB",out1);
    cv::waitKey(0);

    vector<cv::DMatch> matches;
    matcher->match(descriptor_1,descriptor_2,matches);

    double max=0,min=10000;
    for(int i=0;i<descriptor_1.rows;i++)
    {
        if(matches[i].distance<min)
        {
            min=matches[i].distance;
        }
        if(matches[i].distance>max)
        {
            max=matches[i].distance;
        }
    }
    cout<<"min:"<<min<<"max"<<max<<endl;
    cv::Mat match_out;
    cv::drawMatches(image_1,keypoint_1,image_2,keypoint_2,matches,match_out,cv::Scalar::all(-1),4);
    vector<cv::DMatch> good_matches;
    for(int i=0;i<descriptor_1.rows;i++)
    {
        if(matches[i].distance<=cv::max(30.0,2*min))
        {
            good_matches.push_back(matches[i]);
        }

    }
    cv::Mat match_out_2;
    cv::drawMatches(image_1,keypoint_1,image_2,keypoint_2,good_matches,match_out_2,cv::Scalar::all(-1),0);
    cv::imshow("match_out",match_out);
    cv::imshow("good match_out_2",match_out_2);
    cv::waitKey(0);




    cout<<"hello"<<endl;
    return 0;
}