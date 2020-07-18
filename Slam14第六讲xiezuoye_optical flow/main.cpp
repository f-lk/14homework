#include <Dense>
#include <Core>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

string first_image_diret ="1.png";
string second_image_diret ="2.png";


int main()
{
    cv::Mat first_image=cv::imread(first_image_diret,0);
    cv::Mat second_image=cv::imread(second_image_diret,0);
    cout<<"first_image"<<"col,列数:"<<first_image.cols<<"            row:行数"<<first_image.rows<<endl;
    cout<<"second_image"<<"col,列数:"<<second_image.cols<<"          row:行数"<<second_image.rows<<endl;
    cv::imshow("first",first_image);
    cv::imshow("second",second_image);

    return 0;
}


