#include <iostream>
#include <chrono>
#include <string>
#include <Core>
#include <Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
double k1=-0.28340811,k2=0.07395907,p1=0.00019359,p2=1.76187114e-05;
double fx=458.654,fy=457.296,cx=367.215,cy=248.375;

double distance(double y, double x);

double * transport(double y, double x);

class Point
{
private:
    int x_corrected=0;
    int y_corrected=0;
};

double x_corrected=0,y_corrected=0;

int main()
{
    std::cout<<p2<<std::endl;

    cv::Mat image_before;
    cv::Mat image_after=cv::Mat::zeros(480,752,0);
    std::cout<<"列数colsx:"<<image_after.cols<<"     行数rowsy:"<<image_after.rows<<"    channel:"<<image_after.channels()<<std::endl;
//    cv::imshow("zeros",image_after);
//    cv::waitKey(0);
    image_before=cv::imread("/home/flk/桌面/第04讲/xiezuoye/test.png",0);//
    if(image_before.data== nullptr)
    {
        std::cout<<"wenjian"<<"bucunzai"<<std::endl;
  //      std::cout<<"with"<<image_before.cols<<"high:"<<image_before.rows<<"channel:"<<image_before.channels()<<std::endl;

    } else{
        std::cout<<"wenjian"<<"cunzai"<<std::endl;
        std::cout<<"列数x:"<<image_before.cols<<"     行数y:"<<image_before.rows<<"    channel:"<<image_before.channels()<<std::endl;
        std::cout<<"列数x:"<<image_after.cols<<"     行数y:"<<image_after.rows<<"    channel:"<<image_before.channels()<<std::endl;
    }
    cv::imshow("before",image_before);
//    cv::waitKey(0);
    for(size_t y=0;y<image_before.rows;y++)//row代表行数
    {
        unsigned char* row_ptr=image_before.ptr<unsigned char> (y);//把图像的第y行指针赋给row_ptr
        for(size_t x=0;x<image_before.cols;x++)
        {
            unsigned char* data_ptr=&row_ptr[x*image_before.channels()];


//            *data_ptr=*data_ptr+50;
//            std::cout<<(int)*data_ptr<<std::endl;
//            for(int c=0 ;c!=image_before.channels();c++)
//            {
//                unsigned int data=data_ptr[c];
//                std::cout<<"data"<<std::endl;
//                std::cout<<data<<std::endl;
//            }

        }
    }

    for(int v=0;v<image_before.rows;v++)//v  行数  rows 与y平行
    {
 //       unsigned char* row_ptr=image_before.ptr<unsigned char> (v);//把图像的第y行指针赋给row_ptr
        for(int u=0;u<image_before.cols;u++)//  列数   cols  与x平行
        {
 //           unsigned char* data_ptr=&row_ptr[u*image_before.channels()];//data_ptr指向的是第v row(行)和第u col(列)元素
            //从像素平面转化到归一化平面
            double before_guiyi_x=(u-cx)/fx;
            double before_guiyi_y=(v-cy)/fy;
            //在归一化平面中去畸变
            x_corrected=transport(before_guiyi_y,before_guiyi_x)[1];
            y_corrected=transport(before_guiyi_y,before_guiyi_x)[0];
            //去畸变后的坐标值转化到像素平面
            int after_v=fy*y_corrected+cy;
            int after_u=fx*x_corrected+cx;
            //对去畸变后的坐标赋值
//            if(after_u>=0 && after_u<image_after.cols && after_v>=0 && after_v<image_after.rows)
//            {
//                image_after.at<unsigned char>(v,u)=image_before.at<uchar >((int)after_v,(int)after_u);//(int )after_v,(int)after_u
//                image_after.at<unsigned char>((int)after_v,(int )after_u)=image_before.at<uchar >(v,u);//则会加重畸变
//            } else{
//                image_after.at<unsigned char>(after_v,after_u)=0;
//            }
//          与if中语句产生同样的效果
            unsigned char* row_ptr=image_before.ptr<unsigned char> (after_v);//把图像的第y行指针赋给row_ptr
            unsigned char* data_ptr=&row_ptr[after_u];//data_ptr指向的是第v row(行)和第u col(列)元素

            unsigned char * after_row_ptr=image_after.ptr<unsigned char>(v);
            unsigned char * after_data_ptr=&after_row_ptr[u];
            *after_data_ptr=*data_ptr;
        }

    }
    cv::imshow("image_after",image_after);
    cv::waitKey(0);



//    x= transport(y,x)[1];
//    y= transport(y,x)[0];
//    std::cout<<"x:"<<x<<"y:"<<y<<std::endl;


    std::cout<<"test:"<<"hello"<<std::endl;
    return 0;
}

double distance(double y, double x)
{
    double distance=0;
    distance=sqrt(x*x+y*y);
    return distance;
}

double * transport(double y,double x)
{
    static double xy_corrected[2];
    double r=distance(y,x);

    xy_corrected[0]=y*(1+k1*r*r+k2*pow(r,4))+p1*(r*r+2*y*y)+2*p2*x*y;//y
    xy_corrected[1]=x*(1+k1*r*r+k2*pow(r,4))+2*p1*x*y+p2*(r*r+2*x*x);//x

    return xy_corrected;
}
