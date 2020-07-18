#include<iostream>
#include <Dense>
#include <Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);
double fx=718.856,fy=718.856,cx=607.1928,cy=185.2157;
double B=0.573;//基线长度单位是米

int main()
{
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>  point_cloud;

    cv::Mat left,right,disparity;
    //  读取left.png
    left=cv::imread("./left.png",0);
    if ( left.data == nullptr ) //数据不存在,可能是文件不存在
    {
        std::cerr<<"文件"<<"letf"<<"不存在."<<std::endl;
        return 0;
    }

    // 文件顺利读取, 首先输出一些基本信息
    std::cout<<"left图像列为"<<left.cols<<",行为"<<left.rows<<",通道数为"<<left.channels()<<std::endl;
//    cv::imshow ( "left", left );      // 用cv::imshow显示图像
//    cv::waitKey ( 0 );                  // 暂停程序,等待一个按键输入

//    读取right.png
    right=cv::imread("./right.png",0);
    if ( right.data == nullptr ) //数据不存在,可能是文件不存在
    {
        std::cerr<<"文件"<<"right"<<"不存在."<<std::endl;
        return 0;
    }

    // 文件顺利读取, 首先输出一些基本信息
    std::cout<<"right图像列为"<<right.cols<<",行为"<<right.rows<<",通道数为"<<right.channels()<<std::endl;
//    cv::imshow ( "right", right );      // 用cv::imshow显示图像
//    cv::waitKey ( 0 );                  // 暂停程序,等待一个按键输入

    //  读取disparity.png
    disparity=cv::imread("./disparity.png",0);
    if ( disparity.data == nullptr ) //数据不存在,可能是文件不存在
    {
        std::cerr<<"文件"<<"disparity"<<"不存在."<<std::endl;
        return 0;
    }

    // 文件顺利读取, 首先输出一些基本信息
    std::cout<<"disparity图像列为"<<disparity.cols<<",行为"<<disparity.rows<<",通道数为"<<disparity.channels()<<std::endl;
//    cv::imshow ( "disparity", disparity );      // 用cv::imshow显示图像
//    cv::waitKey ( 0 );                  // 暂停程序,等待一个按键输入



    for(int v=0;v<left.rows;v++ ) //row  行数   y
    {
        for (int u = 0; u < left.cols; u++)//  col  列数   x
        {
            unsigned int disp = disparity.ptr<unsigned short>(v)[u];//disparity.at<uchar>(v,u)   为什么不行？？？？？？？？？？？
            if(disp==0)
            {
                cout<<"disp=0"<<endl;
                continue;
            }
            double D=(B*fx*1000)/disp;

            double x=(u-cx)*D/fx;
            double y=(v-cy)*D/fy;
            Eigen::Vector4d p;
            p[0]=x;
            p[1]=y;
            p[2]=D;
            p[3]=(double)left.at<uchar >(v,u)/255;
            point_cloud.push_back(p);
 //           std::cout<<(int)disparity.at<uchar >(v,u)<<","<<std::endl;
        }
    }
  //
    std::cout<<"点云中点的个数："<<point_cloud.size()<<std::endl;
    showPointCloud(point_cloud);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)   //第一组eyex, eyey,eyez 相机在世界坐标的位置,就是脑袋的位置;第二组centerx,centery,centerz 相机镜头对准的物体在世界坐标的位置,就是眼睛看的物体的位置;第三组upx,upy,upz 相机向上的方向在世界坐标中的方向,就是头顶朝向的方向（因为你可以歪着头看同一个物体）

    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
//上面都是设置参数初始化
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//清除颜色缓冲，清除深度缓冲

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//表示是float

        glPointSize(1);//点云中点的大小
        glBegin(GL_POINTS);//把每个顶点作为一个点进行处理，顶点n定义了点n，绘制N个点。
        for (auto &p: pointcloud) {                //引用相当于指针再取值.引用就是指针，两者没有区别。我们可以把引用想象成一个不需要"*"就可以访问变量的指针
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return ;
}


