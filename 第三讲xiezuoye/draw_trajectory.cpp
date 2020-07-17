#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";
string estimated_file = "./estimated.txt";
string groundtruth_file = "./groundtruth.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {
    typedef Eigen::Matrix<double ,6,1> Vector6d;//***会发现在Sophus中，李代数se(3)的平移在前，旋转在后.
 //   Vector6d se3_groundtruth,se3_estimated;
    Vector6d e;
    Eigen::Matrix<double ,4,4> e3;
    double sum=0,eq=0,answer=0;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses,poses_estimated,poses_groundtruth,e2;
    vector<Vector6d> se3_groundtruth,se3_estimated;
    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream inn(trajectory_file);//*****创建一个文件输入的流inn
    ifstream estimated_in(estimated_file);
    ifstream groundtruth_in(groundtruth_file);
    double t1,tx,ty,tz,qx,qy,qz,qw;
    string line;
    string estimated_line;
    string groundtrueh_line;
 //   double sum=0;
    if(inn)
    {
        while(getline(inn,line))
        {
          //  cout<<line<<endl;
            stringstream r(line);    //从string读取数据
            r>>t1>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
            Eigen::Vector3d t(tx,ty,tz);
            Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();  //四元数的顺序要注意
            Sophus::SE3 SE3_qt(q,t);//***定义的是李群

            poses.push_back(SE3_qt);//将所有位姿拼接
        }
    }
    if(estimated_in)
    {
        cout<<"estimated"<<endl;

        while(getline(estimated_in,estimated_line))
        {
            cout<<estimated_line<<endl;
            stringstream r(estimated_line);    //从string读取数据
            r>>t1>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
            Eigen::Vector3d t(tx,ty,tz);
            Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();  //四元数的顺序要注意
            Sophus::SE3 SE3_qt_e(q,t);//***定义的是李群        ***sophus 将李群转换成李代数显示

            se3_estimated.push_back(SE3_qt_e.log());
            poses_estimated.push_back(SE3_qt_e);//将所有位姿拼接
        }
        cout<<poses_estimated.size()<<endl;
    }
    if(groundtruth_in)
    {
        cout<<"groundtruth"<<endl;

        while(getline(groundtruth_in,groundtrueh_line))
        {
            cout<<groundtrueh_line<<endl;
            stringstream r(groundtrueh_line);    //从string读取数据
            r>>t1>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
            Eigen::Vector3d t(tx,ty,tz);
            Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();  //四元数的顺序要注意
            Sophus::SE3 SE3_qt_g(q,t);//李代数表示位姿

            se3_groundtruth.push_back(SE3_qt_g.log());
            cout<<SE3_qt_g<<endl;
//            cout<<se3_groundtruth<<endl;

            poses_groundtruth.push_back(SE3_qt_g);//将所有位姿拼接
        }
        cout<<"poses_estimated_size:"<<poses_estimated.size()<<endl;
        cout<<"poses_groundtruth_size:"<<poses_groundtruth.size()<<endl;
        cout<<"se3_estimated_size:"<<se3_estimated.size()<<endl;
        cout<<"se3_groundtruth_size:"<<se3_groundtruth.size()<<endl;

    }
    else
    {
        cout<<"没找到这个文件"<<endl;
    }
    std::cout<<"poses_groundtruth[0].inverse():"<<poses_groundtruth[0].inverse()<<std::endl;
    std::cout<<"poses_groundtruth[0].matrix():"<<poses_groundtruth[0].matrix()<<std::endl;

    for(int i=0;i<poses_groundtruth.size();i++)
    {
        e=(poses_groundtruth[i].inverse()*poses_estimated[i]).log();//××××××××李群.log()转化成李代数   在sophus中没有定义李代数se3，李代数se3实质上就是一个六维向量。
        eq=e.norm()*e.norm();
        sum=eq+sum;
//        std::cout<<e.norm()<<std::endl;

    }
    answer=sum/se3_groundtruth.size();
    answer=sqrt(answer);
    std::cout<<"answer is :"<<answer<<std::endl;
    Eigen::Vector3d so3(1,1,1);
    Sophus::SO3 SO3=Sophus::SO3::exp(so3);
    std::cout<<"SO3:"<<SO3<<std::endl;
    std::cout<<"SO3.log:"<<SO3.log()<<std::endl;
    std::cout<<"so3.norm:"<<so3.norm()*so3.norm()<<std::endl;

    estimated_in.close();
    groundtruth_in.close();
    inn.close();


    // end your code here

    // draw trajectory in pangolin

    // end your code here

    // draw trajectory in pangolin

    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses)
    {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);//*****先建立一个空白窗口
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);//se3通过函数：rotationMatrix()，translation()转换出的就是旋转矩阵，平移向量
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}