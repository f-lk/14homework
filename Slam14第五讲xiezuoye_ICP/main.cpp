#include <Dense>
#include <Core>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pangolin/pangolin.h>

using namespace std ;

string campare_file="./compare.txt";
typedef vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>>  matrix_T; //定义四维变换矩阵
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> vector_P;

void SVD(vector_P & Pe, vector_P & Pg, Eigen::Matrix3d & R,Eigen::Vector3d & p_e,Eigen::Vector3d & p_g);
void compute_t(Eigen::Vector3d & p_e,Eigen::Vector3d & p_g,Eigen::Matrix3d & R,Eigen::Vector3d & t);
void DrawTrajectory(vector_P & poses,vector_P & poses_2);


int main ()
{
    matrix_T Te ,Tg;
    vector_P Pe ,Pg,Pg_2;

    Eigen::Matrix3d R=Eigen::Matrix3d::Zero();//旋转矩阵
    Eigen::Vector3d p_e=Eigen::Vector3d::Zero();//质心坐标
    Eigen::Vector3d p_g=Eigen::Vector3d::Zero();
    Eigen::Vector3d t=Eigen::Vector3d::Zero();//平移向量
    Eigen::Vector3d pg_2=Eigen::Vector3d::Zero();//平移向量


    ifstream file_in (campare_file);
    double data[16]={0};

    for(int i=0;i<612;i++)
    {
        for(auto & d:data)
        {
            file_in>>d;
        }
        Eigen::Quaterniond q(data[4],data[5],data[6],data[7]);
        Eigen::Isometry3d TE(q);
        TE.pretranslate(Eigen::Vector3d(data[1],data[2],data[3]));
        Eigen::Vector3d pe(data[1],data[2],data[3]);//平移向量既是点的坐标
        Te.push_back(TE);
        Pe.push_back(pe);
        //cout<<"data1"<<data[1]<<endl;
        Eigen::Quaterniond qg(data[12],data[13],data[14],data[15]);
        Eigen::Isometry3d TG(qg);
        TE.pretranslate(Eigen::Vector3d(data[9],data[10],data[11]));
        Eigen::Vector3d pg(data[9],data[10],data[11]);
        Tg.push_back(TG);
        Pg.push_back(pg);
    }

    cout<<"Te.size():"<<Te.size()<<endl;
    SVD(Pe,Pg,R,p_e,p_g);
    compute_t(p_e,p_g,R,t);
   // DrawTrajectory(Pe,Pg);
    for(int i=0;i<Pg.size();i++)
    {
        pg_2=R*Pg[i]+t;
        Pg_2.push_back(pg_2);
    }
    DrawTrajectory(Pe,Pg_2);




    return 0;
}
//参数分别为点的坐标Pe Pg,旋转矩阵R  两组点的质心p_e，p_g
void SVD(vector_P & Pe, vector_P & Pg, Eigen::Matrix3d & R,Eigen::Vector3d & p_e,Eigen::Vector3d & p_g)
{
    p_e=Eigen::Vector3d::Zero();
    p_g=Eigen::Vector3d::Zero();//质心位置

    Eigen::Vector3d q=Eigen::Vector3d::Zero();

    vector_P qe,qg;
    Eigen::Matrix3d W_2=Eigen::Matrix3d::Zero();
    Eigen::Matrix3d W=Eigen::Matrix3d::Zero();
    //计算质心坐标
    for(int i=0;i<Pe.size();i++)
    {
        p_e+=Pe[i];
    }
    p_e=p_e/Pe.size();//质心位置
    cout<<"Pe质心"<<p_e<<endl;
    for(int j=0;j<Pe.size();j++)
    {
        p_g+=Pg[j];
    }
    p_g=p_g/Pg.size();
    cout<<"Pg质心"<<p_g<<endl;

    for(int j=0;j<Pe.size();j++)
    {
        q=Pe[j]-p_e;//去质心位置
        qe.push_back(q);
    }
    for(int j=0;j<Pg.size();j++)
    {
        q=Pg[j]-p_g;
        qg.push_back(q);
    }
    for(int j=0;j<qe.size();j++)
    {
        W_2=qe[j]*qg[j].transpose();
        W+=W_2;
    }
    cout<<"W:"<<W<<endl;
    //EIGEN中的SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U=svd.matrixU();
    Eigen::Matrix3d V=svd.matrixV();
    cout<<"u:"<<U<<endl<<"v:"<<V<<endl;
    R=U*V.transpose();
    cout<<"R:"<<R<<endl;




}


//输入参数：两组点的质心 旋转矩阵  和要求的平移向量
void compute_t(Eigen::Vector3d & p_e,Eigen::Vector3d & p_g,Eigen::Matrix3d & R,Eigen::Vector3d & t)
{
    t=p_e-R*p_g;
    cout<<"t"<<t<<endl;
}



void DrawTrajectory(vector_P & poses,vector_P & poses_2)
{
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("uuu", 1024, 768);//*****先建立一个空白窗口
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 10, 0, 0, 0, 0, 0.0, 0, 1.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout<<"pose.size"<<poses.size()<<endl;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() -1; i++) {
            glColor3f(0,0,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            //cout<<"pose[i]"<<p1<<endl;

            glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses_2.size() -1; i++) {
            glColor3f(0,255,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
            glBegin(GL_LINES);
            auto p1 = poses_2[i], p2 = poses_2[i + 1];
           // cout<<"pose[i]"<<p1<<endl;

            glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }
        glColor3f(255,0,0);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3d(2,0,0);
        glEnd();
        glColor3f(255,255,0);
        glBegin(GL_LINES);
        glVertex3f(0,0,5);
        glVertex3f(0,0,0);
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }


}
