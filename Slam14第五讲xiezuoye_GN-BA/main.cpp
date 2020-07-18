#include "sophus/se3.h"
#include "sophus/se3.h"
#include <Dense>
#include <Core>
#include <Geometry>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>



using namespace std;

string p3d_file ="./p3d.txt";
string p2d_file = "./p2d.txt";
double fx=520.9,fy=521.0,cx=325.1,cy=249.7;

void bundleAdjustment (
        const vector< cv::Point3f > points_3d,
        const vector< cv::Point2f > points_2d,
        const cv::Mat& K,
        cv::Mat& R, cv::Mat& t );


int main ()
{

    vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> vector_p3d;
    vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> vector_p2d;

    vector<cv::Point3f> points_3d;
    vector< cv::Point2f > points_2d;

    Eigen::Matrix<double ,3,3> K ;
    K<<520.9,0,325.1,0,521.0,249.7,0,0,1;

    Eigen::Matrix<double ,2,6> J=Eigen::Matrix<double ,2,6>::Zero();

    Eigen::Matrix <double ,1,6> se3 =Eigen::Matrix <double ,1,6>::Zero();

    int iteration =100;
    double  data_p3d [3]={0,0,0};
    double  data_p2d [2]={0,0};

    Eigen::Vector3d data_p3d_2;
    Eigen::Vector2d data_p2d_2;
    cv::Point3f points_3d_2;
    cv::Point2f points_2d_2;

    ifstream fin_p3d("./p3d.txt");
    ifstream fin_p2d("./p2d.txt");
    for(int i=0;i<76;i++)
    {
        for (auto & d_p3d:data_p3d)
        {
            fin_p3d>>d_p3d;

        }
        for(int j=0;j<3;j++)
        {
            data_p3d_2[j]=data_p3d[j];
        }
        points_3d_2.x=data_p3d[0];
        points_3d_2.y=data_p3d[1];
        points_3d_2.z=data_p3d[2];
        points_3d.push_back(points_3d_2);
        vector_p3d.push_back(data_p3d_2);
       // cout <<"p3d:"<<vector_p3d[i]<<endl;
    }
    for(int i=0;i<76;i++)
    {
        for (auto & d_p2d:data_p2d)
        {
            fin_p2d>>d_p2d;

        }
        for(int j=0;j<2;j++)
        {
            data_p2d_2[j]=data_p2d[j];
        }
        points_2d_2.x=data_p2d_2[0];
        points_2d_2.y=data_p2d_2[1];

        points_2d.push_back(points_2d_2);
        vector_p2d.push_back(data_p2d_2);
       // cout <<"p2d:"<<vector_p2d[i]<<endl;
    }
    cout<<"cv"<<points_3d[18]<<endl;
    cout<<"ve"<<vector_p3d[18]<<endl;


    Eigen::Matrix3d initial_R;
    Eigen::Vector3d initial_t;
    initial_t<<0,0,0;
    initial_R<<1,0,0,0,1,0,0,0,1;
    Sophus::SE3 initial_T(initial_R,initial_t);//李群形式
    cout<<"initial_T"<<initial_T<<endl;//本来这里的李群应该是矩阵，但是输出时都被转换成李代数进行输出了。所以结果形式上是一个6维向量。

    Eigen::Matrix<double ,6,1> T=initial_T.log();//李代数形式
    cout<<"T"<<T<<endl;

    cv::Mat t_initial_mat=(cv::Mat_<double>(3,1)<<0,0,0);
    cv::Mat R_initial_mat=(cv::Mat_<double >(3,3)<<1,0,0,0,1,0,0,0,1);
    cv::Mat K_mat=(cv::Mat_<double >(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);





    bundleAdjustment(points_3d,points_2d,K_mat,R_initial_mat,t_initial_mat);

    for (int i=0;i<iteration;i++)//会发现在Sophus中，se(3)的平移在前，旋转在后.
    {
        Eigen::Matrix<double ,6,6> H=Eigen::Matrix<double ,6,6>::Zero();

        Eigen::Matrix<double ,6,1> g=Eigen::Matrix<double ,6,1>::Zero();

        Eigen::Vector3d P_pian,P=Eigen::Vector3d::Zero();

        Eigen::Matrix<double ,6,1> deta_T=Eigen::Matrix<double ,6,1>::Zero();//李代数
        for(int j=0;j<vector_p3d.size();j++)
        {
            Eigen::Matrix<double ,6,6> H_2=Eigen::Matrix<double ,6,6>::Zero();
            Eigen::Matrix<double ,6,1> g_2=Eigen::Matrix<double ,6,1>::Zero();
            Eigen::Vector2d f,uv=Eigen::Vector2d::Zero();

            P_pian=initial_T*vector_p3d[j];//世界左边系下的坐标转换到相机坐标系
            P=K*P_pian;//相机坐标系转换到像素平面坐标（齐次坐标）
            uv[0]=P[0]/P[2];//得到非齐次坐标，既像素平面坐标
            uv[1]=P[1]/P[2];
            //cout<<"Ppian"<<P_pian<<"p3d"<<vector_p3d[j]<<endl;
            double X_pian =0,Y_pian=0,Z_pian=0;
            X_pian=P_pian[0];
            Y_pian=P_pian[1];
            Z_pian=P_pian[2];
            //雅克比矩阵
            J<<fx/Z_pian,    0     ,-fx*X_pian/(Z_pian*Z_pian)  ,  -fx*X_pian*Y_pian/(Z_pian*Z_pian)    , fx+fx*X_pian*X_pian/(Z_pian*Z_pian)  , -fx*Y_pian/Z_pian,
                0,     fy/Z_pian    ,-fx*Y_pian/(Z_pian*Z_pian) ,  -fy-fy*Y_pian*Y_pian/Z_pian*Z_pian   , fy*X_pian*Y_pian/(Z_pian*Z_pian)     , fy*X_pian/Z_pian;
            J=-J;
            //代价函数
            f[0]=vector_p2d[j][0]-uv[0];
            f[1]=vector_p2d[j][1]-uv[1];
//            cout<<"f"<<f<<endl;
            H_2=J.transpose()*J;
            g_2=-J.transpose()*f;//f取的有问题
//            cout<<"H_2"<<H_2<<endl;
//            cout<<"g_2"<<g_2<<endl;
//            cout<<"J"<<J<<endl;
            H+=H_2;
            g+=g_2;
        }
//        cout<<"H"<<H<<endl;
//        cout<<"g"<<g<<endl;
        deta_T=H.inverse()*g;
//        cout<<"deta_T"<<deta_T<<endl;
        initial_T =Sophus::SE3::exp(deta_T)*initial_T;//左乘更新
    }
//    Sophus::SE3 SE3=Sophus::SE3::exp(T);
    cout<<"FINALLY"<<endl<<initial_T.matrix()<<endl;


    return 0;
}

void bundleAdjustment (
        const vector< cv::Point3f > points_3d,
        const vector< cv::Point2f > points_2d,
        const cv::Mat& K,
        cv::Mat& R, cv::Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    //   Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    //Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
            R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
            R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
            R_mat,
            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
    ) );
    optimizer.addVertex ( pose );
//三维空间点坐标，vertex
    int index = 1;
    for ( const cv::Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges   图像像素坐标
    index = 1;
    for ( const cv::Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );// 观测数值
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
}