#include <iostream>
#include <Dense>
#include <Core>
#include <string>
#include <opencv2/core/core.hpp>


using namespace std;

int main()
{
    int a=1,b=2,c=3;
    Eigen::Vector3d abc(1,1,1);

    vector<double > x_data;
    vector<double > y_data;
    cv::RNG rng;
    int N =120;//生成点的个数。
    int iteration = 100;//迭代的次数
    //生成数据点
    for(int i=0;i<N;i++)
    {
        double x=(double)i/120;
        x_data.push_back(x);
        double y= exp(a*x*x+b*x+c)+rng.gaussian(1);//生成的数据点加上方差为1的高斯噪声
        y_data.push_back(y);
        cout<<"i:"<<i<<",";
        cout<<"x:"<<x_data[i]<<",";
        cout<<"y:"<<y_data[i]<<endl;
    }
    Eigen::Vector3d J=Eigen::Vector3d::Zero();

    Eigen::Vector3d deta_abc=Eigen::Vector3d::Zero();

    double lastcost =0 ,cost =0;//上一次的cost  和本次的cost
    for(int j=0;j<iteration;j++)
    {
        Eigen::Matrix3d H=Eigen::Matrix3d::Zero();
        Eigen::Vector3d g=Eigen::Vector3d::Zero();
        cost=0;
        for(int i=0;i<N;i++)//对于第j次迭代，计算所有数据的H只和作为这次迭代的H    计算所有数据的g之和作为这次迭代的g    来求解H*deta_abc=g;
        {
            Eigen::Matrix3d H_2=Eigen::Matrix3d::Zero();
            Eigen::Vector3d g_2=Eigen::Vector3d::Zero();
            J[0]=-x_data[i]*x_data[i]*exp(abc[0]*x_data[i]*x_data[i]+abc[1]*x_data[i]+abc[2]);//对a求导
            J[1]=-x_data[i]*exp(abc[0]*x_data[i]*x_data[i]+abc[1]*x_data[i]+abc[2]);//对b求导
            J[2]=-exp(abc[0]*x_data[i]*x_data[i]+abc[1]*x_data[i]+abc[2]);//对c求导

            H_2=J*J.transpose();

            double error =y_data[i]-exp(abc[0]*x_data[i]*x_data[i]+abc[1]*x_data[i]+abc[2]);

            g_2=-J*error;


            H+= H_2;

            g+=g_2;

            cost+=error*error;
        }
        //  求 HX=g
        deta_abc=H.inverse()*g;

        abc+=deta_abc;

        if(isnan(deta_abc[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }

        if(j>0  &&  cost > lastcost)//这次迭代的误差大于上次迭代的误差  效果不好
        {
            cout<<"the result is not good "<<endl;
        }

        lastcost=cost;

        cout<<"cost:"<<cost<<endl;

    }
    cout<<abc<<endl;



        return 0;
}