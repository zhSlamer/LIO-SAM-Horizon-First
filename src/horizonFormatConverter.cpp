/* 参考tiers数据集中的horizonFormatConver.cpp */
#define PCL_NO_PRECOMPILE
#include "utility.h"
#include "livox_ros_driver/CustomMsg.h"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
#define   PI      3.1415926535

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D                     // 添加XYZ+填充类型的首选方式
    PCL_ADD_INTENSITY;                  // 添加强度
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 确保新的分配器内存是对其的
} EIGEN_ALIGN16;                        // 强制SSE填充以获得正确对齐
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,         // 注册点云类型宏 XYZI+ ring + time (as fields)
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

using Point = VelodynePointXYZIRT;

class LivoxHorizon : public ParamServer
{
public:
    ros::NodeHandle nodeHandler;
    //声明变量
    string frame_id;

    ros::Subscriber sub_livox_horizon;
    ros::Publisher pub_ros_points;
    ros::Subscriber subgroundTruth;
    ros::Publisher pubGT;
    ros::Subscriber subImuMsg;
    ros::Publisher pubImuMsg;

    //构造函数 初始化
    LivoxHorizon()
    {
        // 订阅horizon点云格式，发布为pcl，rosmsg点云格式
        frame_id = "horizon_frame";
        sub_livox_horizon = nodeHandler.subscribe<livox_ros_driver::CustomMsg>(horizonCloudTopic, 100, &LivoxHorizon::horizonCloudHandler, this, ros::TransportHints().tcpNoDelay());
        pub_ros_points = nodeHandler.advertise<sensor_msgs::PointCloud2>("/points_raw", 100);

        if(use_ros_time)
        {
            // 订阅bag中的真值，使用ros时间，重新发布 /groundTruth
            subgroundTruth = nodeHandler.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/qingqing/pose", 100, &LivoxHorizon::groundTruthHandler, this, ros::TransportHints().tcpNoDelay());
            pubGT = nodeHandler.advertise<geometry_msgs::PoseStamped>("/groundTruth", 100); 
            // 我们需要订阅bag中的IMU数据，使用ros::Time::now时间戳，重新发布imuTopic
            subImuMsg = nodeHandler.subscribe<sensor_msgs::Imu>("/livox/imu", imuRate, &LivoxHorizon::imuMsgHandler, this, ros::TransportHints().tcpNoDelay());
            pubImuMsg = nodeHandler.advertise<sensor_msgs::Imu>(imuTopic, imuRate);
        }
        else
        {
            ROS_INFO(">>>>>>>>>Don't use ros time : false");
        }

    }

    void horizonCloudHandler(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in)
    {
        pcl::PointCloud<Point> pcl_in;
        // 遍历每一个点
        for(unsigned int i=0; i<livox_msg_in->point_num; ++i)
        {
            Point pt;
            /* 
            // the time of first point timebase (ns) = header.stamp (s)
            cout << "the time of first point--timebase:" << livox_msg_in->timebase << endl;
            cout << "stamp sec:" << livox_msg_in->header.stamp.toSec() << endl;
            // offset_time (ns) time relative to the base time
            cout << "offset_time:" << livox_msg_in->points[i].offset_time << "--line:" << int(livox_msg_in->points[i].line) << endl;
             */
            // offset_time 单位是ns， 需要转为s /1000000000.0 10^9 变成浮点数
            pt.x = livox_msg_in->points[i].x;
            pt.y = livox_msg_in->points[i].y;
            pt.z = livox_msg_in->points[i].z;
            pt.intensity = livox_msg_in->points[i].reflectivity;
            
            pt.ring = livox_msg_in->points[i].line;
            pt.time = livox_msg_in->points[i].offset_time/1000000000.0;  // 浮点数 单位s
            // cout << "offset_time:" << pt.time << "--line:" << pt.ring << endl;
            pcl_in.push_back(pt);
        }

        //ros::Time timestamp(livox_msg_in->header.stamp);
        ros::Time timestamp;
        if(use_ros_time)
        {
            timestamp = ros::Time::now();
        }
        else{
            timestamp = livox_msg_in->header.stamp;
        }
        //cout << "ros::Time::now() sec: " << timestamp.sec <<" nsec: " << timestamp.nsec << endl;
        //cout << "stamp sec: " << livox_msg_in->header.stamp.sec <<" nsec: " << livox_msg_in->header.stamp.nsec << endl;

        sensor_msgs::PointCloud2 pcl_rosmsg;
        pcl::toROSMsg(pcl_in, pcl_rosmsg);
        pcl_rosmsg.header.stamp = timestamp;
        pcl_rosmsg.header.frame_id = frame_id;

        pub_ros_points.publish(pcl_rosmsg);
    }

    void groundTruthHandler(const geometry_msgs::PoseStamped::ConstPtr& gtMsg)
    {
        geometry_msgs::PoseStamped ps = *gtMsg;
        ros::Time stamp= ros::Time::now();
        ps.header.stamp = stamp;
        pubGT.publish(ps);
    }

    void imuMsgHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisMsg = *imuMsg;
        ros::Time stamp = ros::Time::now();
        thisMsg.header.stamp = stamp;
        pubImuMsg.publish(thisMsg);
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hrizon_format_converter");
    
    LivoxHorizon LH;

    ROS_INFO("\033[1;32m----> Livox Horizon Started.\033[0m");
    
    ros::spin();

    return 0;
}
