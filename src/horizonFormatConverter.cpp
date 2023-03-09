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
    ros::Subscriber subImu1;

    //构造函数 初始化
    LivoxHorizon()
    {
        frame_id = "horizon_frame";
        sub_livox_horizon = nodeHandler.subscribe<livox_ros_driver::CustomMsg>(horizonCloudTopic, 200, &LivoxHorizon::horizonCloudHandler, this, ros::TransportHints().tcpNoDelay());
        pub_ros_points = nodeHandler.advertise<sensor_msgs::PointCloud2>("/points_raw", 200);
        //subImu1        = nh.subscribe<sensor_msgs::Imu>("/os1_cloud_node/imu", 2000, &LivoxHorizon::imuHandler1, this, ros::TransportHints().tcpNoDelay());

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

        ros::Time timestamp(livox_msg_in->header.stamp.toSec());

        sensor_msgs::PointCloud2 pcl_rosmsg;
        pcl::toROSMsg(pcl_in, pcl_rosmsg);
        pcl_rosmsg.header.stamp = timestamp;
        pcl_rosmsg.header.frame_id = frame_id;

        pub_ros_points.publish(pcl_rosmsg);
    }

    // ----------------os1----------------------------------------
    void imuHandler1(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        return;
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        // debug IMU data
        
        cout << "----------------------OS1----------------------------" << endl;
        cout << "OS1 time:" << thisImu.header.stamp.sec << " . "<<thisImu.header.stamp.nsec << endl;
        cout << std::setprecision(6);
        cout << "OS1 IMU acc: " << endl;
        cout << "x: " << thisImu.linear_acceleration.x << 
              ", y: " << thisImu.linear_acceleration.y << 
              ", z: " << thisImu.linear_acceleration.z << endl;
        cout << "OS1 IMU gyro: " << endl;
        cout << "x: " << thisImu.angular_velocity.x << 
              ", y: " << thisImu.angular_velocity.y << 
              ", z: " << thisImu.angular_velocity.z << endl;

        // sensor_msgs::Imu thisImu1 = imuQueue.front();
        // cout << std::setprecision(6);
        // cout << "horizon time:" << thisImu1.header.stamp.toSec() << endl;
        // cout << "horizon IMU acc: " << endl;
        // cout << "x: " << thisImu1.linear_acceleration.x << 
        //       ", y: " << thisImu1.linear_acceleration.y << 
        //       ", z: " << thisImu1.linear_acceleration.z << endl;
        // cout << "horizon IMU gyro: " << endl;
        // cout << "x: " << thisImu1.angular_velocity.x << 
        //       ", y: " << thisImu1.angular_velocity.y << 
        //       ", z: " << thisImu1.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
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
