#ifndef IMU_ROTATOR_NODE_HPP
#define IMU_ROTATOR_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <slam_msgs/lidar_rotation.h>
#include <Eigen/Core>
#include <deque>
#include <mutex>
#include <vector>

class imu_rotator_node
{
public:
    imu_rotator_node(int argc, char **argv);
    ~imu_rotator_node();

private:
    std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
    std::deque<slam_msgs::lidar_rotation::Ptr> rotation_buffer;

    struct SensorGroup
    {
        std::deque<sensor_msgs::Imu::Ptr> imu;
        std::deque<slam_msgs::lidar_rotation::Ptr> rotation;
    };

    sensor_msgs::Imu::Ptr imu_buff;
    slam_msgs::lidar_rotation::Ptr rotation_buff;

    SensorGroup Sensors;

    std::mutex mtx_buffer;

    double rad_sec = 0.0;
    double last_timestamp_imu = -1.0, last_timestamp_lidar_rot = -1.0, sync_time = -1.0;
    double theta = -45.0, fai = 0.0, psi = 0.0;
    int rpm = 0;

    std::vector<double> extrinT;
    double extrinT_arr[3] = {-0.09, 0.0, 0.1};

    Eigen::Matrix3d R_Matrix;

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void lidar_rotation_cbk(const slam_msgs::lidar_rotation::ConstPtr &msg_in);
    bool sync_packages(SensorGroup &meas);
    Eigen::Matrix3d make_R_matrix(double yaw_radian, double pitch_radian, double roll_radian);
    void run();

    ros::Subscriber sub_imu;
    ros::Subscriber sub_rot;

    ros::Publisher pub_imu;
};

#endif