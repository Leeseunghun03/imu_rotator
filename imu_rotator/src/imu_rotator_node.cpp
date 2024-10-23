#include "../include/imu_rotator/imu_rotator_node.hpp"

#define PI_M (3.14159265358)

imu_rotator_node::imu_rotator_node(int argc, char **argv)
{
    ros::init(argc, argv, "imu_rotator_node");
    ros::NodeHandle nh;

    sub_imu = nh.subscribe("/imu/data_raw", 200000, &imu_rotator_node::imu_cbk, this);
    sub_rot = nh.subscribe("/lidar_rotation", 200000, &imu_rotator_node::lidar_rotation_cbk, this);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/rotated_imu", 100000);

    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<int>("mapping/rpm", rpm, 5);

    for (int i = 0; i < 3; ++i)
    {
        extrinT_arr[i] = extrinT[i];
    }

    ros::Rate rate(5000);
    bool status = ros::ok();

    while (status)
    {
        ros::spinOnce();
        if (sync_packages(Sensors))
        {
            run();
        }
        status = ros::ok();
        rate.sleep();
    }
}

imu_rotator_node::~imu_rotator_node()
{
}

void imu_rotator_node::run()
{
    sensor_msgs::Imu imu_origin = *(Sensors.imu.back());
    psi = -Sensors.rotation.back()->rotation;

    double psi_radian = psi * PI_M / 180.0;
    double theta_radian = theta * PI_M / 180.0;
    double fai_radian = fai * PI_M / 180.0;

    rad_sec = -rpm * PI_M / 30;

    Eigen::Vector3d original_angular_vel(imu_origin.angular_velocity.x,
                                         imu_origin.angular_velocity.y,
                                         imu_origin.angular_velocity.z);

    Eigen::Matrix3d R_Matrix;
    Eigen::Vector3d transformed_angular_vel;

    if (psi == 1.0)
    {
        R_Matrix = make_R_matrix(0.0, theta_radian, 0.0);
        transformed_angular_vel = R_Matrix * original_angular_vel;
    }
    else
    {
        R_Matrix = make_R_matrix(psi_radian, theta_radian, fai_radian);
        Eigen::Vector3d add_angular_vel(0.0, 0.0, rad_sec);
        add_angular_vel = R_Matrix * add_angular_vel;
        transformed_angular_vel = R_Matrix * original_angular_vel + add_angular_vel;
    }

    Eigen::Vector3d original_linear_acc(imu_origin.linear_acceleration.x,
                                        imu_origin.linear_acceleration.y,
                                        imu_origin.linear_acceleration.z);

    Eigen::Vector3d transformed_linear_acc = R_Matrix * original_linear_acc;

    if (!(psi == 1.0))
    {
        Eigen::Vector3d transformed_distance(extrinT_arr[0], extrinT_arr[1], extrinT_arr[2]);

        double centripetal_acc_x = pow(original_angular_vel[0], 2) * transformed_distance[0];
        double centripetal_acc_y = pow(original_angular_vel[1], 2) * transformed_distance[1];
        double centripetal_acc_z = pow(original_angular_vel[2], 2) * transformed_distance[2];

        Eigen::Vector3d centripetal_linear_acc(centripetal_acc_x, centripetal_acc_y, centripetal_acc_z);
        centripetal_linear_acc = R_Matrix * centripetal_linear_acc;

        transformed_linear_acc = transformed_linear_acc - centripetal_linear_acc;
    }

    sensor_msgs::Imu rotated_imu_msg;

    rotated_imu_msg.linear_acceleration.x = transformed_linear_acc[0];
    rotated_imu_msg.linear_acceleration.y = transformed_linear_acc[1];
    rotated_imu_msg.linear_acceleration.z = transformed_linear_acc[2];
    rotated_imu_msg.angular_velocity.x = transformed_angular_vel[0];
    rotated_imu_msg.angular_velocity.y = transformed_angular_vel[1];
    rotated_imu_msg.angular_velocity.z = transformed_angular_vel[2];
    rotated_imu_msg.header.stamp = imu_origin.header.stamp;

    pub_imu.publish(rotated_imu_msg);
}

Eigen::Matrix3d imu_rotator_node::make_R_matrix(double yaw_radian, double pitch_radian, double roll_radian)
{
    Eigen::Matrix3d yawMatrix;
    yawMatrix << cos(yaw_radian), sin(yaw_radian), 0,
        -sin(yaw_radian), cos(yaw_radian), 0,
        0, 0, 1;

    Eigen::Matrix3d pitchMatrix;
    pitchMatrix << cos(pitch_radian), 0, -sin(pitch_radian),
        0, 1, 0,
        sin(pitch_radian), 0, cos(pitch_radian);

    Eigen::Matrix3d Result_Matrix = pitchMatrix * yawMatrix;

    return Result_Matrix;
}

void imu_rotator_node::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec());

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    imu_buff = msg;
    mtx_buffer.unlock();
}

void imu_rotator_node::lidar_rotation_cbk(const slam_msgs::lidar_rotation::ConstPtr &msg_in)
{
    slam_msgs::lidar_rotation::Ptr msg(new slam_msgs::lidar_rotation(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec());

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_lidar_rot)
    {
        ROS_WARN("lidar rotation loop back, clear buffer");
        rotation_buffer.clear();
    }

    last_timestamp_lidar_rot = timestamp;

    rotation_buffer.push_back(msg);
    rotation_buff = msg;
    mtx_buffer.unlock();
}

bool imu_rotator_node::sync_packages(SensorGroup &meas)
{
    if (imu_buffer.empty() || rotation_buffer.empty())
    {
        return false;
    }

    sync_time = ros::Time::now().toSec();

    double rotation_time = rotation_buffer.front()->header.stamp.toSec();
    meas.rotation.clear();
    while ((!rotation_buffer.empty()) && (rotation_time < sync_time))
    {
        rotation_time = rotation_buffer.front()->header.stamp.toSec();
        if (rotation_time > sync_time)
            break;
        meas.rotation.push_back(rotation_buffer.front());
        rotation_buffer.pop_front();
    }

    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < sync_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > sync_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    return true;
}

int main(int argc, char **argv)
{
    imu_rotator_node node(argc, argv);
    return 0;
}