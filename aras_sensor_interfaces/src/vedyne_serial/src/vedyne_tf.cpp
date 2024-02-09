/*
 # @ Author: Pallab Maji
 # @ Create Time: 2024-02-06 10:53:24
 # @ Modified time: 2024-02-06 10:53:28
 # @ Description: Enter description here
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class Vedyne2TFNode
{
public:
    Vedyne2TFNode()
    {
        // Replace with your IMU topic
        imu_subscriber_ = nh_.subscribe("/vehicle_dynamics/imu", 1, &Vedyne2TFNode::imuCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        // Create a TF transform message
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = imu_msg->header.stamp;
        transformStamped.header.frame_id = "vedyne_0";  // Parent frame
        transformStamped.child_frame_id = "imu_link";    // Child frame

        // Update the translation based on linear acceleration
        // Assuming constant linear acceleration, you may need to adjust based on your IMU data
        double dt = (imu_msg->header.stamp - last_imu_time_).toSec();
        last_imu_time_ = imu_msg->header.stamp;

        transformStamped.transform.translation.x += linear_acceleration_factor_ * imu_msg->linear_acceleration.x * dt * dt / 2.0;
        transformStamped.transform.translation.y += linear_acceleration_factor_ * imu_msg->linear_acceleration.y * dt * dt / 2.0;
        transformStamped.transform.translation.z += linear_acceleration_factor_ * imu_msg->linear_acceleration.z * dt * dt / 2.0;

        // Set the orientation
        transformStamped.transform.rotation = imu_msg->orientation;

        // Broadcast the TF transform
        tf_broadcaster_.sendTransform(transformStamped);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Time last_imu_time_;
    double linear_acceleration_factor_ = 0.5;  // Adjust based on your IMU data
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vedyne_tf_publisher");
    Vedyne2TFNode vedyne_to_tf_node;

    ros::spin();

    return 0;
}
