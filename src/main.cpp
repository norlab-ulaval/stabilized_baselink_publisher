#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stabilized_baselink_publisher_node");

	ros::NodeHandle nh;

	std::string odomFrame;
	nh.param<std::string>("odom_frame", odomFrame, "odom");

	std::string robotFrame;
	nh.param<std::string>("robot_frame", robotFrame, "base_link");

	float publishRate;
	nh.param<float>("publish_rate", publishRate, 100.0);
	ros::Rate rate(publishRate);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	tf2_ros::TransformBroadcaster tfBroadcaster;

	while(ros::ok())
	{
		try
		{
			geometry_msgs::TransformStamped robotToOdom = tfBuffer.lookupTransform(odomFrame, robotFrame, ros::Time::now(), ros::Duration(0.1));
			geometry_msgs::Quaternion q = robotToOdom.transform.rotation;
			float yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
			
			tf2::Quaternion quaternion;
			tf2::fromMsg(robotToOdom.transform.rotation, quaternion);
			quaternion.setEuler(yaw, 0, 0);

			geometry_msgs::TransformStamped robotStabilizedToOdom = robotToOdom;
			robotStabilizedToOdom.child_frame_id = robotFrame + "_stabilized";
			robotStabilizedToOdom.transform.rotation = tf2::toMsg(quaternion);
			tfBroadcaster.sendTransform(robotStabilizedToOdom);
		}
		catch(const tf2::TransformException& ex)
		{
			ROS_WARN("%s", ex.what());
		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

