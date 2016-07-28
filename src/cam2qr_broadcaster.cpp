#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

void QRposeCallback(const geometry_msgs::PoseStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
  transform.setRotation( tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "qr", "cam"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cam2qr_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/visp_auto_tracker/object_position", 1, &QRposeCallback);

  ros::spin();
  return 0;
};
