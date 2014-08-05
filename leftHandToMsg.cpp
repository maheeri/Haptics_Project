#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/String.h>

#define TGT_FRAME "/l_gripper_tool_frame"
//#define TGT_FRAME "/l_wrist_roll_link"

int main(int argc, char** argv){
  ros::init(argc, argv, "lh_tf_listener");

  ros::NodeHandle node;

  ros::Publisher lh_pose = node.advertise<geometry_msgs::Pose>("lHandPose", 10);

  tf::TransformListener listener;

    listener.waitForTransform("/torso_lift_link", TGT_FRAME,ros::Time(0),ros::Duration(900.0));  

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/torso_lift_link", TGT_FRAME,ros::Time(0),ros::Duration(3.0));  
      listener.lookupTransform("/torso_lift_link", TGT_FRAME,  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::Pose pose_msg;
    geometry_msgs::Point point_msg;
    geometry_msgs::Quaternion or_msg;
    // BELOW IS WHAT YOU WANT
    //point_msg.x = transform.getOrigin().x();
    //point_msg.y = transform.getOrigin().y();
    //point_msg.z = transform.getOrigin().z();
    //or_msg.x = transform.getRotation().x();
    //or_msg.y = transform.getRotation().y();
    //or_msg.z = transform.getRotation().z();
    //or_msg.w = transform.getRotation().w();
    //pose_msg.position = point_msg;
    //pose_msg.orientation = or_msg;
    poseTFToMsg(transform,pose_msg);

    lh_pose.publish(pose_msg);    

    rate.sleep();
  }
  return 0;
};
