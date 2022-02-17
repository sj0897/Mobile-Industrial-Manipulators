/**
 * @file main.cpp
 * @author Nishd Kulkarni, Sparsh Jaiswal, Divyansh Agarwal (nkulkar2@umd.edu, sjaiswal@umd.edu, dagrawa1@umd.edu)
 * @brief This is our main.cpp code for the Final Project of the class ENPM809Y Fall 2021
 * @version 0.1
 * @date 2021-12-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h> //for geometry_msgs::Twist
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int id = -1;
bool marker_detected = false;
double id_x{0};
double id_y{0};

/**
 * @brief The main broadcaster function to get the aruco marker locations with respect to the camera frame
 *
 * @param msg
 */
void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
{
  ROS_INFO("Searching...");

  if (!msg->transforms.empty() && (msg->transforms[0].transform.translation.x < 3))
  { // check marker is detected broadcaster object
    marker_detected = true;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped; // broadcast the new frame to /tf Topic

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;

    id = msg->transforms[0].fiducial_id;
    br.sendTransform(transformStamped);
    ROS_INFO_STREAM("Marker found! Marker id is : " << id);
  }
}
/**
 * @brief listener function to convert the aruco marker in the frame of world from the frame of camera
 *
 * @param tfBuffer
 */
void listen(tf2_ros::Buffer &tfBuffer)
{
  // for listener
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0), ros::Duration(4.0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
                    << trans_x << ","
                    << trans_y << ","
                    << trans_z << "]");

    id_x = trans_x;
    id_y = trans_y;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
 * @brief Main function for the program
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;
  ros::Subscriber f_m;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }
  XmlRpc::XmlRpcValue my_list[4];
  // nh.getParam("my_list", my_list);
  nh.getParam("/aruco_lookup_locations/target_1", my_list[0]);
  nh.getParam("/aruco_lookup_locations/target_2", my_list[1]);
  nh.getParam("/aruco_lookup_locations/target_3", my_list[2]);
  nh.getParam("/aruco_lookup_locations/target_4", my_list[3]);

  ROS_ASSERT(my_list[0].getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list[1].getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list[2].getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(my_list[3].getType() == XmlRpc::XmlRpcValue::TypeArray);

  double loc[4][2];

  for (int32_t i = 0; i < 4; ++i)
  {
    ROS_ASSERT(my_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    loc[i][0] = static_cast<double>(my_list[i][0]);
    ROS_ASSERT(my_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    loc[i][1] = static_cast<double>(my_list[i][1]);
  }

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 1);
  geometry_msgs::Twist msg_t;
  msg_t.angular.z = 0.1;

  move_base_msgs::MoveBaseGoal explorer_goal[5];
  move_base_msgs::MoveBaseGoal follower_goal[5];

  for (int i = 0; i < 4; i++)
  {

    explorer_goal[i].target_pose.header.frame_id = "map";
    explorer_goal[i].target_pose.header.stamp = ros::Time::now();
    explorer_goal[i].target_pose.pose.position.x = my_list[i][0]; //
    explorer_goal[i].target_pose.pose.position.y = my_list[i][1]; //
    explorer_goal[i].target_pose.pose.orientation.w = 1.0;
  }
  explorer_goal[4].target_pose.header.frame_id = "map";
  explorer_goal[4].target_pose.header.stamp = ros::Time::now();
  explorer_goal[4].target_pose.pose.position.x = -4;  //
  explorer_goal[4].target_pose.pose.position.y = 2.5; //
  explorer_goal[4].target_pose.pose.orientation.w = 1.0;

  follower_goal[4].target_pose.header.frame_id = "map";
  follower_goal[4].target_pose.header.stamp = ros::Time::now();
  follower_goal[4].target_pose.pose.position.x = -4;  //
  follower_goal[4].target_pose.pose.position.y = 3.5; //
  follower_goal[4].target_pose.pose.orientation.w = 1.0;

  bool explorer_task_completed = false;
  bool follower_task_completed = false;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
  int j = 0;

  while (ros::ok())
  {
    while (!explorer_task_completed)
    {
      if (!explorer_goal_sent)
      {
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal[j]); // this should be sent only once
        explorer_goal_sent = true;
      }
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray, explorer reached goal");
        if (j >= 4)
        {
          explorer_task_completed = true;
          j = 0;
        }
        else if (j < 4)
        {
          marker_detected = false;
          f_m = nh.subscribe("/fiducial_transforms", 1000, &broadcast);
          while (!marker_detected)
          {
            msg_t.angular.z = 0.1;
            pub.publish(msg_t);

            ros::spinOnce();
            if (marker_detected)
            {
              ROS_INFO_STREAM(id);
              msg_t.angular.z = 0.0;
              pub.publish(msg_t);
              listen(tfBuffer);

              follower_goal[id].target_pose.header.frame_id = "map";
              follower_goal[id].target_pose.header.stamp = ros::Time::now();
              follower_goal[id].target_pose.pose.position.x = (id_x + loc[j][0]) / 2;
              follower_goal[id].target_pose.pose.position.y = (id_y + loc[j][1]) / 2;
              follower_goal[id].target_pose.pose.orientation.w = 1;

              marker_detected = false;
              break;
            }
          }

          j++;
          explorer_goal_sent = false;
        }
      }
    }
    while (!follower_task_completed)
    {

      if (!follower_goal_sent)
      {
        ROS_INFO("Sending goal for follower");
        follower_client.sendGoal(follower_goal[j]); // this should be sent only once
        follower_goal_sent = true;
      }
      if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray, follower reached goal");
        if (j >= 4)
        {
          follower_task_completed = true;
          j = -1;
        }
        if (j < 4)
        {
          follower_goal_sent = false;
          j++;
        }
      }
    }

    loop_rate.sleep();
    ROS_INFO("Search and Rescue ended successfully");
    ros::shutdown();
  }
}
