#include "nav_points/nav_points.h"

#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <XmlRpcException.h>

namespace nav_points {

NavPoints::NavPoints() : server_("nav_points") {
  ros::NodeHandle private_nh("~");

  std::string goal_topic;

  // this is where we will publish our goals to
  private_nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");

  goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 10, false);
  goal_id_pub_ = private_nh.advertise<std_msgs::Int32>("goal_id", 10, true);
  goal_sub_ = private_nh.subscribe("goal_number", 10, &NavPoints::goalCallback, this);

  // iterate through all points loaded
  // on parameter server and add markers
  private_nh.getParam("nav_points", param_points_);

  ROS_ASSERT(param_points_.getType() == XmlRpc::XmlRpcValue::TypeArray);

  ROS_INFO("size %d", param_points_.size());

  loadPoints();

  addMarkers();

  ros::spin();
}

void NavPoints::onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f)
{
  if (f->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose = f->pose;
    goal_pub_.publish(goal);

    int id;
    sscanf(f->marker_name.c_str(), "loc:%i", &id);
    std_msgs::Int32 id_msg;
    id_msg.data = id;
    goal_id_pub_.publish(id_msg);
  }
}

void NavPoints::addMarkers()
{
  int i=0;
  for (geometry_msgs::PoseStamped pose: points_)
  {
    i++;
    server_.insert(createMarker(pose, std::string("loc:") + std::to_string(i)), boost::bind(&NavPoints::onClick, this, _1));
  }

  server_.applyChanges();
}

visualization_msgs::InteractiveMarker NavPoints::createMarker(geometry_msgs::PoseStamped pose, std::string visible_description)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = pose.header.frame_id;
  int_marker.pose = pose.pose;
  int_marker.pose.position.z += 0.01; //a little off the ground.
  int_marker.scale = 1;
  char buff[20];
  snprintf(buff, 20, "pt_%2.2f_%2.2f", pose.pose.position.x, pose.pose.position.y);
  int_marker.name = buff;
  int_marker.description = visible_description;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  visualization_msgs::Marker shape;
  shape.type = visualization_msgs::Marker::ARROW;
  shape.scale.x = 0.9;
  shape.scale.z = 0.01;
  shape.scale.y = 0.3;
  shape.color.r=0;
  shape.color.g=0.5;
  shape.color.b=0.25;
  shape.color.a=1;

  control.markers.push_back(shape);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  return int_marker;
}

void NavPoints::goalCallback(const std_msgs::Int32& msg)
{
  geometry_msgs::PoseStamped goal = points_.at(msg.data);
  goal_pub_.publish(goal);

  goal_id_pub_.publish(msg);
}

void NavPoints::loadPoints() {
  for (int i = 0; i < param_points_.size(); i++) {
    if (param_points_[i].hasMember("x")) {
      double x = 0, y = 0, theta = 0;

      if (param_points_[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        x = static_cast<int>(param_points_[i]["x"]);
      } else if (param_points_[i]["x"].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble) {
        x = static_cast<double>(param_points_[i]["x"]);
      } else
      {
        continue;
      }

      if (param_points_[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        y = static_cast<int>(param_points_[i]["y"]);
      } else if (param_points_[i]["y"].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble) {
        y = static_cast<double>(param_points_[i]["y"]);
      } else
      {
        continue;
      }


      if (param_points_[i]["theta"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        theta = static_cast<int>(param_points_[i]["theta"]);
      } else if (param_points_[i]["theta"].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble) {
        theta = static_cast<double>(param_points_[i]["theta"]);
      } else
      {
        continue;
      }

      ROS_INFO("adding point (%f,%f,%f)", x, y, theta);
      tf::Quaternion orientation = tf::createQuaternionFromYaw(theta);
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.x = orientation.getX();
      pose.pose.orientation.y = orientation.getY();
      pose.pose.orientation.z = orientation.getZ();
      pose.pose.orientation.w = orientation.getW();

      points_.push_back(pose);
    } else {
      ROS_ERROR("Elements of nav_points list must be of the format '{x: 0, y: "
                "0, theta: 0}'");
    }
  }
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nav_points");
  nav_points::NavPoints pts;
}
