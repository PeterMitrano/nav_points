#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

namespace nav_points {
class NavPoints {
public:
  NavPoints();

private:
  interactive_markers::InteractiveMarkerServer server_;
  ros::Publisher goal_pub_;
  ros::Subscriber goal_sub_;
  XmlRpc::XmlRpcValue param_points_;
  std::vector<geometry_msgs::PoseStamped> points_;

  void createNavPoint(double x, double y, double theta);
  void loadPoints();
  void addMarkers();

  /** @brief navigate to an goal via it's index in the point list.
   * order is whatever's on the parameter server
   * */
  void goalCallback(const std_msgs::Int32& msg);

  visualization_msgs::InteractiveMarker createMarker(geometry_msgs::PoseStamped pose, std::string visible_description);

  void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f);
};
}
