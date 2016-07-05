#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace nav_points {
class NavPoints {
public:
  NavPoints();

private:
  interactive_markers::InteractiveMarkerServer server_;
  ros::Publisher goal_pub_;
  XmlRpc::XmlRpcValue param_points_;
  std::vector<geometry_msgs::PoseStamped> points_;

  void createNavPoint(double x, double y, double theta);
  void loadPoints();
  void addMarkers();

  visualization_msgs::InteractiveMarker createMarker(geometry_msgs::PoseStamped pose);

  void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f);
};
}
