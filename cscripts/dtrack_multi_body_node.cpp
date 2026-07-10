#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
std::string trim(const std::string& input)
{
  const std::string whitespace = " \t\r\n";
  const std::size_t first = input.find_first_not_of(whitespace);
  if (first == std::string::npos)
  {
    return "";
  }

  const std::size_t last = input.find_last_not_of(whitespace);
  return input.substr(first, last - first + 1);
}

bool startsWith(const std::string& input, const std::string& prefix)
{
  return input.compare(0, prefix.size(), prefix) == 0;
}
}  // namespace

class DTrackMultiBodyNode
{
  struct BodyState
  {
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    nav_msgs::Path path;
  };

  struct ParsedPose
  {
    int body_id;
    geometry_msgs::PoseStamped pose;
  };

  using RotationValues = std::array<double, 9>;

public:
  DTrackMultiBodyNode() : private_nh_("~")
  {
    private_nh_.param("udp_port", udp_port_, 5000);
    private_nh_.param<std::string>("frame_id", frame_id_, "odom_mocap");
    private_nh_.param<std::string>("parent_frame", parent_frame_, "odom");
    private_nh_.param("max_path_length", max_path_length_, 500);
    if (max_path_length_ < 1)
    {
      ROS_WARN("Invalid max_path_length %d, using 1 instead", max_path_length_);
      max_path_length_ = 1;
    }

    openSocket();
    ROS_INFO("DTrack multi-body UDP listener started on port %d", udp_port_);
  }

  ~DTrackMultiBodyNode()
  {
    if (socket_fd_ >= 0)
    {
      close(socket_fd_);
    }
  }

  void run()
  {
    std::vector<char> buffer(65535);

    while (ros::ok())
    {
      sockaddr_in sender_addr;
      socklen_t sender_len = sizeof(sender_addr);
      const ssize_t received = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                        reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);

      if (received < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
          publishStaticTransform();
          ros::spinOnce();
          continue;
        }

        ROS_WARN_THROTTLE(1.0, "Failed to receive DTrack UDP packet: %s", strerror(errno));
        publishStaticTransform();
        ros::spinOnce();
        continue;
      }

      const std::string packet(buffer.data(), static_cast<std::size_t>(received));
      const std::vector<ParsedPose> poses = parsePacket(packet);

      for (const auto& pose : poses)
      {
        publishPose(pose.body_id, pose.pose);
      }

      publishStaticTransform();
      ros::spinOnce();
    }
  }

private:
  void openSocket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
      ROS_FATAL("Failed to create UDP socket: %s", strerror(errno));
      ros::shutdown();
      return;
    }

    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
      ROS_WARN("Failed to set SO_REUSEADDR: %s", strerror(errno));
    }

    timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
      ROS_WARN("Failed to set UDP receive timeout: %s", strerror(errno));
    }

    sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(static_cast<uint16_t>(udp_port_));

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0)
    {
      ROS_FATAL("Failed to bind UDP socket on port %d: %s", udp_port_, strerror(errno));
      ros::shutdown();
    }
  }

  std::vector<ParsedPose> parsePacket(const std::string& packet)
  {
    std::vector<ParsedPose> poses;
    std::istringstream stream(packet);
    std::string line;

    while (std::getline(stream, line))
    {
      parseLine(trim(line), poses);
    }

    return poses;
  }

  void parseLine(const std::string& line, std::vector<ParsedPose>& poses)
  {
    if (!startsWith(line, "6d "))
    {
      return;
    }

    std::istringstream header_stream(line);
    std::string tag;
    int expected_body_count = 0;
    header_stream >> tag >> expected_body_count;

    if (expected_body_count <= 0)
    {
      return;
    }

    const std::regex group_regex("\\[([^\\]]*)\\]");
    std::vector<std::string> groups;
    for (std::sregex_iterator it(line.begin(), line.end(), group_regex), end; it != end; ++it)
    {
      groups.push_back((*it)[1].str());
    }

    const int parsed_body_count = static_cast<int>(groups.size() / 3);
    if (parsed_body_count < expected_body_count)
    {
      ROS_WARN_THROTTLE(1.0, "DTrack 6d line expected %d bodies but only %d complete groups were found",
                        expected_body_count, parsed_body_count);
    }

    const int body_count = std::min(expected_body_count, parsed_body_count);
    for (int body_index = 0; body_index < body_count; ++body_index)
    {
      const std::string& body_group = groups[body_index * 3];
      const std::string& pose_group = groups[body_index * 3 + 1];
      const std::string& rotation_group = groups[body_index * 3 + 2];

      int body_id = 0;
      double quality = 0.0;
      std::istringstream body_stream(body_group);
      if (!(body_stream >> body_id >> quality))
      {
        ROS_WARN_THROTTLE(1.0, "Skipping DTrack body with invalid id/quality group: [%s]",
                          body_group.c_str());
        continue;
      }

      double raw_x = 0.0;
      double raw_y = 0.0;
      double raw_z = 0.0;
      double raw_rx = 0.0;
      double raw_ry = 0.0;
      double raw_rz = 0.0;
      std::istringstream pose_stream(pose_group);
      if (!(pose_stream >> raw_x >> raw_y >> raw_z >> raw_rx >> raw_ry >> raw_rz))
      {
        ROS_WARN_THROTTLE(1.0, "Skipping DTrack body %d with invalid pose group: [%s]",
                          body_id, pose_group.c_str());
        continue;
      }

      RotationValues rotation_values;
      if (!parseRotationGroup(rotation_group, rotation_values))
      {
        ROS_WARN_THROTTLE(1.0, "Skipping DTrack body %d with invalid rotation matrix group: [%s]",
                          body_id, rotation_group.c_str());
        continue;
      }

      ParsedPose parsed_pose;
      parsed_pose.body_id = body_id;
      parsed_pose.pose = makePose(body_id, raw_x, raw_y, raw_z, rotation_values);
      poses.push_back(parsed_pose);
    }
  }

  bool parseRotationGroup(const std::string& rotation_group, RotationValues& values) const
  {
    std::istringstream rotation_stream(rotation_group);
    for (double& value : values)
    {
      if (!(rotation_stream >> value))
      {
        return false;
      }
    }

    return true;
  }

  geometry_msgs::PoseStamped makePose(int body_id, double raw_x, double raw_y, double raw_z,
                                      const RotationValues& rotation_values)
  {
    const double x = raw_x / 1000.0;
    const double y = raw_y / 1000.0;
    const double z = raw_z / 1000.0;

    if (!origin_initialized_)
    {
      origin_x_ = x;
      origin_y_ = y;
      origin_z_ = z;
      origin_initialized_ = true;
      ROS_INFO("DTrack global origin initialized from body %d at %.6f %.6f %.6f",
               body_id, origin_x_, origin_y_, origin_z_);
    }

    tf2::Matrix3x3 rotation;
    rotation.setValue(rotation_values[0], rotation_values[3], rotation_values[6],
                      rotation_values[1], rotation_values[4], rotation_values[7],
                      rotation_values[2], rotation_values[5], rotation_values[8]);

    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x - origin_x_;
    pose.pose.position.y = y - origin_y_;
    pose.pose.position.z = z - origin_z_;
    pose.pose.orientation = tf2::toMsg(quaternion);

    return pose;
  }

  void publishPose(int body_id, const geometry_msgs::PoseStamped& pose)
  {
    BodyState& body_state = getBodyState(body_id);
    body_state.pose_pub.publish(pose);

    body_state.path.header.stamp = ros::Time::now();
    body_state.path.poses.push_back(pose);
    while (static_cast<int>(body_state.path.poses.size()) > max_path_length_)
    {
      body_state.path.poses.erase(body_state.path.poses.begin());
    }

    body_state.path_pub.publish(body_state.path);
  }

  BodyState& getBodyState(int body_id)
  {
    const auto existing = bodies_.find(body_id);
    if (existing != bodies_.end())
    {
      return existing->second;
    }

    BodyState body_state;
    const std::string body_namespace = "/dtrack/body_" + std::to_string(body_id);
    body_state.pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(body_namespace + "/pose", 10);
    body_state.path_pub = nh_.advertise<nav_msgs::Path>(body_namespace + "/path", 10);
    body_state.path.header.frame_id = frame_id_;

    ROS_INFO("Created DTrack publishers for body %d", body_id);
    return bodies_.emplace(body_id, body_state).first->second;
  }

  void publishStaticTransform()
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(transform);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  int udp_port_ = 5000;
  int max_path_length_ = 500;
  int socket_fd_ = -1;
  std::string frame_id_ = "odom_mocap";
  std::string parent_frame_ = "odom";

  bool origin_initialized_ = false;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_z_ = 0.0;

  std::unordered_map<int, BodyState> bodies_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dtrack_multi_body_node");

  DTrackMultiBodyNode node;
  node.run();

  return 0;
}
