#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <regex>
#include <sstream>
#include <stdexcept>
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
    if (!private_nh_.getParam("udp_port", udp_port_))
    {
      throw std::runtime_error("Missing required parameter: ~udp_port");
    }
    private_nh_.param<std::string>("frame_id", frame_id_, "odom_mocap"); // config
    private_nh_.param<std::string>("parent_frame", parent_frame_, "odom"); // config
    if (!private_nh_.getParam("max_path_length", max_path_length_))
    {
      throw std::runtime_error("Missing required parameter: ~max_path_length");
    }
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

      if (received < 0) // checken mit continue
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
          ros::spinOnce();
          continue;
        }

        ROS_WARN_THROTTLE(1.0, "Failed to receive DTrack UDP packet: %s", strerror(errno));
        ros::spinOnce();
        continue;
      }

      const std::string packet(buffer.data(), static_cast<std::size_t>(received));
      const std::vector<ParsedPose> poses = parsePacket(packet);

      for (const auto& pose : poses)
      {
        publishPose(pose.body_id, pose.pose);
      }

      if (!poses.empty())
      {
        publishStaticTransform(poses.front().pose.header.stamp);
      }
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
    ros::Time measurement_stamp;
    bool has_measurement_stamp = false;

    // DTrack's "ts" value is the UTC time of the camera exposure in seconds
    // since midnight. Determine it before parsing poses because all data in the
    // datagram belong to the same measurement frame.
    while (std::getline(stream, line))
    {
      const std::string trimmed_line = trim(line);
      if (startsWith(trimmed_line, "ts "))
      {
        has_measurement_stamp = parseTimestamp(trimmed_line, measurement_stamp);
        break;
      }
    }

    if (!has_measurement_stamp)
    {
      ROS_WARN_THROTTLE(1.0, "Skipping DTrack packet without a valid ts timestamp");
      return poses;
    }

    stream.clear();
    stream.seekg(0);
    while (std::getline(stream, line))
    {
      parseLine(trim(line), measurement_stamp, poses);
    }

    return poses;
  }

  bool parseTimestamp(const std::string& line, ros::Time& stamp) const
  {
    std::istringstream timestamp_stream(line);
    std::string tag;
    double seconds_since_midnight = 0.0;
    if (!(timestamp_stream >> tag >> seconds_since_midnight) || tag != "ts" ||
        !std::isfinite(seconds_since_midnight) || seconds_since_midnight < 0.0 ||
        seconds_since_midnight >= 86400.0)
    {
      return false;
    }

    // The DTrack protocol contains no date. Infer the UTC date from the local
    // wall clock (which is NTP-synchronised with the controller) and select the
    // nearest day, also covering packets received around UTC midnight.
    constexpr int64_t nanoseconds_per_second = 1000000000LL;
    constexpr int64_t seconds_per_day = 86400LL;
    const int64_t time_of_day_ns =
        static_cast<int64_t>(std::llround(seconds_since_midnight * nanoseconds_per_second));
    const ros::WallTime wall_now = ros::WallTime::now();
    const int64_t wall_now_ns = static_cast<int64_t>(wall_now.sec) * nanoseconds_per_second +
                                static_cast<int64_t>(wall_now.nsec);
    const int64_t utc_day_start_sec =
        (static_cast<int64_t>(wall_now.sec) / seconds_per_day) * seconds_per_day;

    const int64_t current_day_stamp_ns =
        utc_day_start_sec * nanoseconds_per_second + time_of_day_ns;
    int64_t best_stamp_ns = current_day_stamp_ns;
    int64_t best_distance_ns = std::llabs(best_stamp_ns - wall_now_ns);
    for (const int day_offset : {-1, 1})
    {
      const int64_t candidate_ns = current_day_stamp_ns +
                                   static_cast<int64_t>(day_offset) * seconds_per_day *
                                       nanoseconds_per_second;
      const int64_t distance_ns = std::llabs(candidate_ns - wall_now_ns);
      if (distance_ns < best_distance_ns)
      {
        best_stamp_ns = candidate_ns;
        best_distance_ns = distance_ns;
      }
    }

    stamp = ros::Time(static_cast<uint32_t>(best_stamp_ns / nanoseconds_per_second),
                      static_cast<uint32_t>(best_stamp_ns % nanoseconds_per_second));
    return true;
  }

  void parseLine(const std::string& line, const ros::Time& measurement_stamp,
                 std::vector<ParsedPose>& poses)
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
      parsed_pose.pose = makePose(raw_x, raw_y, raw_z, rotation_values, measurement_stamp);
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

  geometry_msgs::PoseStamped makePose(double raw_x, double raw_y, double raw_z,
                                      const RotationValues& rotation_values,
                                      const ros::Time& measurement_stamp) const
  {
    const double x = raw_x / 1000.0;
    const double y = raw_y / 1000.0;
    const double z = raw_z / 1000.0;

    tf2::Matrix3x3 rotation;
    rotation.setValue(rotation_values[0], rotation_values[3], rotation_values[6],
                      rotation_values[1], rotation_values[4], rotation_values[7],
                      rotation_values[2], rotation_values[5], rotation_values[8]);

    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = measurement_stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf2::toMsg(quaternion);

    return pose;
  }

  void publishPose(int body_id, const geometry_msgs::PoseStamped& pose)
  {
    BodyState& body_state = getBodyState(body_id);
    body_state.pose_pub.publish(pose);

    body_state.path.header.stamp = pose.header.stamp;
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
    const int topic_body_id = body_id + 1;
    const std::string body_namespace = "/dtrack/body_" + std::to_string(topic_body_id);
    body_state.pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(body_namespace + "/pose", 10);
    body_state.path_pub = nh_.advertise<nav_msgs::Path>(body_namespace + "/path", 10);
    body_state.path.header.frame_id = frame_id_;

    ROS_INFO("Created DTrack publishers for raw body %d under body_%d", body_id, topic_body_id);
    return bodies_.emplace(body_id, body_state).first->second;
  }

  void publishStaticTransform(const ros::Time& measurement_stamp)
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = measurement_stamp;
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

  int udp_port_ = 0;
  int max_path_length_ = 0;
  int socket_fd_ = -1;
  std::string frame_id_ = "odom_mocap";
  std::string parent_frame_ = "odom";

  std::unordered_map<int, BodyState> bodies_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dtrack_multi_body_node");

  try
  {
    DTrackMultiBodyNode node;
    node.run();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("%s", error.what());
    return 1;
  }

  return 0;
}
