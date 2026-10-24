// tool_registry_node.cpp
//
// Takes the tag detections from the aruco detector (which are in the CAMERA frame) and the
// drone pose from the mission node (PX4 local NED) and figures out where each tool actually
// is in the world. Keeps a running average per tool so one bad detection doesnt throw it off.
//
// in:  /spacedrone/tool_detections  (spacedrone_interfaces/ToolDetectionArray)
//      /spacedrone/vehicle_pose     (geometry_msgs/PoseStamped, NED, orientation = body FRD -> NED)
// out: /spacedrone/tool_map         (spacedrone_interfaces/ToolMap, transient local so late joiners get it)
// srv: ~/clear                      (std_srvs/Trigger) wipe the map
//
// Wrote this one in C++ mostly to learn rclcpp, but also it runs on every detection
// so it's nice that its fast.

#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "spacedrone_interfaces/msg/tool_detection_array.hpp"
#include "spacedrone_interfaces/msg/tool_map.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using spacedrone_interfaces::msg::ToolDetectionArray;
using spacedrone_interfaces::msg::ToolMap;
using spacedrone_interfaces::msg::Tool;

// running estimate for one tool (welford's algorithm for mean/variance)
struct ToolEstimate
{
  std::string name;
  int count = 0;
  tf2::Vector3 mean{0, 0, 0};
  double m2 = 0.0;  // sum of squared distances from the mean

  void add(const tf2::Vector3 & p)
  {
    count++;
    tf2::Vector3 delta = p - mean;
    mean += delta / static_cast<double>(count);
    tf2::Vector3 delta2 = p - mean;
    m2 += delta.dot(delta2);
  }

  double spread() const
  {
    if (count < 2) {return 0.0;}
    return std::sqrt(m2 / (count - 1));
  }
};

class ToolRegistry : public rclcpp::Node
{
public:
  ToolRegistry()
  : Node("tool_registry")
  {
    // camera position on the drone in body FRD (forward, right, down) meters.
    // the gz mono_cam is basically right at the center so this is ~0 in the sim
    cam_offset_ = declare_parameter<std::vector<double>>("camera_offset_frd", {0.0, 0.0, 0.0});
    outlier_gate_ = declare_parameter<double>("outlier_gate_m", 1.5);
    min_for_gate_ = declare_parameter<int>("min_sightings_for_gate", 5);
    max_tilt_deg_ = declare_parameter<double>("max_tilt_deg", 20.0);
    max_pose_age_ = declare_parameter<double>("max_pose_age_s", 0.5);
    map_file_ = declare_parameter<std::string>("map_file", "");
    bool load_map = declare_parameter<bool>("load_existing_map", false);

    if (cam_offset_.size() != 3) {
      RCLCPP_WARN(get_logger(), "camera_offset_frd should have 3 numbers, using 0,0,0");
      cam_offset_ = {0.0, 0.0, 0.0};
    }

    if (load_map && !map_file_.empty()) {
      loadMap();
    }

    det_sub_ = create_subscription<ToolDetectionArray>(
      "/spacedrone/tool_detections", 10,
      std::bind(&ToolRegistry::detectionCallback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/spacedrone/vehicle_pose", rclcpp::SensorDataQoS(),
      std::bind(&ToolRegistry::poseCallback, this, std::placeholders::_1));

    // transient local = "latched" in ros1 terms, so the mission node gets the
    // last map even if it starts after us
    map_pub_ = create_publisher<ToolMap>("/spacedrone/tool_map", rclcpp::QoS(1).transient_local());

    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        tools_.clear();
        res->success = true;
        res->message = "cleared tool map";
        RCLCPP_INFO(get_logger(), "tool map cleared");
        publishMap();
      });

    timer_ = create_wall_timer(500ms, std::bind(&ToolRegistry::publishMap, this));
    save_timer_ = create_wall_timer(5s, std::bind(&ToolRegistry::saveMap, this));

    RCLCPP_INFO(get_logger(), "tool registry ready (map file: '%s')", map_file_.c_str());
  }

  ~ToolRegistry()
  {
    saveMap();
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_pose_ = *msg;
    // using wall time for the age check, not msg stamp. see note in detectionCallback
    last_pose_time_ = std::chrono::steady_clock::now();
    have_pose_ = true;
  }

  void detectionCallback(const ToolDetectionArray::SharedPtr msg)
  {
    if (msg->detections.empty()) {return;}

    if (!have_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "got detections but no vehicle pose yet");
      return;
    }

    // NOTE: originally tried to match the image timestamp to the closest pose in a buffer,
    // but the image stamps are sim time from gazebo and the pose stamps are from the
    // mission node clock, and when use_sim_time wasnt set everywhere it was a mess.
    // the drone moves slow during the search so just using the latest pose (if it's fresh)
    // is good enough. TODO maybe fix properly for the real drone
    double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_pose_time_).count();
    if (age > max_pose_age_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "vehicle pose is stale (%.2fs), skipping", age);
      return;
    }

    const auto & o = last_pose_.pose.orientation;
    tf2::Quaternion q_body_to_ned(o.x, o.y, o.z, o.w);
    if (q_body_to_ned.length2() < 0.5) {
      return;  // all zeros = not initialized yet
    }
    q_body_to_ned.normalize();
    tf2::Matrix3x3 R(q_body_to_ned);

    // skip if we are tilted a lot (accelerating hard), the pose/image sync is worse then
    double roll, pitch, yaw;
    R.getRPY(roll, pitch, yaw);
    double max_tilt = max_tilt_deg_ * M_PI / 180.0;
    if (std::fabs(roll) > max_tilt || std::fabs(pitch) > max_tilt) {
      return;
    }

    tf2::Vector3 drone_ned(
      last_pose_.pose.position.x, last_pose_.pose.position.y, last_pose_.pose.position.z);
    tf2::Vector3 cam_offset(cam_offset_[0], cam_offset_[1], cam_offset_[2]);

    for (const auto & det : msg->detections) {
      const auto & t = det.pose.position;

      // camera optical frame -> body FRD, for a camera pointing straight down with the
      // top of the image towards the front of the drone:
      //   optical x (image right)  = body right
      //   optical y (image down)   = body backwards
      //   optical z (out of lens)  = body down
      tf2::Vector3 tag_body(-t.y, t.x, t.z);
      // tf2::Vector3 tag_body(t.y, t.x, t.z);   // had forward flipped at first, tools ended up mirrored behind the drone

      tf2::Vector3 tag_ned = drone_ned + R * (tag_body + cam_offset);

      auto it = tools_.find(det.tag_id);
      if (it == tools_.end()) {
        ToolEstimate est;
        est.name = det.tool_name;
        est.add(tag_ned);
        tools_[det.tag_id] = est;
        RCLCPP_INFO(get_logger(), "registered %s (id %d) at N %.2f E %.2f",
          det.tool_name.c_str(), det.tag_id, tag_ned.x(), tag_ned.y());
        continue;
      }

      ToolEstimate & est = it->second;
      double dist = (tag_ned - est.mean).length();
      if (est.count >= min_for_gate_ && dist > outlier_gate_) {
        RCLCPP_DEBUG(get_logger(), "rejected sighting of %d, %.2fm from mean", det.tag_id, dist);
        rejected_++;
        continue;
      }
      est.add(tag_ned);
      // tools_[det.tag_id].mean = tag_ned;   // first version, just overwrote it every time. very jumpy
    }
  }

  void publishMap()
  {
    ToolMap map;
    map.header.stamp = now();
    map.header.frame_id = "local_ned";
    for (const auto & kv : tools_) {
      Tool t;
      t.tag_id = kv.first;
      t.tool_name = kv.second.name;
      t.position.x = kv.second.mean.x();
      t.position.y = kv.second.mean.y();
      t.position.z = kv.second.mean.z();
      t.num_sightings = kv.second.count;
      t.spread_m = static_cast<float>(kv.second.spread());
      map.tools.push_back(t);
    }
    map_pub_->publish(map);
  }

  // save the map to a yaml file so we can come back for the tools on a later flight
  void saveMap()
  {
    if (map_file_.empty() || tools_.empty()) {return;}

    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "tools" << YAML::Value << YAML::BeginSeq;
    for (const auto & kv : tools_) {
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << kv.first;
      out << YAML::Key << "name" << YAML::Value << kv.second.name;
      out << YAML::Key << "north" << YAML::Value << kv.second.mean.x();
      out << YAML::Key << "east" << YAML::Value << kv.second.mean.y();
      out << YAML::Key << "down" << YAML::Value << kv.second.mean.z();
      out << YAML::Key << "sightings" << YAML::Value << kv.second.count;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq << YAML::EndMap;

    std::ofstream f(map_file_);
    if (!f) {
      RCLCPP_WARN_ONCE(get_logger(), "couldnt open %s for writing", map_file_.c_str());
      return;
    }
    f << out.c_str() << std::endl;
  }

  void loadMap()
  {
    try {
      YAML::Node root = YAML::LoadFile(map_file_);
      for (const auto & n : root["tools"]) {
        ToolEstimate est;
        est.name = n["name"].as<std::string>();
        est.mean = tf2::Vector3(n["north"].as<double>(), n["east"].as<double>(), n["down"].as<double>());
        est.count = n["sightings"].as<int>();
        tools_[n["id"].as<int>()] = est;
      }
      RCLCPP_INFO(get_logger(), "loaded %zu tools from %s", tools_.size(), map_file_.c_str());
    } catch (const std::exception & e) {
      // first run, file doesnt exist yet. thats fine
      RCLCPP_WARN(get_logger(), "couldnt load map file (%s), starting empty", e.what());
    }
  }

  // params
  std::vector<double> cam_offset_;
  double outlier_gate_;
  int min_for_gate_;
  double max_tilt_deg_;
  double max_pose_age_;
  std::string map_file_;

  // state
  std::map<int, ToolEstimate> tools_;
  // std::map<int, geometry_msgs::msg::Point> tools_;
  geometry_msgs::msg::PoseStamped last_pose_;
  std::chrono::steady_clock::time_point last_pose_time_;
  bool have_pose_ = false;
  int rejected_ = 0;

  rclcpp::Subscription<ToolDetectionArray>::SharedPtr det_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<ToolMap>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolRegistry>());
  rclcpp::shutdown();
  return 0;
}
