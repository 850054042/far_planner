#include <goalpoint_tool.hpp>

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace goalpoint_rviz_plugin
{
GoalpointTool::GoalpointTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "goalpoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
  
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);

  height_mode_property_ = new rviz_common::properties::EnumProperty(
    "Target height mode", "Current vehicle",
    "Choose how the Z coordinate of the clicked cross-floor goal is computed.",
    getPropertyContainer());
  height_mode_property_->addOption("Current vehicle", 0);
  height_mode_property_->addOption("Floor ID", 1);
  height_mode_property_->addOption("Explicit Z", 2);

  floor_id_property_ = new rviz_common::properties::IntProperty(
    "Target floor ID", 0, "Zero-based target floor index.", getPropertyContainer());
  floor_origin_z_property_ = new rviz_common::properties::FloatProperty(
    "Floor 0 goal Z", 0.75f,
    "Robot/goal Z on floor 0, normally vehicleHeight above the floor surface.",
    getPropertyContainer());
  floor_height_property_ = new rviz_common::properties::FloatProperty(
    "Floor height", 2.0f, "Vertical distance between adjacent floor goal planes.",
    getPropertyContainer());
  floor_height_property_->setMin(0.01f);
  explicit_z_property_ = new rviz_common::properties::FloatProperty(
    "Explicit target Z", 0.75f, "Exact map-frame Z used in Explicit Z mode.",
    getPropertyContainer());
}

GoalpointTool::~GoalpointTool() = default;

void GoalpointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("Goalpoint");
  updateTopic();
  vehicle_z = 0;
}

void GoalpointTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = raw_node->template create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5 ,std::bind(&GoalpointTool::odomHandler,this,std::placeholders::_1));
  
  pub_ = raw_node->template create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", qos_profile_);
  pub_joy_ = raw_node->template create_publisher<sensor_msgs::msg::Joy>("/joy", qos_profile_);
  clock_ = raw_node->get_clock();
}

void GoalpointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void GoalpointTool::onPoseSet(double x, double y, double theta)
{
  (void)theta;
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = clock_->now();
  joy.header.frame_id = "goalpoint_tool";
  pub_joy_->publish(joy);

  geometry_msgs::msg::PointStamped goalpoint;
  goalpoint.header.frame_id = "map";
  goalpoint.header.stamp = joy.header.stamp;
  goalpoint.point.x = x;
  goalpoint.point.y = y;
  switch (height_mode_property_->getOptionInt()) {
    case 1:
      goalpoint.point.z = floor_origin_z_property_->getFloat() +
        floor_id_property_->getInt() * floor_height_property_->getFloat();
      break;
    case 2:
      goalpoint.point.z = explicit_z_property_->getFloat();
      break;
    default:
      goalpoint.point.z = vehicle_z;
      break;
  }

  RVIZ_COMMON_LOG_INFO_STREAM(
    "Publishing goal (" << goalpoint.point.x << ", " << goalpoint.point.y <<
    ", " << goalpoint.point.z << ") in map frame");

  pub_->publish(goalpoint);
  usleep(10000);
  pub_->publish(goalpoint);
}
}

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(goalpoint_rviz_plugin::GoalpointTool, rviz_common::Tool)
