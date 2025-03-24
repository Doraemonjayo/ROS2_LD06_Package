#include <iostream>
#include <ld06/LD06.hpp>
#include <ld06/Serial.hpp>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

LD06 ld06;
Serial mySerial("/dev/ttyUSB1");
void ld06_received(LD06 *ld06_);
std::vector<float> x_array = {};
std::vector<float> y_array = {};
bool array_rock = false;
uint8_t ld06_buffer[1024];

class LD06Node : public rclcpp::Node
{
public:
  LD06Node()
      : Node("ld06_node")
  {
    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ld06", 10);

    // 10Hz (100ms) でパブリッシュ
    timer_1_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LD06Node::publish_pointcloud, this));

    timer_2_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&LD06Node::read_lidar, this));  
  }

private:
  void publish_pointcloud()
  {
    array_rock = true;
    uint32_t num_points = x_array.size();

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map"; // RVizで認識できる座標系

    cloud_msg.height = 1;  // 1行のみ（ポイントのリスト）
    cloud_msg.width = num_points;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    // フィールドの定義
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud_msg, "rgb");

    for (uint32_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      *iter_x = x_array[i];
      *iter_y = y_array[i];
      *iter_z = 0.0f;
      *iter_rgb = 0x00FFFFFF;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 with %d points", cloud_msg.width);
    publisher_->publish(cloud_msg);

    x_array.clear();
    y_array.clear();

    array_rock = false;
  }

  void read_lidar(){
    ssize_t data_len = mySerial.read(ld06_buffer, sizeof(ld06_buffer));
    for (size_t i = 0; i < data_len; i++)
    {
      LD06_calc(&ld06, ld06_buffer[i]);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
};


int main(int argc, char **argv)
{
  mySerial.begin(B230400);
  LD06_init(&ld06, ld06_received);
  if (!mySerial.isOpen())
  {
    std::cout << "cannot open \"" << mySerial.getPortName() << "\"" << std::endl;
    return -1;
  }
  x_array.clear();
  y_array.clear();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LD06Node>());
  rclcpp::shutdown();
  return 0;
}

void ld06_received(LD06 *ld06_)
{
  for (size_t i = 0; i < LD06_NUM_POINTS; i++)
  {
    float angle = ld06_->angle[i];
    float distance = ld06_->distance[i] / 1000.0f;//[mm]->[m]
    x_array.push_back(cos(-angle) * distance);
    y_array.push_back(sin(-angle) * distance);
  }
}
