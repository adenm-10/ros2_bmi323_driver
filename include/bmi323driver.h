#ifndef BMI323DRIVER_H
#define BMI323DRIVER_H

#include "mpu9250sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class BMI323Driver : public rclcpp::Node {
 public:
  BMI323Driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<MPU9250Sensor> mpu9250_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleInput();
  void declareParameters();
  void calculateOrientation(sensor_msgs::msg::Imu& imu_message);
};

#endif  // BMI323DRIVER_H