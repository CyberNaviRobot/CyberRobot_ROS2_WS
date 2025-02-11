#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor04_odom_kc/odom_mecX4.h"
#include "sensor04_odom_kc/serial_pack.h"
#include "serial_driver/serial_driver.hpp"
#include <chrono>
#include <functional>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

extern std::vector<uint8_t> tx_data_buffer;

//（单位：米）
//轮距
extern fp32 Wheel_Spacing;
//轴距
extern fp32 Alex_Spacing;
//轮子半径
extern fp32 Wheel_Radius;

class Odom_KC_Node : public rclcpp::Node {
public:
  Odom_KC_Node() : Node("Odom_KC_Node") {
    // 声明参数（带默认值）
    // 串口设备默认名（根据实际设备调整）
    this->declare_parameter("device_name", "/dev/ttyACM1");
    // 串口默认波特率
    this->declare_parameter("baud_rate", 115200);

    // 获取参数值
    const std::string device_name =
        this->get_parameter("device_name").as_string();
    const uint32_t baud_rate = this->get_parameter("baud_rate").as_int();

    RCLCPP_INFO(this->get_logger(), "Serial port Node Open!");
    // 创建串口配置对象
    // 波特率默认115200；不开启流控制；无奇偶效验；停止位1。
    drivers::serial_driver::SerialPortConfig config(
        baud_rate, drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    // 初始化串口
    try {
      io_context_ = std::make_shared<drivers::common::IoContext>(1);
      // 初始化 serial_driver_
      serial_driver_ =
          std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
      serial_driver_->init_port(device_name, config);
      serial_driver_->port()->open();

      RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
      RCLCPP_INFO(this->get_logger(), "Using device: %s",
                  serial_driver_->port().get()->device_name().c_str());
      RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s",
                   ex.what());
      return;
    }

    // odom发布方
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // 初始化tf广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 初始化定时器
    //  odom_timer_ =
    //  this->create_wall_timer(std::chrono::duration<DurationRepT, DurationT>
    //  period, CallbackT callback)
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(4),
        std::bind(&Odom_KC_Node::odom_pub_timer_callback, this));

                  // 初始化cmd_vel消息订阅
                  cmd_vel_sub_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&Odom_KC_Node::cmd_vel_sub_callback, this,
                      std::placeholders::_1));

    async_receive_message(); // 进入异步接收
  }

private:
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // 四个轮子的速度(对应单片机的顺序)（单位：rpm）
  fp32 received_encoder_wheel_velocities_[4] = {0.0, 0.0, 0.0, 0.0};

  // 四个轮子的速度(对应单片机的顺序)（单位：2000pc）
  fp32 received_encoder_wheel_angle_[4] = {0.0, 0.0, 0.0, 0.0};

  std::mutex mcu_data_mutex_; // 定义 mutex

  // 从MCU传输的速度数据
  void update_wheel_velocities_from_mcu() {
    std::unique_lock<std::mutex> lock(mcu_data_mutex_); // 加锁
    // 将 RPM 转换为 m/s
    for (short i = 0; i < 4; i++) 
    {
      odom_motor_.encoder_wheel_velocities_[i] =
          received_encoder_wheel_velocities_[i] * 2 * M_PI * Wheel_Radius /
          60.0;
    }
  }

  void async_receive_message() // 创建一个函数更方便重新调用
  {
    auto port = serial_driver_->port();

    // 设置接收回调函数
    port->async_receive(
        [this](const std::vector<uint8_t> &data, const size_t &size) {
          if (size > 0) {
            for (int16_t i = 0; i < (int16_t)size; ++i) {
              uint8_t data_buffer = data[i];
              // 处理接收到的数据
              serial_pack_.rx.Data_Analysis(&data_buffer, 0x01, 0, 0, 0, 0, 8);
            }
            // 由于在ROS2中，node是局部变量，所以发布方只能在node类里，故Data_Apply不写任何东西，直接在接收下面的回调函数里实现功能。
            if (serial_pack_.rx.data.cmd == 0x01) {
              RCLCPP_INFO(this->get_logger(), "\n");
              RCLCPP_INFO(this->get_logger(), "以下是电机编码器的odom数据：");

              std::unique_lock<std::mutex> lock(mcu_data_mutex_); // 加锁

              // 存储电机速度数据
              received_encoder_wheel_velocities_[0] =
                  serial_pack_.rx.data.fp32_buffer[0]; // 电机 0 速度
              received_encoder_wheel_velocities_[1] =
                  serial_pack_.rx.data.fp32_buffer[1]; // 电机 1 速度
              received_encoder_wheel_velocities_[2] =
                  serial_pack_.rx.data.fp32_buffer[2]; // 电机 2 速度
              received_encoder_wheel_velocities_[3] =
                  serial_pack_.rx.data.fp32_buffer[3]; // 电机 3 速度

              // 存储电机位置数据
              received_encoder_wheel_angle_[0] =
                  serial_pack_.rx.data.fp32_buffer[4]; // 电机 0 位置
              received_encoder_wheel_angle_[1] =
                  serial_pack_.rx.data.fp32_buffer[5]; // 电机 1 位置
              received_encoder_wheel_angle_[2] =
                  serial_pack_.rx.data.fp32_buffer[6]; // 电机 2 位置
              received_encoder_wheel_angle_[3] =
                  serial_pack_.rx.data.fp32_buffer[7]; // 电机 3 位置

              // vx = serial_pack_.rx.data.fp32_buffer[8];
              // vy = serial_pack_.rx.data.fp32_buffer[9];
              // vw = serial_pack_.rx.data.fp32_buffer[10];
              // yaw_ = serial_pack_.rx.data.fp32_buffer[11];
              // dt_ = serial_pack_.rx.data.fp32_buffer[12];
              // x_position_ = serial_pack_.rx.data.fp32_buffer[13];
              // y_position_ = serial_pack_.rx.data.fp32_buffer[14];

              // 打印电机速度和位置（角度）
              for (int i = 0; i < 4; ++i) {
                RCLCPP_DEBUG(this->get_logger(),
                             "%d号电机的速度: %.6f RPM, 位置: %.6f (2000pc)", i,
                             received_encoder_wheel_velocities_[i],
                             received_encoder_wheel_angle_[i]);
              }

              // RCLCPP_DEBUG(this->get_logger(),"线速度:x:%.6f,y:%.6f,z:%.6f",vx,vy,0.0f);
              // RCLCPP_DEBUG(this->get_logger(),"角速度:x:%.6f,y:%.6f,z:%.6f",0.0f,0.0f,vw);
              // RCLCPP_DEBUG(this->get_logger(),"欧拉角(逆正顺负):r:%.6f,p:%.6f,y:%.6f",0.0f,0.0f,yaw_);
              // RCLCPP_DEBUG(this->get_logger(),"积分间隔:%.6f",dt_);
              // RCLCPP_DEBUG(this->get_logger(),"右手坐标系X坐标(前正后负):%.6f",x_position_);
              // RCLCPP_DEBUG(this->get_logger(),"右手坐标系Y坐标(左正右负):%.6f",y_position_);
            }
          }
          // 继续监听新的数据
          async_receive_message();
        });
  }

  void odom_pub_timer_callback() 
  {
    update_wheel_velocities_from_mcu();
    
    odom_motor_.Analysis(0.004);

    // 创建odom消息类型
    auto odom_msg = nav_msgs::msg::Odometry();

    // 时间戳
    odom_msg.header.stamp = this->get_clock()->now();

    // 位姿信息所参考的坐标系
    odom_msg.header.frame_id = "odom";

    // 设置child_frame_id（底盘坐标系）
    odom_msg.child_frame_id = "base_link"; // 设置子坐标系为机器人底盘坐标系

    // DOF平动位置
    odom_msg.pose.pose.position.x = odom_motor_.frame_id.x_position;
    odom_msg.pose.pose.position.y = odom_motor_.frame_id.y_position;
    odom_msg.pose.pose.position.z = 0.0;

    // 从欧拉角转换为四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_motor_.frame_id.yaw);
    // DOF转动（四元数）
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 线速度
    odom_msg.twist.twist.linear.x = odom_motor_.child_frame_id.vx;
    odom_msg.twist.twist.linear.y = odom_motor_.child_frame_id.vy;
    odom_msg.twist.twist.linear.z = 0.0;

    // 角速度
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = odom_motor_.child_frame_id.vw;

    // 发布消息
    odom_pub_->publish(odom_msg);

    // 打印位置（XYZ）
    RCLCPP_DEBUG(this->get_logger(), "位置Position(XYZ): %.6f, %.6f, %.6f",
                 odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                 odom_msg.pose.pose.position.z);

    // 打印姿态（四元数WXYZ）
    RCLCPP_DEBUG(
        this->get_logger(), "姿态Orientation(WXYZ): %.6f, %.6f, %.6f, %.6f",
        odom_msg.pose.pose.orientation.w, // 注意顺序：WXYZ
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z);

    // 打印姿态（欧拉角RPY）
    RCLCPP_DEBUG(this->get_logger(), "欧拉角Euler(RPY): %.6f, %.6f, %.6f", 0.0,
                 0.0, odom_motor_.frame_id.yaw);

    // 打印线速度（XYZ）
    RCLCPP_DEBUG(this->get_logger(), "线速度LinearVel(XYZ): %.6f, %.6f, %.6f",
                 odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y,
                 odom_msg.twist.twist.linear.z);

    // 打印角速度（XYZ）
    RCLCPP_DEBUG(this->get_logger(), "角速度AngularVel(XYZ): %.6f, %.6f, %.6f",
                 odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y,
                 odom_msg.twist.twist.angular.z);

    /* 接下来发布tf */
    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header.stamp = odom_msg.header.stamp; // 时间戳与odom同步
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = odom_msg.pose.pose.position.z;
    transform.transform.rotation = odom_msg.pose.pose.orientation; // 复用四元数
    tf_broadcaster_->sendTransform(transform);
  }

  void cmd_vel_sub_callback(const geometry_msgs::msg::Twist &cmd_msg) {
    // bool bool_buffer[] = {1, 0, 1, 0};
    // int8_t int8_buffer[] = {0x11,0x22};
    // int16_t int16_buffer[] = {2000,6666};
    // int32_t int32_buffer[] = {305419896};
    fp32 fp32_buffer[] = {(fp32)cmd_msg.linear.x,  (fp32)cmd_msg.linear.y,
                          (fp32)cmd_msg.linear.z,  (fp32)cmd_msg.angular.x,
                          (fp32)cmd_msg.angular.y, (fp32)cmd_msg.angular.z};

    // 由于ROS2中node为局部变量，所以只能在node中调用send函数，所以Serial_Transmit只负责处理data_buffer。
    serial_pack_.tx.Data_Pack(0x01, nullptr, 0, nullptr, 0, nullptr, 0, nullptr,
                              0, fp32_buffer,
                              sizeof(fp32_buffer) / sizeof(fp32));

    auto port = serial_driver_->port();

    try {
      size_t bytes_size = port->send(tx_data_buffer);

      RCLCPP_DEBUG(this->get_logger(), "平动XYZ：%.6f,%.6f,%.6f",
                   fp32_buffer[0], fp32_buffer[1], fp32_buffer[2]);
      RCLCPP_DEBUG(this->get_logger(), "转动XYZ：%.6f,%.6f,%.6f",
                   fp32_buffer[3], fp32_buffer[4], fp32_buffer[5]);
      RCLCPP_DEBUG(this->get_logger(), "(%ld bytes)", bytes_size);
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",
                   ex.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Odom_KC_Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}