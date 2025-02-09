#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "sensor03_imu/WitStandardProtocol.h"
#include "sensor_msgs/msg/imu.hpp"
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <tf2/LinearMath/Quaternion.h>



class ImuNode: public rclcpp::Node
{
  public:
    ImuNode():Node("ImuNode_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"惯性计imu节点启动!");
      
      //创建odom话题通信发布方
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu",10);

    // 声明参数（带默认值）
    // 串口默认设备名（根据实际设备调整）
    this->declare_parameter("device_name", "/dev/ttyUSB0");
    //串口默认波特率
    this->declare_parameter("baud_rate", 115200);

    // 获取参数值
    const std::string device_name = this->get_parameter("device_name").as_string();
    const uint32_t baud_rate = this->get_parameter("baud_rate").as_int();

      // 创建串口配置对象
      // 波特率默认115200；不开启流控制；无奇偶效验；停止位1。
      drivers::serial_driver::SerialPortConfig config(
          baud_rate,
          drivers::serial_driver::FlowControl::NONE,
          drivers::serial_driver::Parity::NONE,
          drivers::serial_driver::StopBits::ONE);
      
      // 初始化串口
      try
      {
        io_context_ = std::make_shared<drivers::common::IoContext>(1);
        // 初始化 serial_driver_
        serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
        serial_driver_->init_port(device_name, config);
        serial_driver_->port()->open();
        
        RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
        RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
        return;
      }

      async_receive_message();   //进入异步接收
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;

    void async_receive_message()  //创建一个函数更方便重新调用
    {
      auto port = serial_driver_->port();

      // 设置接收回调函数
      port->async_receive([this](const std::vector<uint8_t> &data,const size_t &size) 
      {
          if (size > 0)
          {
            RCLCPP_DEBUG(this->get_logger(),"\n");
            RCLCPP_DEBUG(this->get_logger(),"以下是接收到的IMU惯性计数据:");
            //创建imu消息类型
            auto imu_msg = sensor_msgs::msg::Imu();
            bool valid_frame = false;  // 添加有效帧标志
            bool valid_data = false;  // 添加有效帧标志

            for(int16_t i = 0;i < (int16_t)size;++i)
            {
              uint8_t data_buffer = data[i];
              // RCLCPP_INFO(this->get_logger(),"接收到的字节：%x",data_buffer);
              // 处理接收到的数据
              if(wit_imu_.rx.Data_Analysis(&data_buffer) == true)
              {
                valid_frame = true;  // 标记有有效帧需要处理
              }

              if(valid_frame == true)
              {
                if(wit_imu_.rx.data.cmd == 0x51)
                {
                  // 存储加速度计数据
                  wit_imu_.rx.data.imu.Accel.X = wit_imu_.rx.data.int16_buffer[0];
                  wit_imu_.rx.data.imu.Accel.Y = wit_imu_.rx.data.int16_buffer[1];
                  wit_imu_.rx.data.imu.Accel.Z = wit_imu_.rx.data.int16_buffer[2];
                  wit_imu_.rx.data.imu.Temp    = wit_imu_.rx.data.int16_buffer[3];
                  
                  //加速度单位m/s2，温度单位℃摄氏度
                  wit_imu_.rx.data.imu.Accel.X = wit_imu_.rx.data.imu.Accel.X / 32768.0f * 16.0f * 9.8f;
                  wit_imu_.rx.data.imu.Accel.Y = wit_imu_.rx.data.imu.Accel.Y / 32768.0f * 16.0f * 9.8f;
                  wit_imu_.rx.data.imu.Accel.Z = wit_imu_.rx.data.imu.Accel.Z / 32768.0f * 16.0f * 9.8f;
                  wit_imu_.rx.data.imu.Temp    =  wit_imu_.rx.data.imu.Temp / 100.0f;

                  // 打印加速度计数据
                  RCLCPP_DEBUG(this->get_logger(), "加速度(XYZ): %.6f, %.6f, %.6f",
                              wit_imu_.rx.data.imu.Accel.X, wit_imu_.rx.data.imu.Accel.Y, wit_imu_.rx.data.imu.Accel.Z);
                  //减少时间戳带来的延迟
                  imu_msg.header.stamp = this->get_clock()->now();
                }

                else if(wit_imu_.rx.data.cmd == 0x52)
                {
                  // 存储陀螺仪数据
                  wit_imu_.rx.data.imu.Gyro.X = wit_imu_.rx.data.int16_buffer[0];
                  wit_imu_.rx.data.imu.Gyro.Y = wit_imu_.rx.data.int16_buffer[1];
                  wit_imu_.rx.data.imu.Gyro.Z = wit_imu_.rx.data.int16_buffer[2];
                  
                  //角速度单位rad/s
                  wit_imu_.rx.data.imu.Gyro.X = wit_imu_.rx.data.imu.Gyro.X / 32768.0f * 2000.0f * (M_PI / 180.0f);
                  wit_imu_.rx.data.imu.Gyro.Y = wit_imu_.rx.data.imu.Gyro.Y / 32768.0f * 2000.0f * (M_PI / 180.0f);
                  wit_imu_.rx.data.imu.Gyro.Z = wit_imu_.rx.data.imu.Gyro.Z / 32768.0f * 2000.0f * (M_PI / 180.0f);

                  // 打印陀螺仪数据
                  RCLCPP_DEBUG(this->get_logger(), "角速度(XYZ): %.6f, %.6f, %.6f",
                              wit_imu_.rx.data.imu.Gyro.X, wit_imu_.rx.data.imu.Gyro.Y, wit_imu_.rx.data.imu.Gyro.Z);
                }

                else if(wit_imu_.rx.data.cmd == 0x54)
                {
                  // 存储磁力计数据(单位lsb)
                  wit_imu_.rx.data.imu.Magnet.X = wit_imu_.rx.data.int16_buffer[0];
                  wit_imu_.rx.data.imu.Magnet.Y = wit_imu_.rx.data.int16_buffer[1];
                  wit_imu_.rx.data.imu.Magnet.Z = wit_imu_.rx.data.int16_buffer[2];
                  
                  //磁场单位uT
                  // wit_imu_.rx.data.imu.Magnet.X = wit_imu_.rx.data.imu.Magnet.X / 1.0f;
                  // wit_imu_.rx.data.imu.Magnet.Y = wit_imu_.rx.data.imu.Magnet.Y / 1.0f;
                  // wit_imu_.rx.data.imu.Magnet.Z = wit_imu_.rx.data.imu.Magnet.Z / 1.0f;

                  // 打印磁力计数据
                  RCLCPP_DEBUG(this->get_logger(), "磁场(XYZ): %.6f, %.6f, %.6f",
                              wit_imu_.rx.data.imu.Magnet.X, wit_imu_.rx.data.imu.Magnet.Y, wit_imu_.rx.data.imu.Magnet.Z);
                }

                else if(wit_imu_.rx.data.cmd == 0x53)
                {
                  // 存储欧拉角数据
                  wit_imu_.rx.data.imu.Euler.roll = wit_imu_.rx.data.int16_buffer[0];
                  wit_imu_.rx.data.imu.Euler.pitch = wit_imu_.rx.data.int16_buffer[1];
                  wit_imu_.rx.data.imu.Euler.yaw = wit_imu_.rx.data.int16_buffer[2];
                  
                  //欧拉角单位dec
                  wit_imu_.rx.data.imu.Euler.roll = wit_imu_.rx.data.imu.Euler.roll / 32768.0f * 180.0f;
                  wit_imu_.rx.data.imu.Euler.pitch = wit_imu_.rx.data.imu.Euler.pitch / 32768.0f * 180.0f;
                  wit_imu_.rx.data.imu.Euler.yaw = wit_imu_.rx.data.imu.Euler.yaw / 32768.0f * 180.0f;

                  // 打印欧拉角数据
                  RCLCPP_DEBUG(this->get_logger(), "欧拉角(XYZ_RPY): %.6f, %.6f, %.6f",
                              wit_imu_.rx.data.imu.Euler.roll, wit_imu_.rx.data.imu.Euler.pitch, wit_imu_.rx.data.imu.Euler.yaw);

                  //欧拉角单位rad
                  wit_imu_.rx.data.imu.Euler.roll = wit_imu_.rx.data.imu.Euler.roll * (M_PI / 180.0f);
                  wit_imu_.rx.data.imu.Euler.pitch = wit_imu_.rx.data.imu.Euler.pitch  * (M_PI / 180.0f);
                  wit_imu_.rx.data.imu.Euler.yaw = wit_imu_.rx.data.imu.Euler.yaw  * (M_PI / 180.0f);
                }

                else if(wit_imu_.rx.data.cmd == 0x59)
                {
                  for(int16_t i = 0 ; i < 4 ; i++)
                  {
                    // 存储四元数数据
                    wit_imu_.rx.data.imu.Quat.q[i] = wit_imu_.rx.data.int16_buffer[i];

                    //四元数是归一化的四元数
                    wit_imu_.rx.data.imu.Quat.q[i] = wit_imu_.rx.data.imu.Quat.q[i] / 32768.0f;
                  }

                  // 打印四元数数据
                  //注意，0对应y，1对应x，-2才对应z。
                  RCLCPP_DEBUG(this->get_logger(), "四元数(xyzw): %.6f, %.6f, %.6f, %.6f",
                              wit_imu_.rx.data.imu.Quat.q[1],wit_imu_.rx.data.imu.Quat.q[0],-wit_imu_.rx.data.imu.Quat.q[2],wit_imu_.rx.data.imu.Quat.q[3]);
                  valid_data = true;
                }
                wit_imu_.rx.data.cmd = 0x00;   //跑过一次就进行清0
                valid_frame = false;
              }
            }

            if(valid_data == true)
            {
              //时间戳
              // imu_msg.header.stamp = this->get_clock()->now();
              //位姿信息所参考的坐标系
              imu_msg.header.frame_id = "imu";

              // tf2::Quaternion q;
              // //DOF欧拉角单位rad
              // q.setRPY(roll_, pitch_, yaw_);
              // //DOF欧拉角（四元数）
              // imu_msg.orientation.x = q.x();
              // imu_msg.orientation.y = q.y();
              // imu_msg.orientation.z = q.z();
              // imu_msg.orientation.w = q.w();

              //DOF欧拉角（四元数）
              //注意对应关系
              // 传感器坐标系 -> ROS坐标系：
              // X -> Y
              // Y -> X
              // Z -> -Z
              imu_msg.orientation.x =   wit_imu_.rx.data.imu.Quat.q[1];
              imu_msg.orientation.y =   wit_imu_.rx.data.imu.Quat.q[0];
              imu_msg.orientation.z = - wit_imu_.rx.data.imu.Quat.q[2];
              imu_msg.orientation.w =   wit_imu_.rx.data.imu.Quat.q[3];

              //加速度
              imu_msg.linear_acceleration.x = wit_imu_.rx.data.imu.Accel.X;
              imu_msg.linear_acceleration.y = wit_imu_.rx.data.imu.Accel.Y;
              imu_msg.linear_acceleration.z = wit_imu_.rx.data.imu.Accel.Z;

              //角速度
              imu_msg.angular_velocity.x = wit_imu_.rx.data.imu.Gyro.X;
              imu_msg.angular_velocity.y = wit_imu_.rx.data.imu.Gyro.Y;
              imu_msg.angular_velocity.z = wit_imu_.rx.data.imu.Gyro.Z;

              imu_pub_->publish(imu_msg);
              valid_data = false;
            }

          }
          // 继续监听新的数据
          async_receive_message();
      }
      );
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node = std::make_shared<ImuNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}