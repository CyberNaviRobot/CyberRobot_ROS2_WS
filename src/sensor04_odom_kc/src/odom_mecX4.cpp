#include "sensor04_odom_kc/odom_mecX4.h"
#include <cmath>

ODOM_Motor odom_motor_;

// 坐标系满足右手坐标系




// 麦克纳姆轮底盘参数
fp32 Wheel_Spacing = 0.093;  // 轮间距（单位：米）
fp32 Alex_Spacing = 0.085;   // 轮距（单位：米）
fp32 Wheel_Radius = 0.0375; // 轮子半径（单位：米)


void ODOM_Motor::Analysis(fp32 dt)
{
	this->dt = dt;
    // 计算机器人在 base_link 下的速度（child_frame_id）
    // 注意：此处的 vx/vy/vw 是 base_link 坐标系下的瞬时速度

    // 计算机器人前进的线速度和角速度，公式不需要轮半径
    //线速度，四个轮子在机器人的运动学模型中贡献相同，所以要除以4
    this->child_frame_id.vx = ( encoder_wheel_velocities_[0] + encoder_wheel_velocities_[1] - encoder_wheel_velocities_[2] - encoder_wheel_velocities_[3]) / 4.0f;
    this->child_frame_id.vy = ( encoder_wheel_velocities_[0] - encoder_wheel_velocities_[1] - encoder_wheel_velocities_[2] + encoder_wheel_velocities_[3]) / 4.0f;

    //线速度，轮距（wheel_spacing） 决定了左右轮对旋转的贡献程度，轴距（alex_spacing） 决定了前后轮对旋转的贡献程度，
    //所以要除以底盘尺寸，alex_spacing + wheel_spacing 是底盘尺寸。
    this->child_frame_id.vw = (-encoder_wheel_velocities_[0] - encoder_wheel_velocities_[1] - encoder_wheel_velocities_[2] - encoder_wheel_velocities_[3]) / (4.0f * (Wheel_Spacing + Alex_Spacing));


    // 3. 更新 odom 坐标系下的位姿
    this->frame_id.yaw += this->child_frame_id.vw * this->dt;
//		this->frame_id.yaw_deg = this->frame_id.yaw *  180.0f / 3.14159265358979f;

    // 保证 yaw 始终在 -PI 到 PI 之间(归一化:需限制在 [-π, π])
    if(this->frame_id.yaw > M_PI) 
		{
			this->frame_id.yaw -= 2.0 * M_PI;
		}
    if(this->frame_id.yaw < -M_PI) 
		{
			this->frame_id.yaw += 2.0 * M_PI;
		}

    // 计算相对于child_frame_id(base_link)坐标系的角度, 将 base_link 速度转换到 odom 坐标系，并积分位置
    fp32 cos_yaw = std::cos(this->frame_id.yaw);
    fp32 sin_yaw = std::sin(this->frame_id.yaw);

    this->frame_id.x_position += (child_frame_id.vx * cos_yaw - child_frame_id.vy * sin_yaw) * dt;
    this->frame_id.y_position += (child_frame_id.vx * sin_yaw + child_frame_id.vy * cos_yaw) * dt;
    
}
