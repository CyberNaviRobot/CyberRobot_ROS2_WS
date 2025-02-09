#ifndef __WIT_STANDARD_PROTOCOL_H_
#define __WIT_STANDARD_PROTOCOL_H_

// #ifdef __cplusplus
//  extern "C" {
// #endif

#include "sensor03_imu/struct_typedef.h"
#include <iterator>

typedef struct
{
	struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Accel;

	struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Gyro;
	
	struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Magnet;
	
	struct
	{
		fp32 q[4];
	}Quat;//归一化四元数
	
	struct
	{
		fp32 yaw;    //偏航角  前进的偏移
		fp32 pitch;  //俯仰角	 前后的上下摆动
		fp32 roll;   //翻滚角  左右的上下摆动
	}Euler;//欧拉角
	
	fp32 Temp;
	
}Wit_IMU_t;

class Wit_IMU
{
    public:
    class RX
    {
        public:
        class DATA
        {
            public:
            uint8_t buffer[11];
            uint8_t cmd;
            int16_t int16_buffer[4];

            Wit_IMU_t imu;
        }data;
        bool Data_Analysis(uint8_t *msg_data);

		uint16_t rx_cnt = 0;
    	uint8_t finded_flag = 0;
    }rx;

    class CHECKSUM
	{
		public:
		uint8_t __SUMCRC(uint8_t *puchMsg, uint16_t usDataLen);
		uint16_t __CRC16_Check(uint8_t *puchMsg,uint16_t usDataLen);
	}checksum;

    class CONVERT
	{
		public:
		int16_t Bytes2Short(uint8_t DH,uint8_t DL);
		int32_t Bytes2Int(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL);
		fp32 Bytes2Fp32(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL);
	}convert;
};

extern Wit_IMU wit_imu_;


// #ifdef __cplusplus
// }
// #endif

#endif
