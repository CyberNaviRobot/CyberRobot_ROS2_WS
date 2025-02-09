#include "sensor03_imu/WitStandardProtocol.h"
#include <string.h> 


Wit_IMU wit_imu_;


bool Wit_IMU::RX::Data_Analysis(uint8_t *msg_data)
{
    // 包头检测阶段
    if (finded_flag == 0) 
    {
        if (*msg_data == 0x55) 
        {
            this->data.buffer[rx_cnt++] = *msg_data;
            finded_flag = 1;
        }
        else
        {
            rx_cnt = 0;  // 确保在丢失同步时重置
        }
        return false;
    }

    // 防止缓冲区溢出
    if (rx_cnt >= sizeof(this->data.buffer)) 
    {
        finded_flag = 0;
        rx_cnt = 0;
        return false;
    }

    // 数据接收阶段
    this->data.buffer[rx_cnt++] = *msg_data;

    // （包头1 + cmd1 + 数据8 + SUM1 = 11字节）
    if (finded_flag == 1) 
    {
        // 接收功能码
        if (rx_cnt == 2) 
        {
            this->data.cmd = this->data.buffer[1];
        }
    }

        // 完整数据包接收完成
    if (rx_cnt >= 11) //一帧有11个字节
    {
        // SUM校验验证
        if (wit_imu_.checksum.__SUMCRC(this->data.buffer, 10) == this->data.buffer[10]) 
        {
            // 直接解析4个16位数据（小端格式）
            for (int i = 0; i < 4; ++i) 
            {
                this->data.int16_buffer[i] = (this->data.buffer[2*i+3] << 8) | this->data.buffer[2*i+2];
            }
            finded_flag = 0;
            rx_cnt = 0;
            return true;
        }

        // 校验失败重置
        finded_flag = 0;
        rx_cnt = 0;
    }
    return false;
}


uint8_t Wit_IMU::CHECKSUM::__SUMCRC(uint8_t *puchMsg, uint16_t usDataLen)
{
    int16_t i = 0;
	uint8_t uchSUMCRC = 0x00;
    for (; i < usDataLen; i++)
    {
			uchSUMCRC += puchMsg[i];
    }
    return uchSUMCRC;
}

uint16_t Wit_IMU::CHECKSUM::__CRC16_Check(uint8_t *puchMsg,uint16_t usDataLen)
{
    uint16_t uchCRC16 = 0xFFFF;
    uint8_t state,i,j;
    for(i = 0; i < usDataLen; i++ )
    {
        uchCRC16 ^= puchMsg[i];
        for( j = 0; j < 8; j++)
        {
            state = uchCRC16 & 0x01;
            uchCRC16 >>= 1;
            if(state)
            {
                uchCRC16 ^= 0xA001;
            }
        }
    }
    return uchCRC16;
}

int16_t Wit_IMU::CONVERT::Bytes2Short(uint8_t DH,uint8_t DL)
{
	int16_t result = (int16_t)((int16_t)DH << 8 | DL);
	return result;
}

int32_t Wit_IMU::CONVERT::Bytes2Int(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL)
{
    int32_t result = (int32_t)((int32_t)DH << 24 | (int32_t)D2 << 16 | (int32_t)D3 << 8 | DL);
    return result;
}

fp32 Wit_IMU::CONVERT::Bytes2Fp32(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL)
{
    uint8_t bytes[4] = {DL, D3, D2, DH};
    fp32 result;
    memcpy(&result, bytes, sizeof(result));
    return result;
}