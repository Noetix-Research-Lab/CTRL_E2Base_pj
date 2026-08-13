//
// Created by han on 23-9-3.
//

#ifndef BAER_CORE_CAN_PROTOCOL_FUNC_H
#define BAER_CORE_CAN_PROTOCOL_FUNC_H

#include "cstdint"
#include <chrono>
#include <functional>
#include <iomanip>
#include <mutex>
#include <string>
#include "../FSM_States/ControlFSMData.h"
#include "EthercatParameter.h"
#include "PvtCommandSafety.h"

// ==================== 电机报文及报错 ====================

// INKEX错误信息枚举 (bits 0-4)
enum class MotorError : uint8_t {
    NO_ERROR = 0,               // 0: 无错误
    MOTOR_OVERHEAT = 0x01,         // 1: 电机过热
    MOTOR_OVERCURRENT = 0x02,      // 2: 电机过流
    ENCODER_ERROR = 0x03,          // 3: 电机编码器错误
    MOTOR_UNDERVOLTAGE = 0x04,     // 4: 电机电压过低
    RESERVED_5 = 0x05,
    BRAKE_OVERVOLTAGE = 0x06,      // 6: 电机刹车电压过高
    DRV_ERROR = 0x07               // 7: DRV 驱动错误
};

// DMBOT错误信息枚举
enum class DmMotorError : uint8_t {
    MOTOR_DISABLED = 0,
    MOTOR_ENABLED = 0x01,
    MOTOR_AXES_CALIB_FAULT = 0x03,      // 电机轴故障
    MOTOR_SENSOR_FAULT = 0x04,          // 电机传感器故障
    MOTOR_ENCODER_FAULT = 0x05,         // 电机编码器故障
    MOTOR_OVERVOLTAGE = 0x08,       // 电机电压过高
    MOTOR_UNDERVOLTAGE = 0x09,      // 电机电压过低
    MOTOR_OVERCURRENT = 0x0A,       // 电机过流
    MOTOR_MOS_OVERHEAT = 0x0B,      // 电机MOS过热
    MOTOR_LINE_OVERHEAT = 0x0C,     // 电机线圈过热
    NO_COMM = 0x0D,                 // 无通信
    MOTOR_OVERLOAD = 0x0E           // 过载
};

// 时间戳（最准确，不受丢帧影响）
#define PRINT_TIME_1000MS  (1000)

#define MOTOR_TEMP_LIMIT 180
#define MOS_TEMP_LIMIT   120
// 定义列宽常量
const int WIDTH_NODE   = 2;   // 节点号 (8, 10, 13等)
const int WIDTH_ERR    = 5;   // 错误码 0xXX
const int WIDTH_TEMP   = 3;   // 温度值 (°C)
const int WIDTH_LIMIT  = 3;   // 上限值

// 辅助宏：统一格式
#define PRINT_FIELD(name, value, width) \
    ", " << name << "=" << std::setw(width) << std::right << (value)

#define MAX_JOINTS 23

// 关节温度数据结构
typedef struct {
    // 本周期数据
    uint8_t curr_motor_temp;
    uint8_t curr_mos_temp;
    float curr_current;
    bool valid;              // 本周期是否收到数据
    
    // 历史最大温度
    uint8_t max_motor_temp;
    uint8_t max_mos_temp;
    bool max_initialized;    // 最大值是否已初始化
} JointTempRecord;

// 所有关节的温度数据数组
static JointTempRecord g_joint_temps[MAX_JOINTS] = {0};

// 打印控制
static std::mutex g_print_mutex;
static auto g_last_print_time = std::chrono::steady_clock::now();

// 温度数据回调函数类型 (使用整数类型)
using TemperatureDataCallback = std::function<void(int, uint8_t, uint8_t, uint8_t, uint8_t)>;
// joint_no, motor_temp, mos_temp, motor_temp_max, mos_temp_max

void SetTemperatureDataCallback(TemperatureDataCallback callback);

void InitMotorErrorLogger(const std::string& log_path = std::string());
void SetMotorErrorLoggingEnabled(bool enabled);
void SetCheckAndPrintEnabled(bool enabled);
void SetMotorNonErrorLoggingEnabled(bool enabled);
void SetThermalProtectionLimitReleaseEnabled(bool enabled);

enum class MotorType {
    YOBOT = 1, DMBOT, LZBOT, INKEXBOT, UNKNOWN
};

void can_msg_hs_unpack(const std::shared_ptr<ControlFSMData>& a, int slave_no, uint64_t data);

int float_to_uint(float x, float x_min, float x_max, int bits);

bool pack_pvt_cmd_ex(uint8_t *buffer, float kp, float kd, float pos, float spd, float tor,
                   int joint_no, const std::shared_ptr<ControlFSMData>& a, const MotorType& motor_type);

void unpack_pvt_data_ex(uint8_t *buffer, const std::shared_ptr<ControlFSMData>& a,
                     int node_no, int motor_no, int joint_no, const MotorType& motor_type,
                     uint32_t stamp_sec, uint32_t stamp_nsec);


void read_motor_setting(const EthercatOptionsFromYaml& config, const std::shared_ptr<ControlFSMData>& a);



#endif //BAER_CORE_CAN_PROTOCOL_FUNC_H
