//
// Created by han on 23-9-3.
//

#include "can_protocol_func.h"

#include <legged_common/SpscRingBuffer.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>

// Common pre-clamp ranges (applied in pack_pvt_cmd_ex before dispatching per motor type).
// Should be the union (widest envelope) of all supported motor types below.
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN (-12.5f)
#define POS_MAX 12.5f

// DMBOT-specific ranges
#define DMBOT_KP_MIN 0.0f
#define DMBOT_KP_MAX 500.0f
#define DMBOT_KD_MIN 0.0f
#define DMBOT_KD_MAX 5.0f
#define DMBOT_POS_MIN (-12.5f)
#define DMBOT_POS_MAX 12.5f

// INKEXBOT-specific ranges
#define INKEXBOT_KP_MIN 0.0f
#define INKEXBOT_KP_MAX 500.0f
#define INKEXBOT_KD_MIN 0.0f
#define INKEXBOT_KD_MAX 50.0f
#define INKEXBOT_POS_MIN (-12.5f)
#define INKEXBOT_POS_MAX 12.5f

const char* GetErrorString(MotorError error_code) {
    switch (error_code) {
        case MotorError::NO_ERROR:           return "无错误";
        case MotorError::MOTOR_OVERHEAT:     return "电机过热";
        case MotorError::MOTOR_OVERCURRENT:  return "电机过流";
        case MotorError::MOTOR_UNDERVOLTAGE: return "电机欠压";
        case MotorError::ENCODER_ERROR:      return "编码器故障";
        case MotorError::RESERVED_5:         return "保留";
        case MotorError::BRAKE_OVERVOLTAGE:  return "刹车过压";
        case MotorError::DRV_ERROR:          return "DRV驱动错误";
        default:                                return "未知错误";
    }
}

bool IsKnownInkexMotorFault(MotorError error_code) {
    switch (error_code) {
        case MotorError::MOTOR_OVERHEAT:
        case MotorError::MOTOR_OVERCURRENT:
        case MotorError::ENCODER_ERROR:
        case MotorError::MOTOR_UNDERVOLTAGE:
        case MotorError::BRAKE_OVERVOLTAGE:
        case MotorError::DRV_ERROR:
            return true;
        default:
            return false;
    }
}

const char* GetErrorString(DmMotorError error_code) {
    switch (error_code) {
        case DmMotorError::MOTOR_DISABLED:         return "电机未使能";
        case DmMotorError::MOTOR_ENABLED:          return "电机已使能";
        case DmMotorError::MOTOR_AXES_CALIB_FAULT: return "电机轴校准故障";
        case DmMotorError::MOTOR_SENSOR_FAULT:     return "电机传感器故障";
        case DmMotorError::MOTOR_ENCODER_FAULT:    return "电机编码器故障";
        case DmMotorError::MOTOR_OVERVOLTAGE:     return "电机电压过高";
        case DmMotorError::MOTOR_UNDERVOLTAGE:    return "电机电压过低";
        case DmMotorError::MOTOR_OVERCURRENT:     return "电机过流";
        case DmMotorError::MOTOR_MOS_OVERHEAT:    return "电机MOS过热";
        case DmMotorError::MOTOR_LINE_OVERHEAT:   return "电机线圈过热";
        case DmMotorError::NO_COMM:               return "无通信";
        case DmMotorError::MOTOR_OVERLOAD:        return "过载";
        default:                                  return "未知错误";
    }
}

bool IsKnownDmMotorFault(DmMotorError error_code) {
    switch (error_code) {
        case DmMotorError::MOTOR_AXES_CALIB_FAULT:
        case DmMotorError::MOTOR_SENSOR_FAULT:
        case DmMotorError::MOTOR_ENCODER_FAULT:
        case DmMotorError::MOTOR_OVERVOLTAGE:
        case DmMotorError::MOTOR_UNDERVOLTAGE:
        case DmMotorError::MOTOR_OVERCURRENT:
        case DmMotorError::MOTOR_MOS_OVERHEAT:
        case DmMotorError::MOTOR_LINE_OVERHEAT:
        case DmMotorError::NO_COMM:
        case DmMotorError::MOTOR_OVERLOAD:
            return true;
        default:
            return false;
    }
}

namespace {

constexpr std::size_t kMotorErrorLogCapacity = 1024;
constexpr std::size_t kMotorErrorLogBatchSize = 64;
constexpr std::size_t kMotorTemperatureSnapshotCapacity = 8;
constexpr auto kMotorErrorPollPeriod = std::chrono::milliseconds(10);
constexpr auto kMotorErrorFlushPeriod = std::chrono::seconds(1);
constexpr auto kMotorFaultSummaryPeriod = std::chrono::seconds(1);
constexpr uint32_t kMotorFaultTimeCheckStride = 32U;
constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;
constexpr const char* kMotorErrorLogHeader =
    "ros_time,event,sample_count,node_no,motor_no,joint_no,error_code,error_name,"
    "motor_temp_c,mos_temp_c,current_a";

struct MotorErrorLogRecord {
    uint32_t stamp_sec;
    uint32_t stamp_nsec;
    int node_no;
    int motor_no;
    int joint_no;
    int error_code;
    const char* error_name;
    bool is_error;
    bool print_terminal;
    const char* event_name;
    uint32_t sample_count;
    uint8_t motor_temp;
    uint8_t mos_temp;
    float current;
};

struct MotorTemperatureSnapshot {
    uint32_t stamp_sec;
    uint32_t stamp_nsec;
    std::array<JointTempRecord, MAX_JOINTS> joints;
};

uint64_t ToNanoseconds(uint32_t stamp_sec, uint32_t stamp_nsec) noexcept
{
    return static_cast<uint64_t>(stamp_sec) * kNanosecondsPerSecond + stamp_nsec;
}

bool PeriodElapsed(uint64_t now, uint64_t last, bool initialized) noexcept
{
    return !initialized || now < last || now - last >= kNanosecondsPerSecond;
}

bool EnsureDirectoryExists(const std::string& directory)
{
    if (directory.empty()) {
        return true;
    }

    struct stat stat_buffer {};
    if (stat(directory.c_str(), &stat_buffer) == 0) {
        return S_ISDIR(stat_buffer.st_mode);
    }

    const std::size_t slash_pos = directory.find_last_of('/');
    if (slash_pos != std::string::npos) {
        const std::string parent = (slash_pos == 0) ? "/" : directory.substr(0, slash_pos);
        if (!EnsureDirectoryExists(parent)) {
            return false;
        }
    }

    return mkdir(directory.c_str(), 0755) == 0 || errno == EEXIST;
}

std::string GetParentDirectory(const std::string& path)
{
    const std::size_t slash_pos = path.find_last_of('/');
    if (slash_pos == std::string::npos) {
        return std::string();
    }
    if (slash_pos == 0) {
        return "/";
    }
    return path.substr(0, slash_pos);
}

std::string GetMotorErrorLogPath(const std::string& configured_path)
{
    std::string log_path = configured_path;
    if (log_path.empty()) {
        return std::string();
    }
    if (log_path == "none" || log_path == "NONE" ||
        log_path == "off" || log_path == "OFF" ||
        log_path == "disable" || log_path == "DISABLE" ||
        log_path == "disabled" || log_path == "DISABLED") {
        return std::string();
    }

    const std::string parent_directory = GetParentDirectory(log_path);
    if (!EnsureDirectoryExists(parent_directory)) {
        std::cerr << "[motor_error_logger] failed to create log directory for "
                  << log_path << ", "
                  << "falling back to /tmp/e2_motor_error.log\n";
        return "/tmp/e2_motor_error.log";
    }

    return log_path;
}

bool FileIsNonEmpty(const std::string& path)
{
    struct stat stat_buffer {};
    return stat(path.c_str(), &stat_buffer) == 0 && stat_buffer.st_size > 0;
}

bool HasMotorErrorLogHeader(const std::string& path)
{
    std::ifstream input(path);
    std::string first_line;
    return input.is_open() && std::getline(input, first_line) &&
           first_line == kMotorErrorLogHeader;
}

std::string MakeHeaderedLogPath(const std::string& path)
{
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const std::size_t slash_pos = path.find_last_of('/');
    const std::size_t dot_pos = path.find_last_of('.');
    const bool has_extension = dot_pos != std::string::npos &&
                               (slash_pos == std::string::npos || dot_pos > slash_pos);
    const std::string stem = has_extension ? path.substr(0, dot_pos) : path;
    const std::string extension = has_extension ? path.substr(dot_pos) : std::string();
    const std::string base = stem + ".headered-" + std::to_string(timestamp);

    std::string candidate = base + extension;
    unsigned int suffix = 1;
    struct stat stat_buffer {};
    while (stat(candidate.c_str(), &stat_buffer) == 0) {
        candidate = base + '-' + std::to_string(suffix++) + extension;
    }
    return candidate;
}

void PrintMotorErrorToTerminal(const MotorErrorLogRecord& rec)
{
    std::cout << (rec.is_error ? "电机故障" : "电机恢复")
              << " event=" << rec.event_name
              << ", 节点=" << std::setw(WIDTH_NODE) << std::right << rec.joint_no
              << ", 错误码=0x" << std::hex << std::setw(2) << std::setfill('0')
              << rec.error_code << std::dec << std::setfill(' ')
              << "(" << std::setw(10) << std::left << rec.error_name << ")"
              << PRINT_FIELD("当前电机温度", static_cast<unsigned>(rec.motor_temp), WIDTH_TEMP) << "°C"
              << PRINT_FIELD("当前MOS温度", static_cast<unsigned>(rec.mos_temp), WIDTH_TEMP) << "°C"
              << ", 电流=" << rec.current
              << ", 样本数=" << rec.sample_count << '\n';
}

void PrintMotorTemperatureSnapshot(const MotorTemperatureSnapshot& snapshot)
{
    std::cout << "motor temperature snapshot " << snapshot.stamp_sec << '.'
              << std::setw(9) << std::setfill('0') << snapshot.stamp_nsec
              << std::setfill(' ') << '\n';
    for (int i = 0; i < MAX_JOINTS; ++i) {
        const JointTempRecord& rec = snapshot.joints[static_cast<std::size_t>(i)];
        std::cout << (rec.valid ? "aa节点=" : "bb节点=")
                  << std::setw(WIDTH_NODE) << std::right << i;
        if (rec.valid) {
            std::cout << PRINT_FIELD("当前电机温度", static_cast<unsigned>(rec.curr_motor_temp), WIDTH_TEMP) << "°C";
        }
        std::cout << PRINT_FIELD("最大电机温度", static_cast<unsigned>(rec.max_motor_temp), WIDTH_TEMP) << "°C";
        if (rec.valid) {
            std::cout << PRINT_FIELD("当前MOS温度", static_cast<unsigned>(rec.curr_mos_temp), WIDTH_TEMP) << "°C";
        }
        std::cout << PRINT_FIELD("最大MOS温度", static_cast<unsigned>(rec.max_mos_temp), WIDTH_TEMP) << "°C"
                  << ", 电流(INK)/扭矩(DM)=" << rec.curr_current << '\n';
    }
}

class MotorErrorAsyncLogger {
public:
    MotorErrorAsyncLogger() = default;

    ~MotorErrorAsyncLogger()
    {
        Stop();
    }

    void Start(const std::string& configured_path)
    {
        std::lock_guard<std::mutex> lock(start_mutex_);
        if (started_.load(std::memory_order_acquire)) {
            return;
        }

        log_path_ = GetMotorErrorLogPath(configured_path);
        if (log_path_.empty()) {
            std::cerr << "[motor_error_logger] log file disabled; terminal worker remains active\n";
        }

        stop_ = false;
        worker_ = std::thread(&MotorErrorAsyncLogger::Run, this);
        started_.store(true, std::memory_order_release);
    }

    void Stop()
    {
        {
            std::lock_guard<std::mutex> start_lock(start_mutex_);
            if (!started_.load(std::memory_order_acquire)) {
                return;
            }

            started_.store(false, std::memory_order_release);
            stop_.store(true, std::memory_order_release);
        }

        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void Enqueue(const MotorErrorLogRecord& record)
    {
        if (!started_.load(std::memory_order_acquire)) {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        if (!records_.push(record)) {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void EnqueueTemperatureSnapshot(const MotorTemperatureSnapshot& snapshot) noexcept
    {
        if (!started_.load(std::memory_order_acquire) ||
            !temperature_snapshots_.push(snapshot)) {
            dropped_temperature_snapshot_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    bool IsStarted() const
    {
        return started_.load(std::memory_order_acquire);
    }

private:
    void Run()
    {
        if (!log_path_.empty() && FileIsNonEmpty(log_path_) && !HasMotorErrorLogHeader(log_path_)) {
            const std::string replacement_path = MakeHeaderedLogPath(log_path_);
            std::cerr << "[motor_error_logger] existing log has no recognized header: "
                      << log_path_ << "; preserving it and writing new records to "
                      << replacement_path << '\n';
            log_path_ = replacement_path;
        }

        std::ofstream log_file;
        if (!log_path_.empty()) {
            log_file.open(log_path_, std::ios::out | std::ios::app);
        }
        if (!log_path_.empty() && !log_file.is_open()) {
            std::cerr << "[motor_error_logger] failed to open " << log_path_
                      << ", motor errors will only be printed to terminal\n";
        }

        if (log_file.is_open() && log_file.tellp() == 0) {
            log_file << kMotorErrorLogHeader << '\n';
            log_file.flush();
            if (!log_file) {
                std::cerr << "[motor_error_logger] failed to write log header to "
                          << log_path_ << '\n';
            }
        }

        auto last_flush = std::chrono::steady_clock::now();
        while (!stop_.load(std::memory_order_acquire)) {
            std::array<MotorErrorLogRecord, kMotorErrorLogBatchSize> batch;
            std::size_t batch_size = 0;
            MotorErrorLogRecord record{};
            while (batch_size < batch.size() && records_.pop(record)) {
                batch[batch_size++] = record;
            }

            MotorTemperatureSnapshot snapshot{};
            bool printed_temperature_snapshot = false;
            while (temperature_snapshots_.pop(snapshot)) {
                PrintMotorTemperatureSnapshot(snapshot);
                printed_temperature_snapshot = true;
            }

            const uint64_t dropped = dropped_count_.exchange(0, std::memory_order_relaxed);
            if (dropped > 0) {
                if (log_file.is_open()) {
                    log_file << "# dropped " << dropped
                             << " motor log records because the async queue was full\n";
                }
                std::cout << "[motor_error_logger] dropped " << dropped
                          << " records because the async queue was full\n";
            }

            for (std::size_t i = 0; i < batch_size; ++i) {
                const MotorErrorLogRecord& rec = batch[i];

                if (log_file.is_open()) {
                    log_file << rec.stamp_sec << '.'
                             << std::setw(9) << std::setfill('0') << rec.stamp_nsec
                             << std::setfill(' ')
                             << ',' << rec.event_name
                             << ',' << rec.sample_count
                             << ',' << rec.node_no
                             << ',' << rec.motor_no
                             << ',' << rec.joint_no
                             << ",0x" << std::hex << rec.error_code << std::dec
                             << ',' << rec.error_name
                             << ',' << static_cast<unsigned>(rec.motor_temp)
                             << ',' << static_cast<unsigned>(rec.mos_temp)
                             << ',' << rec.current
                             << '\n';
                }

                if (rec.print_terminal) {
                    PrintMotorErrorToTerminal(rec);
                }
            }
            const uint64_t dropped_snapshots =
                dropped_temperature_snapshot_count_.exchange(0, std::memory_order_relaxed);
            if (dropped_snapshots > 0) {
                std::cout << "[motor_error_logger] dropped " << dropped_snapshots
                          << " temperature snapshots because the async queue was full\n";
            }

            const auto now = std::chrono::steady_clock::now();
            if (log_file.is_open() && now - last_flush >= kMotorErrorFlushPeriod) {
                log_file.flush();
                last_flush = now;
            }
            if (batch_size == 0 && !printed_temperature_snapshot) {
                std::this_thread::sleep_for(kMotorErrorPollPeriod);
            }
        }

        MotorErrorLogRecord record{};
        while (records_.pop(record)) {
            if (log_file.is_open()) {
                log_file << record.stamp_sec << '.'
                         << std::setw(9) << std::setfill('0') << record.stamp_nsec
                         << std::setfill(' ')
                         << ',' << record.event_name
                         << ',' << record.sample_count
                         << ',' << record.node_no
                         << ',' << record.motor_no
                         << ',' << record.joint_no
                         << ",0x" << std::hex << record.error_code << std::dec
                         << ',' << record.error_name
                         << ',' << static_cast<unsigned>(record.motor_temp)
                         << ',' << static_cast<unsigned>(record.mos_temp)
                         << ',' << record.current << '\n';
            }
            if (record.print_terminal) {
                PrintMotorErrorToTerminal(record);
            }
        }
        MotorTemperatureSnapshot snapshot{};
        while (temperature_snapshots_.pop(snapshot)) {
            PrintMotorTemperatureSnapshot(snapshot);
        }
        if (log_file.is_open()) {
            log_file.flush();
        }
    }

    std::string log_path_;
    legged::SpscRingBuffer<MotorErrorLogRecord, kMotorErrorLogCapacity> records_;
    legged::SpscRingBuffer<MotorTemperatureSnapshot, kMotorTemperatureSnapshotCapacity>
        temperature_snapshots_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> started_{false};
    std::mutex start_mutex_;
    std::atomic<uint64_t> dropped_count_{0};
    std::atomic<uint64_t> dropped_temperature_snapshot_count_{0};
    std::thread worker_;
};

MotorErrorAsyncLogger g_motor_error_logger;
std::atomic<bool> g_log_motor_error{true};
std::atomic<bool> g_enable_check_and_print{true};
std::atomic<bool> g_log_motor_non_error{false};
std::atomic<bool> g_enable_thermal_protection_limit_release{true};
std::array<JointTempRecord, MAX_JOINTS> g_joint_temps{};
std::array<uint64_t, MAX_JOINTS> g_last_motor_non_error_log_ns{};
std::array<bool, MAX_JOINTS> g_motor_non_error_log_initialized{};
uint64_t g_last_temperature_snapshot_ns{0U};
bool g_temperature_snapshot_initialized{false};

bool ParseBoolEnv(const char* value, bool default_value)
{
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }

    const std::string text(value);
    return text == "1" || text == "true" || text == "TRUE" ||
           text == "on" || text == "ON" || text == "yes" || text == "YES";
}

bool ShouldLogMotorError()
{
    return g_log_motor_error.load(std::memory_order_relaxed);
}

bool ShouldLogMotorNonError()
{
    return g_log_motor_non_error.load(std::memory_order_relaxed);
}

bool ShouldCheckAndPrint()
{
    return g_enable_check_and_print.load(std::memory_order_relaxed);
}

bool ShouldThermalProtectionLimitRelease()
{
    return g_enable_thermal_protection_limit_release.load(std::memory_order_relaxed);
}

}  // namespace

void InitMotorErrorLogger(const std::string& log_path)
{
    g_log_motor_error.store(
        ParseBoolEnv(std::getenv("E2_LOG_MOTOR_ERROR"), true),
        std::memory_order_relaxed);
    g_enable_check_and_print.store(
        ParseBoolEnv(std::getenv("E2_ENABLE_CHECK_AND_PRINT"), true),
        std::memory_order_relaxed);
    g_log_motor_non_error.store(
        ParseBoolEnv(std::getenv("E2_LOG_MOTOR_NON_ERROR"), false),
        std::memory_order_relaxed);
    g_motor_error_logger.Start(log_path);
}

void SetMotorErrorLoggingEnabled(bool enabled)
{
    g_log_motor_error.store(enabled, std::memory_order_relaxed);
}

void SetCheckAndPrintEnabled(bool enabled)
{
    g_enable_check_and_print.store(enabled, std::memory_order_relaxed);
}

void SetMotorNonErrorLoggingEnabled(bool enabled)
{
    g_log_motor_non_error.store(enabled, std::memory_order_relaxed);
}

void SetThermalProtectionLimitReleaseEnabled(bool enabled)
{
    g_enable_thermal_protection_limit_release.store(enabled, std::memory_order_relaxed);
}

namespace {

void EnqueueMotorErrorLog(uint32_t stamp_sec, uint32_t stamp_nsec, int node_no, int motor_no,
                          int joint_no, int error_code, const char* error_name,
                          uint8_t motor_temp, uint8_t mos_temp, float current,
                          bool is_error, bool print_terminal,
                          const char* event_name, uint32_t sample_count)
{
    if (!g_motor_error_logger.IsStarted()) {
        return;
    }

    MotorErrorLogRecord record{};
    record.stamp_sec = stamp_sec;
    record.stamp_nsec = stamp_nsec;
    record.node_no = node_no;
    record.motor_no = motor_no;
    record.joint_no = joint_no;
    record.error_code = error_code;
    record.error_name = error_name;
    record.is_error = is_error;
    record.print_terminal = print_terminal;
    record.event_name = event_name;
    record.sample_count = sample_count;
    record.motor_temp = motor_temp;
    record.mos_temp = mos_temp;
    record.current = current;

    g_motor_error_logger.Enqueue(record);
}

struct MotorFaultLogState {
    bool active{false};
    int error_code{0};
    uint32_t samples_since_log{0};
    std::chrono::steady_clock::time_point last_log_time{};
};

std::array<MotorFaultLogState, MAX_JOINTS> g_motor_fault_log_states{};

bool UpdateMotorFaultLog(uint32_t stamp_sec, uint32_t stamp_nsec,
                         int node_no, int motor_no, int joint_no,
                         int error_code, const char* error_name, bool is_fault,
                         uint8_t motor_temp, uint8_t mos_temp, float current)
{
    if (!ShouldLogMotorError() || joint_no < 0 || joint_no >= MAX_JOINTS) {
        return false;
    }

    auto& state = g_motor_fault_log_states[static_cast<std::size_t>(joint_no)];
    if (!is_fault) {
        if (!state.active) {
            return false;
        }
        EnqueueMotorErrorLog(stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                             error_code, error_name, motor_temp, mos_temp, current,
                             false, true, "RECOVERED", 1U);
        state = MotorFaultLogState{};
        return true;
    }

    if (!state.active || state.error_code != error_code) {
        const auto now = std::chrono::steady_clock::now();
        const char* const event_name = state.active ? "FAULT_CHANGED" : "FAULT";
        state.active = true;
        state.error_code = error_code;
        state.samples_since_log = 0U;
        state.last_log_time = now;
        EnqueueMotorErrorLog(stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                             error_code, error_name, motor_temp, mos_temp, current,
                             true, true, event_name, 1U);
        return false;
    }

    if (state.samples_since_log != UINT32_MAX) {
        ++state.samples_since_log;
    }
    if (state.samples_since_log % kMotorFaultTimeCheckStride != 0U) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - state.last_log_time >= kMotorFaultSummaryPeriod) {
        EnqueueMotorErrorLog(stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                             error_code, error_name, motor_temp, mos_temp, current,
                             true, true, "FAULT_REPEAT", state.samples_since_log);
        state.samples_since_log = 0U;
        state.last_log_time = now;
    }
    return false;
}

}  // namespace

static void RecordJointTemp(int joint_no, uint8_t motor_temp, uint8_t mos_temp, float current)
{
    if (joint_no < 0 || joint_no >= MAX_JOINTS) {
        return;
    }

    JointTempRecord* rec = &g_joint_temps[joint_no];
    
    // 记录本周期数据
    rec->curr_motor_temp = motor_temp;
    rec->curr_mos_temp = mos_temp;
    rec->curr_current = current;
    rec->valid = true;
    
    // 更新最大值
    if (!rec->max_initialized) {
        rec->max_motor_temp = motor_temp;
        rec->max_mos_temp = mos_temp;
        rec->max_initialized = true;
    } else {
        if (motor_temp > rec->max_motor_temp) rec->max_motor_temp = motor_temp;
        if (mos_temp > rec->max_mos_temp) rec->max_mos_temp = mos_temp;
    }
}

static bool ShouldLogMotorNonErrorNow(int joint_no, uint32_t stamp_sec,
                                      uint32_t stamp_nsec) noexcept
{
    if (!ShouldLogMotorNonError() || joint_no < 0 || joint_no >= MAX_JOINTS) {
        return false;
    }

    const uint64_t now = ToNanoseconds(stamp_sec, stamp_nsec);
    const std::size_t joint_index = static_cast<std::size_t>(joint_no);
    uint64_t& last_log_time = g_last_motor_non_error_log_ns[joint_index];
    if (!PeriodElapsed(now, last_log_time,
                       g_motor_non_error_log_initialized[joint_index])) {
        return false;
    }

    last_log_time = now;
    g_motor_non_error_log_initialized[joint_index] = true;
    return true;
}

void PublishMotorTemperatureSnapshotIfDue(uint32_t stamp_sec, uint32_t stamp_nsec) noexcept
{
    if (!ShouldCheckAndPrint()) {
        return;
    }
    const uint64_t now = ToNanoseconds(stamp_sec, stamp_nsec);
    if (!g_temperature_snapshot_initialized) {
        g_last_temperature_snapshot_ns = now;
        g_temperature_snapshot_initialized = true;
        return;
    }
    if (!PeriodElapsed(now, g_last_temperature_snapshot_ns,
                       g_temperature_snapshot_initialized)) {
        return;
    }

    MotorTemperatureSnapshot snapshot{};
    snapshot.stamp_sec = stamp_sec;
    snapshot.stamp_nsec = stamp_nsec;
    snapshot.joints = g_joint_temps;
    g_motor_error_logger.EnqueueTemperatureSnapshot(snapshot);
    g_last_temperature_snapshot_ns = now;
    for (JointTempRecord& record : g_joint_temps) {
        record.valid = false;
    }
}

// 添加回调变量
static TemperatureDataCallback temperature_callback_ = nullptr;

void SetTemperatureDataCallback(TemperatureDataCallback callback)
{
    temperature_callback_ = callback;
}

void can_msg_hs_unpack(const std::shared_ptr<ControlFSMData>& a, int slave_no, uint64_t data)
{
    uint64_t tmp_ff = 0xff;
    for (int i = 0; i < 6; ++i) {
        auto id_tmp = (uint8_t)((data >> ((i) * 8)) & tmp_ff);
        a->hs_data[slave_no][i] = id_tmp;
    }
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}



void unpack_pvt_data_ex(uint8_t *buffer, const std::shared_ptr<ControlFSMData>& a,
                         int node_no, int motor_no, int joint_no, const MotorType& motor_type,
                         uint32_t stamp_sec, uint32_t stamp_nsec)
{
    switch (motor_type) {
        case MotorType::DMBOT: 
        {
            // 数据解析
            uint8_t error_code_raw = (buffer[0] & 0xF0) >> 4;
            uint8_t raw_motor_temp = static_cast<uint8_t>(buffer[7]);
            uint8_t raw_mos_temp = static_cast<uint8_t>(buffer[6]);
            int pos_int = buffer[1] << 8 | buffer[2];
            int spd_int = buffer[3] << 4 | (buffer[4] & 0xF0) >> 4;
            int torque_int = (buffer[4] & 0x0F) << 8 | buffer[5];

            // 转换为DMBOT错误码枚举
            DmMotorError error_code = static_cast<DmMotorError>(error_code_raw);
            
            uint8_t current_motor_temp = 0;
            uint8_t current_mos_temp = 0;

            if ((raw_motor_temp > 0) && (raw_mos_temp > 0)) 
            {
                current_motor_temp = static_cast<uint8_t>(buffer[7]);
                current_mos_temp = static_cast<uint8_t>(buffer[6]);

                // 记录温度数据（只记录，不打印）
                RecordJointTemp(joint_no, current_motor_temp, current_mos_temp, torque_int);

                // 只在实时路径中入队，文件写入和终端输出交给后台线程。
                const bool is_fault = IsKnownDmMotorFault(error_code);
                const bool recovery_logged = UpdateMotorFaultLog(
                    stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                    static_cast<int>(error_code), GetErrorString(error_code), is_fault,
                    current_motor_temp, current_mos_temp, torque_int);
                if (!is_fault) { // 否则（没有错误码）检查是否到达打印时间
                    if (!recovery_logged &&
                        ShouldLogMotorNonErrorNow(joint_no, stamp_sec, stamp_nsec)) {
                        const char* status_name =
                            (error_code == DmMotorError::MOTOR_DISABLED ||
                             error_code == DmMotorError::MOTOR_ENABLED)
                                ? GetErrorString(error_code)
                                : "未知状态";
                        EnqueueMotorErrorLog(stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                                             static_cast<int>(error_code), status_name,
                                             current_motor_temp, current_mos_temp, torque_int,
                                             false, false, "STATUS", 1U);
                    }
                }
            }

            a->position_act_raw_[node_no][motor_no] = a->joint_dir_[joint_no]*uint_to_float(pos_int, DMBOT_POS_MIN, DMBOT_POS_MAX, 16);

            a->velocity_act_raw_[node_no][motor_no] = a->joint_dir_[joint_no]*uint_to_float(
                    spd_int, static_cast<float>(a->joint_min_velocity_[joint_no]),
                    static_cast<float>(a->joint_max_velocity_[joint_no]), 12);

            a->torque_act_raw_[node_no][motor_no] = a->joint_dir_[joint_no]*uint_to_float(torque_int, static_cast<float>(a->joint_min_torque_rec_[joint_no]),
                                                                                            static_cast<float>(a->joint_max_torque_rec_[joint_no]), 12);
        }
        break;
        case MotorType::INKEXBOT: {
            // 数据解析
            uint8_t error_code_raw = buffer[0] & 0x1F;
            uint8_t raw_motor_temp = static_cast<uint8_t>(buffer[6]);
            uint8_t raw_mos_temp = static_cast<uint8_t>(buffer[7]);
            int pos_int = buffer[1] << 8 | buffer[2];
            int spd_int = buffer[3] << 4 | (buffer[4] & 0xF0) >> 4;
            int cur_int = (buffer[4] & 0x0F) << 8 | buffer[5];
            float current = (cur_int * 280.0 / 4095.0) - 140.0;

            // 转换为错误码枚举
            MotorError error_code = static_cast<MotorError>(error_code_raw);

            uint8_t current_motor_temp = 0;
            uint8_t current_mos_temp = 0;

            if ((raw_motor_temp >= 0) && (raw_mos_temp >= 0))
            {
                if (ShouldThermalProtectionLimitRelease()) {
                    // 解除热保护限制
                    current_motor_temp = static_cast<uint8_t>(raw_motor_temp);
                    current_mos_temp = static_cast<uint8_t>(raw_mos_temp);
                } else {
                    // 未解除热保护限制
                    raw_motor_temp = (raw_motor_temp >= 50) ? raw_motor_temp : 50;
                    raw_mos_temp = (raw_mos_temp >= 50) ? raw_mos_temp : 50;
                    current_motor_temp = static_cast<uint8_t>((raw_motor_temp - 50) / 2);
                    current_mos_temp = static_cast<uint8_t>((raw_mos_temp - 50) / 2);
                }

                // 记录温度数据（只记录，不打印）
                RecordJointTemp(joint_no, current_motor_temp, current_mos_temp, current);

                // 只在实时路径中入队，文件写入和终端输出交给后台线程。
                const bool is_fault = IsKnownInkexMotorFault(error_code);
                const bool recovery_logged = UpdateMotorFaultLog(
                    stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                    static_cast<int>(error_code), GetErrorString(error_code), is_fault,
                    current_motor_temp, current_mos_temp, current);
                if (!is_fault) { // 否则（没有错误码）检查是否到达打印时间
                    if (!recovery_logged &&
                        ShouldLogMotorNonErrorNow(joint_no, stamp_sec, stamp_nsec)) {
                        const char* status_name =
                            (error_code == MotorError::NO_ERROR ||
                             error_code == MotorError::RESERVED_5)
                                ? GetErrorString(error_code)
                                : "未知状态";
                        EnqueueMotorErrorLog(stamp_sec, stamp_nsec, node_no, motor_no, joint_no,
                                             static_cast<int>(error_code), status_name,
                                             current_motor_temp, current_mos_temp, current,
                                             false, false, "STATUS", 1U);
                    }
                }
            }

            a->position_act_raw_[node_no][motor_no] = a->joint_dir_[joint_no]*uint_to_float(pos_int, INKEXBOT_POS_MIN, INKEXBOT_POS_MAX, 16);

            a->velocity_act_raw_[node_no][motor_no] = a->joint_dir_[joint_no]*uint_to_float(
                    spd_int, static_cast<float>(a->joint_min_velocity_[joint_no]),
                    static_cast<float>(a->joint_max_velocity_[joint_no]), 12);

            a->torque_act_raw_[node_no][motor_no] = a->joint_dir_[joint_no]*uint_to_float(cur_int, static_cast<float>(a->joint_min_torque_rec_[joint_no]),
                                                                                            static_cast<float>(a->joint_max_torque_rec_[joint_no]), 12);
        }
        break;
        default:
        break;
    }
}

// todo: assume rang is symmetry
bool pack_pvt_cmd_ex(uint8_t *buffer, float kp, float kd, float pos, float spd, float tor,
                    int joint_no, const std::shared_ptr<ControlFSMData>& a,
                    const MotorType& motor_type)
{
    if (buffer == nullptr) {
        return false;
    }
    if (!a || joint_no < 0 || joint_no >= MAX_JOINTS ||
        (motor_type != MotorType::DMBOT && motor_type != MotorType::INKEXBOT)) {
        std::fill_n(buffer, 8, uint8_t{0});
        return false;
    }

    PvtCommandValues command{kp, kd, pos, spd, tor};
    const bool command_valid = SanitizePvtCommandValues(command);
    kp = command.kp;
    kd = command.kd;
    pos = command.pos;
    spd = command.spd;
    tor = command.tor;

    const float min_velocity = static_cast<float>(a->joint_min_velocity_[joint_no]);
    const float max_velocity = static_cast<float>(a->joint_max_velocity_[joint_no]);
    const float min_torque = static_cast<float>(a->joint_min_torque_send_[joint_no]);
    const float max_torque = static_cast<float>(a->joint_max_torque_send_[joint_no]);
    const float direction = static_cast<float>(a->joint_dir_[joint_no]);
    if (!std::isfinite(min_velocity) || !std::isfinite(max_velocity) ||
        !std::isfinite(min_torque) || !std::isfinite(max_torque) ||
        !std::isfinite(direction) || !(min_velocity < max_velocity) ||
        !(min_torque < max_torque) || (direction != -1.0F && direction != 1.0F)) {
        std::fill_n(buffer, 8, uint8_t{0});
        return false;
    }

    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > static_cast<float>(a->joint_max_velocity_[joint_no]))
        spd = static_cast<float>(a->joint_max_velocity_[joint_no]);
    else if (spd < static_cast<float>(a->joint_min_velocity_[joint_no]))
        spd = static_cast<float>(a->joint_min_velocity_[joint_no]);
    if (tor > a->joint_max_torque_send_[joint_no])
        tor = static_cast<float>(a->joint_max_torque_send_[joint_no]);
    else if (tor < a->joint_min_torque_send_[joint_no])
        tor = static_cast<float>(a->joint_min_torque_send_[joint_no]);

    float new_pos = direction * pos;
    float new_spd = direction * spd;
    float new_tor = direction * tor;

    switch (motor_type) {
        case MotorType::DMBOT: {
            kp_int = float_to_uint(kp, DMBOT_KP_MIN, DMBOT_KP_MAX, 12);
            kd_int = float_to_uint(kd, DMBOT_KD_MIN, DMBOT_KD_MAX, 12);
            pos_int = float_to_uint(new_pos, DMBOT_POS_MIN, DMBOT_POS_MAX, 16);
            spd_int = float_to_uint(new_spd, min_velocity, max_velocity, 12);
            tor_int = float_to_uint(new_tor, min_torque, max_torque, 12);


            buffer[0] = pos_int >> 8;
            buffer[1] = pos_int;
            buffer[2] = spd_int >> 4;
            buffer[3] = ((spd_int & 0xF)<<4)|(kp_int >> 8);
            buffer[4] = kp_int;
            buffer[5] = (kd_int >> 4);
            buffer[6] = ((kd_int&0xF)<<4)|(tor_int>>8);
            buffer[7] = tor_int;
        }
        break;
        case MotorType::INKEXBOT: {
            kp_int = float_to_uint(kp, INKEXBOT_KP_MIN, INKEXBOT_KP_MAX, 12);
            kd_int = float_to_uint(kd, INKEXBOT_KD_MIN, INKEXBOT_KD_MAX, 9);
            pos_int = float_to_uint(new_pos, INKEXBOT_POS_MIN, INKEXBOT_POS_MAX, 16);
            spd_int = float_to_uint(new_spd, min_velocity, max_velocity, 12);
            tor_int = float_to_uint(new_tor, min_torque, max_torque, 12);

            buffer[0] = 0x00 | (kp_int >> 7);                             // kp5
            buffer[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
            buffer[2] = kd_int & 0xFF;
            buffer[3] = pos_int >> 8;
            buffer[4] = pos_int & 0xFF;
            buffer[5] = spd_int >> 4;
            buffer[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
            buffer[7] = tor_int & 0xff;
        }
        break;
        default:
        std::fill_n(buffer, 8, uint8_t{0});
        return false;
    }
    return command_valid;
}


void read_motor_setting(const EthercatOptionsFromYaml& config, const std::shared_ptr<ControlFSMData>& a)
{
    const auto assign_joint = [&config, &a](int joint, int motor_type, int direction) {
        a->joint_max_torque_rec_[joint] = config.motor_max_torque_rec[motor_type];
        a->joint_min_torque_rec_[joint] = config.motor_min_torque_rec[motor_type];
        a->joint_max_torque_send_[joint] = config.motor_max_torque_send[motor_type];
        a->joint_min_torque_send_[joint] = config.motor_min_torque_send[motor_type];
        a->joint_max_current_[joint] = config.motor_max_current[motor_type];
        a->joint_min_current_[joint] = config.motor_min_current[motor_type];
        a->joint_max_velocity_[joint] = config.motor_max_velocity[motor_type];
        a->joint_min_velocity_[joint] = config.motor_min_velocity[motor_type];
        a->joint_dir_[joint] = direction;
    };

    // Reference E2 order: right arm, left arm, left leg, right leg, waist.
    for (int i = 0; i < 4; ++i) {
        assign_joint(i, config.right_arm_motor_type[i], config.right_arm_motor_dir[i]);
        assign_joint(i + 4, config.left_arm_motor_type[i], config.left_arm_motor_dir[i]);
    }
    for (int i = 0; i < 6; ++i) {
        assign_joint(i + 8, config.left_leg_motor_type[i], config.left_leg_motor_dir[i]);
        assign_joint(i + 14, config.right_leg_motor_type[i], config.right_leg_motor_dir[i]);
    }
    for (int i = 0; i < 3; ++i) {
        assign_joint(i + 20, config.waist_motor_type[i], config.waist_motor_dir[i]);
    }


  }
