//
// Created by han on 9/20/23.
//

#ifndef NOETIX_CORE_MEDULLA_H
#define NOETIX_CORE_MEDULLA_H

#include "EthercatSlaveBase.h"
#include "ethercat.h"

#include <cstddef>
#include <cstdint>

struct MedullaData{
    uint64_t    hs;
    uint64_t    motor_1;
    uint64_t    motor_2;
    uint64_t    motor_3;
    uint64_t    motor_4;
    uint64_t    motor_5;
    uint64_t    motor_6;
    uint64_t    motor_7;
    uint64_t    motor_8;
    uint64_t    motor_9;
    uint64_t    hs_record;
    uint32_t    can1_error_log;
    uint32_t    can2_error_log;
    uint32_t    can3_error_log;
    uint16_t    lz_3_error;
    uint16_t    lz_4_error;
    uint16_t    rec_error_can1;
    uint16_t    rec_error_can2;
    uint16_t    rec_error_can3;
};

struct MedullaCommand{
    uint64_t    hs;
    uint64_t    motor_1;
    uint64_t    motor_2;
    uint64_t    motor_3;
    uint64_t    motor_4;
    uint64_t    motor_5;
    uint64_t    motor_6;
    uint64_t    motor_7;
    uint64_t    motor_8;
    uint64_t    motor_9;
    uint16_t    lz_3;
    uint16_t    lz_4;
    uint16_t    control_word;
    uint16_t    motor_enable;
};

// These structures describe the prefix of the Medulla PDO that this host
// version understands.  sizeof(MedullaData) is 112 on the target ABI because
// the compiler adds two bytes of tail padding; tail padding is not part of the
// EtherCAT wire protocol and must never be copied from the IO map.
constexpr std::size_t kMedullaInputPdoBytes =
    offsetof(MedullaData, rec_error_can3) + sizeof(std::uint16_t);
constexpr std::size_t kMedullaOutputPdoBytes =
    offsetof(MedullaCommand, motor_enable) + sizeof(std::uint16_t);

static_assert(kMedullaInputPdoBytes == 110U,
              "Unexpected Medulla input PDO field layout");
static_assert(kMedullaOutputPdoBytes == 88U,
              "Unexpected Medulla output PDO field layout");
static_assert(sizeof(MedullaData) >= kMedullaInputPdoBytes,
              "MedullaData cannot hold its PDO prefix");
static_assert(sizeof(MedullaCommand) >= kMedullaOutputPdoBytes,
              "MedullaCommand cannot hold its PDO prefix");

class Medulla: public EthercatSlaveBase {
public:
    Medulla(const std::string& name, const uint32_t address);

    MedullaData medulla_data_{};
    MedullaCommand medulla_cmd_{};

    void updateRead() override;
    void updateWrite() override;
    std::size_t expectedInputBytes() const noexcept override { return kMedullaInputPdoBytes; }
    std::size_t expectedOutputBytes() const noexcept override { return kMedullaOutputPdoBytes; }

};


#endif //NOETIX_CORE_MEDULLA_H
