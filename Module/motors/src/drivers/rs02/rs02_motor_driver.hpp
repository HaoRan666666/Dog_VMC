#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can_iso.hpp"
#include "utils.hpp"

// RS02 default host (master) CAN ID — responses arrive with this ID
constexpr uint16_t RS02_HOST_ID = 0xFD;

// RS02 command magic bytes (Byte7 in 8-byte data)
enum RS02Cmd : uint8_t {
    RS02_CMD_ENABLE      = 0xFC,  // 指令1: motor enable
    RS02_CMD_DISABLE     = 0xFD,  // 指令2: motor stop
    RS02_CMD_SET_ZERO    = 0xFE,  // 指令4: set zero position
    RS02_CMD_CLEAR_ERROR = 0xFB,  // 指令5: clear error / read error status
    RS02_CMD_SET_MODE    = 0xFC,  // 指令6: set operation mode (distinguished by data[6])
    RS02_CMD_SET_ID      = 0xFA,  // 指令7: modify motor CAN ID
    RS02_CMD_SAVE        = 0xF8,  // 指令12: save motor data
};

// RS02 operation modes (used with 指令6)
enum RS02Mode : uint8_t {
    RS02_MODE_MIT = 0,  // MIT impedance control (default)
    RS02_MODE_POS = 1,  // Position mode (CSP)
    RS02_MODE_SPD = 2,  // Speed mode
};

// RS02 CAN ID bit layout for control commands (standard 11-bit frame)
// bits 10-8: mode type (0=MIT, 1=POS, 2=SPD, 3=READ, 4=WRITE)
// bits 7-0:  motor CAN ID
constexpr uint8_t RS02_MODE_MIT_CAN  = 0;  // MIT mode bits[10:8]
constexpr uint8_t RS02_MODE_POS_CAN  = 1;  // Position mode bits[10:8]
constexpr uint8_t RS02_MODE_SPD_CAN  = 2;  // Speed mode bits[10:8]
constexpr uint8_t RS02_MODE_READ_CAN = 3;  // Read param bits[10:8]
constexpr uint8_t RS02_MODE_WRITE_CAN = 4; // Write param bits[10:8]

// RS02 motor parameter limits
typedef struct {
    float PosMax;   // Maximum position (rad), default 12.57
    float SpdMax;   // Maximum velocity (rad/s), default 44
    float TauMax;   // Maximum torque (Nm), default 17
    float OKpMax;   // Maximum Kp, default 500
    float OKdMax;   // Maximum Kd, default 5
} RS02_Limit_Param;

class Rs02MotorDriver : public MotorDriver {
   public:
    Rs02MotorDriver(uint16_t motor_id, const std::string& can_interface,
                    double motor_zero_offset = 0.0);
    ~Rs02MotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;
    virtual void get_motor_param(uint8_t param_cmd) override;

    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const override {
        return response_count_;
    }
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) override;
    virtual void reset_motor_id() override;
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    std::atomic<int> response_count_{0};
    RS02_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};

    void set_motor_zero_rs02();
    void clear_motor_error_rs02();
    void write_register_rs02(uint16_t index, float value);
    void read_register_rs02(uint16_t index);

    virtual void can_rx_cbk(const can_frame& rx_frame);
    std::shared_ptr<MotorsCAN> can_;
};
