#include "lro_motor_driver_can.hpp"

LRO_Can_Limit_Param lro_can_limit_param[LRO_CAN_Num_Of_Motor] = {
    {12.5, 45.0, 40.0, 500.0, 5.0},       // LRO_CAN_PJ2_55_5550
    {12.5, 45.0, 40.0, 500.0, 5.0},       // LRO_CAN_PJ3_60_6562
    {12.5, 18.0, 90.0, 500.0, 5.0},     // LRO_CAN_PJ3_75_8462
    {12.5, 25.0, 80.0, 500.0, 5.0}        // LRO_CAN_PJ3_97_10062
};

LroMotorDriverCAN::LroMotorDriverCAN(uint16_t motor_id, const std::string& can_interface,
                                     LRO_CAN_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    comm_type_ = CommType::CAN;
    motor_id_ = motor_id;
    limit_param_ = lro_can_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    can_ = MotorsCAN::get(can_interface);

    CanCbkFunc can_callback = std::bind(&LroMotorDriverCAN::can_rx_cbk, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, motor_id_);
}

LroMotorDriverCAN::~LroMotorDriverCAN() {
    can_->remove_can_callback(motor_id_);
}

void LroMotorDriverCAN::lock_motor() {
    can_frame tx_frame{};
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x04;

    tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
    tx_frame.data[1] = motor_id_ & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = LRO_CAN_CMD_ENABLE;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::unlock_motor() {
    can_frame tx_frame{};
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x04;

    tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
    tx_frame.data[1] = motor_id_ & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = LRO_CAN_CMD_DISABLE;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

uint8_t LroMotorDriverCAN::init_motor() {
    LroMotorDriverCAN::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriverCAN::set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriverCAN::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriverCAN::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    switch (error_id_) {
        case LROCanError::LRO_CAN_MOTOR_OVERHEAT:
            return LROCanError::LRO_CAN_MOTOR_OVERHEAT;
        case LROCanError::LRO_CAN_OVER_CURRENT:
            return LROCanError::LRO_CAN_OVER_CURRENT;
        case LROCanError::LRO_CAN_UNDER_VOLTAGE:
            return LROCanError::LRO_CAN_UNDER_VOLTAGE;
        case LROCanError::LRO_CAN_ENCODER_ERROR:
            return LROCanError::LRO_CAN_ENCODER_ERROR;
        case LROCanError::LRO_CAN_BRAKE_OVERVOLT:
            return LROCanError::LRO_CAN_BRAKE_OVERVOLT;
        case LROCanError::LRO_CAN_DRV_ERROR:
            return LROCanError::LRO_CAN_DRV_ERROR;
        default:
            return error_id_;
    }
    return error_id_;
}

void LroMotorDriverCAN::deinit_motor() {
    LroMotorDriverCAN::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool LroMotorDriverCAN::write_motor_flash() {
    return true;
}

bool LroMotorDriverCAN::set_motor_zero() {
    LroMotorDriverCAN::set_motor_zero_lro();
    Timer::sleep_for(setup_sleep_time);
    LroMotorDriverCAN::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);
    logger_->info("motor_id: {0}\tposition: {1}", motor_id_, get_motor_pos());
    LroMotorDriverCAN::unlock_motor();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
}

void LroMotorDriverCAN::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.can_dlc < 0x08) return;
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    uint8_t fb_type = (rx_frame.data[0] >> 5) & 0x07;
    error_id_ = rx_frame.data[0] & 0x1F;
    if (error_id_ > 0) {
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, static_cast<uint32_t>(error_id_));
        }
    }
    pos_int = (static_cast<uint16_t>(rx_frame.data[1]) << 8) | rx_frame.data[2];
    spd_int = (static_cast<uint16_t>(rx_frame.data[3]) << 4) | ((rx_frame.data[4] >> 4) & 0x0F);
    t_int = (static_cast<uint16_t>(rx_frame.data[4] & 0x0F) << 8) | rx_frame.data[5];
    motor_pos_ =
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + static_cast<float>(motor_zero_offset_);
    motor_spd_ =
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ =
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = rx_frame.data[7];
    motor_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[6]) - 25);
}

void LroMotorDriverCAN::get_motor_param(uint8_t param_cmd) {
    can_frame tx_frame{};
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x02;

    tx_frame.data[0] = (uint8_t)(LRO_CAN_MODE_QUERY << 5);
    tx_frame.data[1] = param_cmd;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    float pos_deg = (pos - static_cast<float>(motor_zero_offset_)) * 180.0f / static_cast<float>(M_PI);
    float spd_rpm = std::abs(spd) * 60.0f / (2.0f * static_cast<float>(M_PI));
    uint16_t spd_val = static_cast<uint16_t>(limit(spd_rpm * 10.0f, 0.0f, 32767.0f));
    uint16_t cur_limit = 4095;
    uint8_t ack = 1;
    union32_t rv_type_convert;
    rv_type_convert.f = pos_deg;

    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    uint64_t packed = 0;
    packed |= (static_cast<uint64_t>(LRO_CAN_MODE_POS & 0x07)) << 61;
    packed |= (static_cast<uint64_t>(rv_type_convert.buf[3]) << 29) | (static_cast<uint64_t>(rv_type_convert.buf[2]) << 21)
             | (static_cast<uint64_t>(rv_type_convert.buf[1]) << 13) | (static_cast<uint64_t>(rv_type_convert.buf[0]) << 5);
    packed |= (static_cast<uint64_t>(spd_val & 0x7FFF)) << 14;
    packed |= (static_cast<uint64_t>(cur_limit & 0x0FFF)) << 2;
    packed |= (static_cast<uint64_t>(ack & 0x03));

    for (int i = 0; i < 8; ++i) {
        tx_frame.data[i] = (packed >> (56 - i * 8)) & 0xFF;
    }

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::motor_spd_cmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    float spd_rpm = spd * 60.0f / (2.0f * static_cast<float>(M_PI));
    uint16_t cur_limit = 65535;
    uint8_t ack = 1;
    union32_t rv_type_convert;
    rv_type_convert.f = spd_rpm;

    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x07;

    tx_frame.data[0] = ((LRO_CAN_MODE_SPD & 0x07) << 5) | (ack & 0x03);
    tx_frame.data[1] = rv_type_convert.buf[3];
    tx_frame.data[2] = rv_type_convert.buf[2];
    tx_frame.data[3] = rv_type_convert.buf[1];
    tx_frame.data[4] = rv_type_convert.buf[0];
    tx_frame.data[5] = (cur_limit >> 8) & 0xFF;
    tx_frame.data[6] = cur_limit & 0xFF;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        Timer::sleep_for(normal_sleep_time);
    }
    uint16_t p, v, kp, kd, t;

    f_p -= static_cast<float>(motor_zero_offset_);
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);

    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(9));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    uint64_t packed = 0;
    packed |= (static_cast<uint64_t>(LRO_CAN_MODE_MIT & 0x07)) << 61;
    packed |= (static_cast<uint64_t>(kp & 0x0FFF)) << 49;
    packed |= (static_cast<uint64_t>(kd & 0x01FF)) << 40;
    packed |= (static_cast<uint64_t>(p & 0xFFFF)) << 24;
    packed |= (static_cast<uint64_t>(v & 0x0FFF)) << 12;
    packed |= static_cast<uint64_t>(t & 0x0FFF);

    tx_frame.data[0] = (packed >> 56) & 0xFF;
    tx_frame.data[1] = (packed >> 48) & 0xFF;
    tx_frame.data[2] = (packed >> 40) & 0xFF;
    tx_frame.data[3] = (packed >> 32) & 0xFF;
    tx_frame.data[4] = (packed >> 24) & 0xFF;
    tx_frame.data[5] = (packed >> 16) & 0xFF;
    tx_frame.data[6] = (packed >> 8) & 0xFF;
    tx_frame.data[7] = packed & 0xFF;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) {
        return;
    }
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        Timer::sleep_for(normal_sleep_time);
    }

    for (uint8_t slot = 0; slot < 8; ++slot) {
        float p_f, v_f, kp_f, kd_f, t_f;
        uint16_t p, v, kp, kd, t;

        p_f = limit(f_p[slot] - static_cast<float>(motor_zero_offset_), -limit_param_.PosMax, limit_param_.PosMax);
        v_f = limit(f_v[slot], -limit_param_.SpdMax, limit_param_.SpdMax);
        kp_f = limit(f_kp[slot], 0.0f, limit_param_.OKpMax);
        kd_f = limit(f_kd[slot], 0.0f, limit_param_.OKdMax);
        t_f = limit(f_t[slot], -limit_param_.TauMax, limit_param_.TauMax);

        kp = range_map(kp_f, 0.0f, limit_param_.OKpMax, uint16_t(0), uint16_t(0x0FFF));
        kd = range_map(kd_f, 0.0f, limit_param_.OKdMax, uint16_t(0), uint16_t(0x01FF));
        p = range_map(p_f, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
        v = range_map(v_f, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), uint16_t(0x0FFF));
        t = range_map(t_f, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), uint16_t(0x0FFF));

        uint64_t packed = 0;
        packed |= (static_cast<uint64_t>(LRO_CAN_MODE_MIT & 0x07)) << 61;
        packed |= (static_cast<uint64_t>(kp & 0x0FFF)) << 49;
        packed |= (static_cast<uint64_t>(kd & 0x01FF)) << 40;
        packed |= (static_cast<uint64_t>(p & 0xFFFF)) << 24;
        packed |= (static_cast<uint64_t>(v & 0x0FFF)) << 12;
        packed |= static_cast<uint64_t>(t & 0x0FFF);

        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = (packed >> 56) & 0xFF;
        tx_frame.data[1] = (packed >> 48) & 0xFF;
        tx_frame.data[2] = (packed >> 40) & 0xFF;
        tx_frame.data[3] = (packed >> 32) & 0xFF;
        tx_frame.data[4] = (packed >> 24) & 0xFF;
        tx_frame.data[5] = (packed >> 16) & 0xFF;
        tx_frame.data[6] = (packed >> 8) & 0xFF;
        tx_frame.data[7] = packed & 0xFF;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::set_motor_control_mode(uint8_t motor_control_mode) {
    if (motor_control_mode > LRO_CAN_MODE_CUR) {
        logger_->error("Invalid motor control mode: {} (ID: {})", motor_control_mode, motor_id_);
        return;
    }

    uint8_t old_mode = motor_control_mode_;
    logger_->info("Switching motor control mode: {} -> {} (ID: {})", old_mode, motor_control_mode, motor_id_);

    if (motor_control_mode == LRO_CAN_MODE_MIT && old_mode != LRO_CAN_MODE_MIT) {
        logger_->debug("Sending zero MIT command to synchronize mode (ID: {})", motor_id_);
        motor_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        Timer::sleep_for(normal_sleep_time);
    }

    motor_control_mode_ = motor_control_mode;
}

void LroMotorDriverCAN::set_motor_id(uint8_t old_id, uint8_t new_id) {
    if (old_id < 1 || old_id > 0x7FF || new_id < 1 || new_id > 0x7FF) {
        logger_->error("Invalid ID range: old={}, new={}", old_id, new_id);
        return;
    }

    if (old_id == new_id) {
        logger_->warn("Skipping ID set: Old and New ID are identical ({})", old_id);
        return;
    }

    logger_->info("Changing Motor ID: {} -> {} (Interface: {})", old_id, new_id, can_interface_);

    can_frame tx_frame{};
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x06;

    tx_frame.data[0] = (old_id >> 8) & 0xFF;
    tx_frame.data[1] = old_id & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = LRO_CAN_CMD_SET_ID;
    tx_frame.data[4] = (new_id >> 8) & 0xFF;
    tx_frame.data[5] = new_id & 0xFF;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }

    Timer::sleep_for(setup_sleep_time);
    logger_->info("Set ID command sent. Verify the new ID via bus query.");
}

void LroMotorDriverCAN::reset_motor_id() {
    can_frame tx_frame{};
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x06;

    tx_frame.data[0] = 0x7F;
    tx_frame.data[1] = 0x7F;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = LRO_CAN_CMD_RESET_ID;
    tx_frame.data[4] = 0x7F;
    tx_frame.data[5] = 0x7F;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::set_motor_zero_lro() {
    can_frame tx_frame{};
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x04;

    tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
    tx_frame.data[1] = motor_id_ & 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = LRO_CAN_CMD_SET_ZERO;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::clear_motor_error_lro() {
    {
        can_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.can_dlc = 0x04;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CAN_CMD_DISABLE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
    Timer::sleep_for(normal_sleep_time);
    {
        can_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.can_dlc = 0x04;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CAN_CMD_ENABLE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::write_register_lro(uint8_t rid, float value) {
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&value);

    can_frame tx_frame{};
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x06;

    tx_frame.data[0] = (LRO_CAN_MODE_CONFIG << 5);
    tx_frame.data[1] = rid;
    tx_frame.data[2] = vbuf[0];
    tx_frame.data[3] = vbuf[1];
    tx_frame.data[4] = vbuf[2];
    tx_frame.data[5] = vbuf[3];

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::write_register_lro(uint8_t index, int32_t value) {
    can_frame tx_frame{};
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x06;

    tx_frame.data[0] = (LRO_CAN_MODE_CONFIG << 5);
    tx_frame.data[1] = index;
    tx_frame.data[2] = (value >> 24) & 0xFF;
    tx_frame.data[3] = (value >> 16) & 0xFF;
    tx_frame.data[4] = (value >> 8) & 0xFF;
    tx_frame.data[5] = value & 0xFF;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriverCAN::save_register_lro() {
    logger_->warn("save_register_lro: LRO protocol has no explicit save command, parameters are auto-saved on write");
}

void LroMotorDriverCAN::refresh_motor_status() {
    motor_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void LroMotorDriverCAN::clear_motor_error() {
    clear_motor_error_lro();
}
