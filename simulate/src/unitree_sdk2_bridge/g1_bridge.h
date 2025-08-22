#pragma once

#include "unitree_sdk2_bridge_base.h"
#include <unitree/dds_wrapper/robots/g1/g1.h>

class G1Bridge : public UnitreeSDK2BridgeBase
{
using LowCmd_t = unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowCmd_>;
using LowState_t = unitree::robot::g1::publisher::LowState;

public:
    G1Bridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd", [this](const void *msg){
            LowCmd_t::MsgType msg_ = *(const LowCmd_t::MsgType*)msg;
            for(int i(0); i<num_motor_; i++) {
                auto & m = msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        });
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        wireless_controller->joystick = joystick;
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, std::bind(&G1Bridge::run, this));
    }
    
    void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            if(have_frame_sensor_) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
            }
            lowstate->unlockAndPublish();
        }
        // highstate
        if(have_frame_sensor_ && highstate->trylock()) {
            highstate->msg_.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
            highstate->msg_.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
            highstate->msg_.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];
            highstate->msg_.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
            highstate->msg_.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
            highstate->msg_.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }


    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};