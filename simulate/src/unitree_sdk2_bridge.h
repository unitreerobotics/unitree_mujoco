#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/go2/public/jsonize_type.hpp>
#include <unitree/robot/server/server.hpp>

#include <iostream>
#include <mutex>

#include "param.h"
#include "physics_joystick.h"
#include "keyboard_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "keyboard") {
                // Keyboard joystick will be initialized later via setGLFWWindow()
                std::cout << "[Info] Keyboard joystick mode enabled. Waiting for GLFW window..." << std::endl;
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }
    
    // Set GLFW window for keyboard joystick
    virtual void setGLFWWindow(GLFWwindow* window) {
        if(param::config.use_joystick == 1 && param::config.joystick_type == "keyboard") {
            joystick = std::make_shared<KeyboardJoystick>(window);
        }
    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data, const std::string &lowcmd_topic = "rt/lowcmd") : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>(lowcmd_topic);
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
        std::cout << "[Info] Subscribing LowCmd on " << lowcmd_topic << std::endl;
    }

    void setGLFWWindow(GLFWwindow* window) override {
        UnitreeSDK2BridgeBase::setGLFWWindow(window);
        // Sync joystick pointer to lowstate and wireless_controller
        if(lowstate) lowstate->joystick = joystick;
        if(wireless_controller) wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1LocoServer : public unitree::robot::Server
{
public:
    G1LocoServer() : unitree::robot::Server(unitree::robot::g1::LOCO_SERVICE_NAME) {}

    void Init() override
    {
        SetApiVersion(unitree::robot::g1::LOCO_API_VERSION);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_GET_FSM_ID, &G1LocoServer::GetFsmId);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_GET_FSM_MODE, &G1LocoServer::GetFsmMode);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_GET_BALANCE_MODE, &G1LocoServer::GetBalanceMode);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, &G1LocoServer::GetSwingHeight);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, &G1LocoServer::GetStandHeight);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_GET_PHASE, &G1LocoServer::GetPhase);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_FSM_ID, &G1LocoServer::SetFsmId);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_BALANCE_MODE, &G1LocoServer::SetBalanceMode);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, &G1LocoServer::SetSwingHeight);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, &G1LocoServer::SetStandHeight);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_VELOCITY, &G1LocoServer::SetVelocity);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_ARM_TASK, &G1LocoServer::SetArmTask);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SET_SPEED_MODE, &G1LocoServer::SetSpeedMode);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL, &G1LocoServer::SwitchToUserCtrl);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(unitree::robot::g1::ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL, &G1LocoServer::SwitchToInternalCtrl);
    }

private:
    int fsm_id_ = 1;
    int fsm_mode_ = 0;
    int balance_mode_ = 0;
    int speed_mode_ = 0;
    float swing_height_ = 0.0f;
    float stand_height_ = 0.0f;
    bool user_ctrl_enabled_ = false;
    std::mutex mutex_;

    static std::string DataInt(int value)
    {
        unitree::robot::go2::JsonizeDataInt json;
        json.data = value;
        return unitree::common::ToJsonString(json);
    }

    static std::string DataFloat(float value)
    {
        unitree::robot::go2::JsonizeDataFloat json;
        json.data = value;
        return unitree::common::ToJsonString(json);
    }

    static int ParseDataInt(const std::string &parameter, int fallback)
    {
        if (parameter.empty()) {
            return fallback;
        }
        try {
            unitree::robot::go2::JsonizeDataInt json;
            unitree::common::FromJsonString(parameter, json);
            return json.data;
        } catch (...) {
            return fallback;
        }
    }

    static float ParseDataFloat(const std::string &parameter, float fallback)
    {
        if (parameter.empty()) {
            return fallback;
        }
        try {
            unitree::robot::go2::JsonizeDataFloat json;
            unitree::common::FromJsonString(parameter, json);
            return json.data;
        } catch (...) {
            return fallback;
        }
    }

    int32_t GetFsmId(const std::string &, std::string &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data = DataInt(fsm_id_);
        return 0;
    }

    int32_t GetFsmMode(const std::string &, std::string &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data = DataInt(fsm_mode_);
        return 0;
    }

    int32_t GetBalanceMode(const std::string &, std::string &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data = DataInt(balance_mode_);
        return 0;
    }

    int32_t GetSwingHeight(const std::string &, std::string &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data = DataFloat(swing_height_);
        return 0;
    }

    int32_t GetStandHeight(const std::string &, std::string &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data = DataFloat(stand_height_);
        return 0;
    }

    int32_t GetPhase(const std::string &, std::string &data)
    {
        unitree::robot::g1::JsonizeDataVecFloat json;
        json.data = {0.0f, 0.0f};
        data = unitree::common::ToJsonString(json);
        return 0;
    }

    int32_t SetFsmId(const std::string &parameter, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fsm_id_ = ParseDataInt(parameter, fsm_id_);
        return 0;
    }

    int32_t SetBalanceMode(const std::string &parameter, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        balance_mode_ = ParseDataInt(parameter, balance_mode_);
        return 0;
    }

    int32_t SetSwingHeight(const std::string &parameter, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        swing_height_ = ParseDataFloat(parameter, swing_height_);
        return 0;
    }

    int32_t SetStandHeight(const std::string &parameter, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stand_height_ = ParseDataFloat(parameter, stand_height_);
        return 0;
    }

    int32_t SetVelocity(const std::string &, std::string &) { return 0; }
    int32_t SetArmTask(const std::string &, std::string &) { return 0; }

    int32_t SetSpeedMode(const std::string &parameter, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_mode_ = ParseDataInt(parameter, speed_mode_);
        return 0;
    }

    int32_t SwitchToUserCtrl(const std::string &, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        user_ctrl_enabled_ = true;
        return 0;
    }

    int32_t SwitchToInternalCtrl(const std::string &parameter, std::string &)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        user_ctrl_enabled_ = false;
        int mode = ParseDataInt(parameter, 0);
        if (mode == 1) {
            fsm_id_ = 1;
        } else if (mode == 2) {
            fsm_id_ = 500;
        }
        return 0;
    }
};

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data)
        : RobotBridge(model, data, param::config.lowcmd_topic.empty() ? DefaultLowCmdTopic() : param::config.lowcmd_topic)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");

        if (param::config.robot.find("g1") != std::string::npos) {
            loco_server_ = std::make_unique<G1LocoServer>();
            loco_server_->Init();
            loco_server_->Start();
            std::cout << "[Info] Started simulated G1 loco service: " << unitree::robot::g1::LOCO_SERVICE_NAME << std::endl;
        }
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
    std::unique_ptr<G1LocoServer> loco_server_;

private:
    static std::string DefaultLowCmdTopic()
    {
        return param::config.robot.find("g1") != std::string::npos ? "rt/user_lowcmd" : "rt/lowcmd";
    }
};
