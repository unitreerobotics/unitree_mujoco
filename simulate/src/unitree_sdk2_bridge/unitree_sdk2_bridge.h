#ifndef UNITREE_SDK2_BRIDGE_H
#define UNITREE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <cstring>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <mujoco/mujoco.h>

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define MOTOR_SENSOR_NUM 3

class UnitreeSdk2Bridge
{
public:
    UnitreeSdk2Bridge(mjModel *model, mjData *data);
    ~UnitreeSdk2Bridge();

    void LowCmdHandler(const void *msg);
    void PublishLowState();
    void PublishHighState();
    void Run();
    void PrintSceneInformation();
    void CheckSensor();

    unitree_go::msg::dds_::LowState_ low_state{};
    unitree_go::msg::dds_::SportModeState_ high_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::SportModeState_> high_state_puber_;

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_suber_;

    ThreadPtr lowStatePuberThreadPtr;
    ThreadPtr HighStatePuberThreadPtr;

    mjData *mj_data_;
    mjModel *mj_model_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
};

#endif
