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
#include <unitree/idl/go2/WirelessController_.hpp>
#include <mujoco/mujoco.h>
#include "../joystick/joystick.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_WIRELESS_CONTROLLER "rt/wirelesscontroller"
#define MOTOR_SENSOR_NUM 3

typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

typedef struct
{
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;

    uint8_t idle[16];
} xRockerBtnDataStruct;

// Defaults to xbox gamepad
struct JoystickId
{

    map<string, int> axis =
        {
            {"LX", 0}, // Left stick axis x
            {"LY", 1}, // Left stick axis y
            {"RX", 3}, // Right stick axis x
            {"RY", 4}, // Right stick axis y
            {"LT", 2}, // Left trigger
            {"RT", 5}, // Right trigger
            {"DX", 6}, // Directional pad x
            {"DY", 7}, // Directional pad y
    };

    map<string, int> button =
        {
            {"X", 2},
            {"Y", 3},
            {"B", 1},
            {"A", 0},
            {"LB", 4},
            {"RB", 5},
            {"SELECT", 6},
            {"START", 7},
    };
};

class UnitreeSdk2Bridge
{
public:
    UnitreeSdk2Bridge(mjModel *model, mjData *data);
    ~UnitreeSdk2Bridge();

    void LowCmdHandler(const void *msg);
    void PublishLowState();
    void PublishHighState();
    void PublishWirelessController();
    void Run();
    void PrintSceneInformation();
    void CheckSensor();
    void SetupJoystick(string device, string js_type, int bits);

    unitree_go::msg::dds_::LowState_ low_state{};
    unitree_go::msg::dds_::SportModeState_ high_state{};
    unitree_go::msg::dds_::WirelessController_ wireless_controller{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::SportModeState_> high_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> wireless_controller_puber_;

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_suber_;

    ThreadPtr lowStatePuberThreadPtr;
    ThreadPtr HighStatePuberThreadPtr;
    ThreadPtr WirelessControllerPuberThreadPtr;

    xKeySwitchUnion dds_keys_ = {};
    xRockerBtnDataStruct wireless_remote_ = {};

    JoystickId js_id_;
    Joystick *js_;
    int max_value_ = (1 << 15); // 16 bits joystick

    mjData *mj_data_;
    mjModel *mj_model_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
};

#endif
