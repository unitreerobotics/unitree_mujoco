#include "unitree_sdk2_bridge.h"

UnitreeSdk2Bridge::UnitreeSdk2Bridge(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    low_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_puber_->InitChannel();

    high_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    high_state_puber_->InitChannel();

    low_cmd_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowCmdHandler, this, placeholders::_1), 1);

    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishLowState, this);
    HighStatePuberThreadPtr = CreateRecurrentThreadEx("highstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishHighState, this);

    CheckSensor();
}

UnitreeSdk2Bridge::~UnitreeSdk2Bridge(){};

void UnitreeSdk2Bridge::LowCmdHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}

void UnitreeSdk2Bridge::PublishLowState()
{
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            low_state.motor_state()[i].q() = mj_data_->sensordata[i];
            low_state.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
            low_state.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
        }

        if (have_frame_sensor_)
        {
            low_state.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
            low_state.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
            low_state.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
            low_state.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

            low_state.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
            low_state.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
            low_state.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

            low_state.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
            low_state.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
            low_state.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
        }

        low_state_puber_->Write(low_state);
    }
}

void UnitreeSdk2Bridge::PublishHighState()
{
    if (mj_data_ && have_frame_sensor_)
    {

        high_state.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
        high_state.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
        high_state.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];

        high_state.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
        high_state.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
        high_state.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];

        high_state_puber_->Write(high_state);
    }
};

void UnitreeSdk2Bridge::Run()
{
    while (1)
    {
        sleep(2);
    }
};

void UnitreeSdk2Bridge::PrintSceneInformation()
{
    cout << endl;

    cout << "<<------------- Link ------------->> " << endl;
    for (int i = 0; i < mj_model_->nbody; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_BODY, i);
        if (name)
        {
            cout << "link_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Joint ------------->> " << endl;
    for (int i = 0; i < mj_model_->njnt; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
        if (name)
        {
            cout << "joint_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Actuator ------------->> " << endl;
    for (int i = 0; i < mj_model_->nu; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);
        if (name)
        {
            cout << "actuator_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Sensor ------------->> " << endl;
    int index = 0;
    // 多维传感器，输出第一维的index
    for (int i = 0; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (name)
        {
            cout << "sensor_index: " << index << ", "
                 << "name: " << name << ", "
                 << "dim: " << mj_model_->sensor_dim[i]
                 << endl;
        }
        index = index + mj_model_->sensor_dim[i];
    }
    cout << endl;
};

void UnitreeSdk2Bridge::CheckSensor()
{
    num_motor_ = mj_model_->nu;
    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "imu_quat") == 0)
        {
            have_imu_ = true;
        }
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
    }
}
