#include "RemoteControl.hpp"

#if USE_REMOTE_CONTROL

#include "PID.hpp"
#include "DR16.hpp"
#include "DJIMotor.hpp"

namespace RemoteControl{

chassis_move chassiss_data;
float k_stick = 1 / 660;
float k_v = 0.7;
void classis_set_update()
{
    DR16::DR16Data *data = DR16::getDR16Data();

    chassiss_data.mode = data->rc.sw.s2;
    chassiss_data.set_vx = (data->rc.stick.ch3 - 1024U) * k_stick;
    chassiss_data.set_vy = (data->rc.stick.ch2 - 1024U) * k_stick;
    chassiss_data.set_wz = (data->rc.stick.ch0 - 1024U) * k_stick;

    chassiss_data.set_vx *= k_v;
    chassiss_data.set_vy *= k_v;
    chassiss_data.set_wz *= (1.0 - k_v);

    return;
}

void forward_kinematics()
{
    //正数是往车头方向转（现在是意淫写的，到时根据实际调整）
    chassiss_data.vm1 = chassiss_data.set_vx - chassiss_data.set_vy - chassiss_data.set_wz;
    chassiss_data.vm2 = -chassiss_data.set_vx - chassiss_data.set_vy - chassiss_data.set_wz;
    chassiss_data.vm3 = -chassiss_data.set_vx + chassiss_data.set_vy - chassiss_data.set_wz;
    chassiss_data.vm4 = chassiss_data.set_vx + chassiss_data.set_vy - chassiss_data.set_wz;

    chassiss_data.vm1 *= MAX_RPM;
    chassiss_data.vm2 *= MAX_RPM;
    chassiss_data.vm3 *= MAX_RPM;
    chassiss_data.vm4 *= MAX_RPM;

    return;
}

const DJIMotor::DJIMotor *m1_measure = DJIMotor::get_motor_measure_point(DJIMotor::CAN_3508_M1_ID);
const DJIMotor::DJIMotor *m2_measure = DJIMotor::get_motor_measure_point(DJIMotor::CAN_3508_M2_ID);
const DJIMotor::DJIMotor *m3_measure = DJIMotor::get_motor_measure_point(DJIMotor::CAN_3508_M3_ID);
const DJIMotor::DJIMotor *m4_measure = DJIMotor::get_motor_measure_point(DJIMotor::CAN_3508_M4_ID);

static Control::PID m1(chassis_Kp , chassis_Ki,
                chassis_Kd, chassis_max_iout, chassis_max_out);
static Control::PID m2(chassis_Kp , chassis_Ki,
                chassis_Kd, chassis_max_iout, chassis_max_out);
static Control::PID m3(chassis_Kp , chassis_Ki,
                chassis_Kd, chassis_max_iout, chassis_max_out);
static Control::PID m4(chassis_Kp , chassis_Ki,
                chassis_Kd, chassis_max_iout, chassis_max_out);

void classis_pid_calculate()
{
    chassiss_data.set_m1_current = m1.update(chassiss_data.vm1, m1_measure->speed_rpm, 0.001f);
    chassiss_data.set_m2_current = m2.update(chassiss_data.vm2, m2_measure->speed_rpm, 0.001f);
    chassiss_data.set_m3_current = m3.update(chassiss_data.vm3, m3_measure->speed_rpm, 0.001f);
    chassiss_data.set_m4_current = m4.update(chassiss_data.vm4, m4_measure->speed_rpm, 0.001f);
    
    float k_mode = chassiss_data.mode == NORMAL ? 1.0 : 0.3;

    DJIMotor::CAN_cmd_chassis(chassiss_data.set_m1_current * k_mode, chassiss_data.set_m2_current * k_mode,
                chassiss_data.set_m3_current * k_mode, chassiss_data.set_m4_current * k_mode);

    return;
}
    
}
#endif