/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RangeFinder/my.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

    // @Param: 2SRV_TCONST
    // @DisplayName: Pitch Time Constant
    // @Description: Time constant in seconds from demanded to achieved pitch angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
    // @Range: 0.4 1.0
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("2SRV_TCONST",      0, AP_PitchController, gains.tau,       0.5f),

    // index 1 to 3 reserved for old PID values

    // @Param: 2SRV_RMAX_UP
    // @DisplayName: Pitch up max rate
    // @Description: This sets the maximum nose up pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_RMAX_UP",     4, AP_PitchController, gains.rmax_pos,   0.0f),

    // @Param: 2SRV_RMAX_DN
    // @DisplayName: Pitch down max rate
    // @Description: This sets the maximum nose down pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_RMAX_DN",     5, AP_PitchController, gains.rmax_neg,   0.0f),

    // @Param: 2SRV_RLL
    // @DisplayName: Roll compensation
    // @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.
    // @Range: 0.7 1.5
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("2SRV_RLL",      6, AP_PitchController, _roll_ff,        1.0f),

    // index 7, 8 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain. Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: _RATE_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: _RATE_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _RATE_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: _RATE_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, AP_PitchController, AC_PID),

    AP_GROUPEND
};

AP_PitchController::AP_PitchController(const AP_FixedWing &parms)
    : aparm(parms)
{
    AP_Param::setup_object_defaults(this, var_info);
    rate_pid.set_slew_limit_scale(45);
}

/*
  AC_PID based rate controller
*/
float AP_PitchController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed,bool ground_mode)
{
    const float dt = AP::scheduler().get_loop_period_s();
    const AP_AHRS &_ahrs = AP::ahrs();
    //低空速修正
    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }
    bool underspeed = aspeed <= float(9);//飞翼
    // bool underspeed = aspeed <= float(4);//冲浪者

    // bool limit_I =fabsf(_last_out)>=45;
    float rate_y = _ahrs.get_gyro().y;
    if(underspeed){
        aoa = 0;
    }else{
        aoa = _ahrs.getAOA();
    }
    if(abs(aoa) <= 0.01){
        aoa = 0;
    }
    const aoassa &myaoa = *AP::Aoassa();
    float my_aoa =0 - myaoa._aoa;
    my_aoa = constrain_float(my_aoa , -15.0F,15.0F);
    // hal.console -> printf("%f\n",my_aoa);



    //nz反馈p
    Vector3f ab = _ahrs.get_accel();//机体坐标系
 
    Vector3f Ve;//计算航迹倾角
    float out_az = 0;
    if(_ahrs.get_velocity_NED(Ve)){
        float qing = atan2(-Ve.z, sqrt(Ve.x * Ve.x + Ve.y * Ve.y));
        float pian = atan2(Ve.y, Ve.x);
        float T[3][3] = {{cosf(qing)*cosf(pian), cosf(qing)*sinf(pian), -sinf(qing)},
                        {-sinf(pian),                cosf(pian),                  0}, 
                        {sinf(qing)*cosf(pian), sinf(pian)*sinf(qing),  cosf(qing)}};//地面系到航迹系转换矩阵
        Vector3f g_g = {0,0,9.81};
        Matrix3f dcm = _ahrs.get_DCM_rotation_body_to_ned();//体轴系到地面系转换矩阵
        Vector3f ab2e = multiplyMatrixVector2(dcm,ab);
        Vector3f a_relative_e = g_g + ab2e;//转换到地面系后，相对惯性系的加速度
        Vector3f a_relative_e2hangji = multiplyMatrixVector(T,a_relative_e);
        // hal.console -> printf("x=%f,y=%f,z=%f\n",a_relative_e2hangji.x, a_relative_e2hangji.y, a_relative_e2hangji.z);
        
        //飞翼不同速度下线化 根轨迹求出的结果
        float boost_az= 2.5;
        if(aspeed >= 9){
            boost_az =2.5-(0.11*(aspeed - 10));
        }
        boost_az = constrain_float(boost_az , 0.1,2.5);

        // //冲浪者不同速度下线化 根轨迹求出的结果
        // float boost_az= 3.88;
        // if(aspeed >= 10){
        //     boost_az =3.88-(0.42*(aspeed - 10));
        // }
        // boost_az = constrain_float(boost_az , 1.7,3.88);

        out_az = boost_az * (a_relative_e2hangji.z);
    }




    //迎角反馈p

    //飞翼不同速度下线化 根轨迹求出的结果
    boost_aoa = 1.66;
    if(aspeed >= 9){
        boost_aoa =1.66-(0.023*(aspeed - 10));
    }
    boost_aoa = constrain_float(boost_aoa , 0.1,1.66);

    // //冲浪者不同速度下线化 根轨迹求出的结果
    // boost_aoa = 0.3;
    // if(aspeed >= 10){
    //     boost_aoa =0.3-(0.024*(aspeed - 10));
    // }
    // boost_aoa = constrain_float(boost_aoa , 0.1,0.3);

    error_aoa =0 - aoa;
    out_aoa = boost_aoa * error_aoa + boost_aoa * my_aoa*0;




    //姿态反馈pid_feiyi
    float boost_pitch_p = 0.2f;
    if(aspeed > 0.0f){
        boost_pitch_p =0.2f - 0.2f/40.0f*((abs(aspeed)));
    }
    boost_pitch_p = constrain_float(boost_pitch_p, 0.005f,0.2f);
    //计算P
    error_my = desired_rate-degrees(rate_y);
    float out_my_p = boost_pitch_p * error_my;

    //计算I
    float ki_pitch = 0.3;
    if(!disable_integrator){
        if(underspeed){
            ki_pitch = 0.0;
        }
    }else{
            ki_pitch = 0.0;
    }
    integral +=  ki_pitch * error_my * dt;
    float I_term =  integral;
    I_term = constrain_float(I_term , -300.0F,300.0F);//如果拉杆起不来就是这个地方小了

    //计算D
    float boost_pitch_d = 0.003f;
    if(aspeed > 0.0f){
        boost_pitch_d =0.003 - 0.003f/40*((abs(aspeed)));
    }
    boost_pitch_d = constrain_float(boost_pitch_d, 0.0005f,0.003f);
    
    derivative = (error_my - previous_error) / dt;
    D_term = boost_pitch_d * derivative;


    //速度修正
    //飞翼
    float k = 0.1;
    if(aspeed > 0.0f){
        k =0.1f - 0.1f/40.0f*((abs(aspeed)));
    }
    k = constrain_float(k, 0.01f,0.1f);

    float out_my = out_my_p + I_term  + D_term  + out_aoa* k + out_az* k;//

    previous_error = error_my;

    if (underspeed) {
        out_my -= D_term + out_my_p * 0.25 + I_term + out_aoa + out_az;
    }

    // 法一飞翼
    float k_out = 1;
    out_my = k_out * out_my;




    // //姿态反馈pid_clz
    // float boost_pitch_p = 0.2f;//飞翼冲浪者0.2太大 高速不行0.1
    // if(aspeed > 0.0f){
    //     // boost_pitch_p =0.2f - 0.1f/30.0f*((abs(aspeed)));//feiyi
    //     boost_pitch_p =0.2f - 0.1f/10.0f*((abs(aspeed)));//clz
    // }
    // boost_pitch_p = constrain_float(boost_pitch_p, 0.02f,0.2f);
    // //计算P
    // error_my = desired_rate-degrees(rate_y);
    // float out_my_p = boost_pitch_p * error_my;

    // //计算I
    // float ki_pitch = 0.3;//飞翼冲浪者0.2 0.05

    // if(!disable_integrator){
    //     if(underspeed){
    //         ki_pitch = 0.0;
    //     }
    // }else{
    //         ki_pitch = 0.0;
    // }

    // // if (!is_zero(ki_pitch) && is_positive(dt)) {
    // //     // Ensure that integrator can only be reduced if the output is saturated
    // //     if (((is_positive(integral) && is_negative(error_my)) || (is_negative(integral) && is_positive(error_my)))) {
    // //         integral += (error_my * ki_pitch) * dt;
    // //         integral = constrain_float(integral, -100.0F,100.0F);
    // //     }
    // // } else {
    // //     integral = 0.0f;
    // // }
    // //     hal.console -> printf("%f\n",integral);

    // integral +=  ki_pitch * error_my * dt;
    // integral = constrain_float(integral, -300.0f,300.0f);//如果拉杆起不来就是这个地方小了

    // float I_term =  integral;

    // //计算D
    // float boost_pitch_d = 0.01f;//飞翼0.05 clz0.01
    // if(aspeed > 0.0f){
    //     // boost_pitch_d =0.03 - 0.005f/30*((abs(aspeed)));//feiyi
    //     boost_pitch_d =0.01 - 0.005f/10*((abs(aspeed)));//clz
    // }
    // // boost_pitch_d = constrain_float(boost_pitch_d, 0.001f,0.03f);//feiyi
    // boost_pitch_d = constrain_float(boost_pitch_d, 0.001f,0.01f);//clz
    // derivative = (error_my - previous_error) / dt;

    // D_term = boost_pitch_d * derivative;//冲浪者0.01


    // //速度修正
    // // //飞翼
    //  // float k_p = 1;
    // // float k_d = 1;
    // // if(aspeed > 15){
    // //     k_d = 1 + (0.1/10*(aspeed - 15));//
    // // }
    // // float k_i = 1;
    // // if(aspeed > 15){
    // //     k_i = 1 + (0.1/10*(aspeed - 15));//
    // // }

    // //冲浪者
    // float k = 0.2f;

    // if(aspeed > 0.0f){
    //     // k =0.2f - 0.1f/30.0f*((abs(aspeed)));//feiyi
    //     k =0.2f - 0.1f/10.0f*((abs(aspeed)));//clz
    // }
    // k = constrain_float(k, 0.02f,0.2f);

    // float out_my = out_my_p  + I_term  + D_term  + out_aoa * k + out_az * k;//

    // previous_error = error_my;

    // if (underspeed) {
    //     out_my -= D_term + out_my_p * 0.25 + I_term + out_aoa + out_az;
    // }

    // //冲浪者
    // float k_out = 1.0f;
    // out_my = k_out * out_my;





    
    // remember the last output to trigger the I limit
    // _last_out = out_my;

    // output is scaled to notional centidegrees of deflection
    return constrain_float( -out_my * 100, -4500, 4500);
}

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are:
 1) demanded pitch rate in degrees/second
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
 5) maximum FBW airspeed (metres/sec)
*/
float AP_PitchController::get_rate_out(float desired_rate, float scaler)
{
    float aspeed;
    if (!AP::ahrs().airspeed_estimate(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    return _get_rate_out(desired_rate, scaler, false, aspeed, false);
}

/*
  get the rate offset in degrees/second needed for pitch in body frame
  to maintain height in a coordinated turn.

  Also returns the inverted flag and the estimated airspeed in m/s for
  use by the rest of the pitch controller
 */
float AP_PitchController::_get_coordination_rate_offset(float &aspeed, bool &inverted) const
{
    float rate_offset;
    float bank_angle = AP::ahrs().get_roll();

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < radians(90))	{
        bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
        inverted = false;
    } else {
        inverted = true;
        if (bank_angle > 0.0f) {
            bank_angle = constrain_float(bank_angle,radians(100),radians(180));
        } else {
            bank_angle = constrain_float(bank_angle,-radians(180),-radians(100));
        }
    }
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.airspeed_estimate(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(_ahrs.get_pitch())*fabsf(ToDeg((GRAVITY_MSS / MAX((aspeed * _ahrs.get_EAS2TAS()), MAX(aparm.airspeed_min, 1))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff;
    }
    if (inverted) {
        rate_offset = -rate_offset;
    }
    return rate_offset;
}

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are:
// 1) demanded pitch angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
// 5) maximum FBW airspeed (metres/sec)
//
float AP_PitchController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode)
{
    // Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
    // Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
    // Pitch rate offset is the component of turn rate about the pitch axis
    float aspeed;
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }


    // 方法一角度误差
    // //冲浪者
    // float k_sp = 0.2f;
    // if(aspeed > 0.0f){
    //     k_sp =0.2f - 0.1f/10.0f*((abs(aspeed)));//
    // }
    // k_sp = constrain_float(k_sp, 0.01f,0.2f);

    // hal.console -> printf("%f\n",k_sp);

    // angle_err_deg = angle_err * 0.01;
    // float desired_rate = angle_err_deg * 40 * k_sp;

    //飞翼
    float k_sp = 0.1f;
    if(aspeed > 0.0f){
        k_sp =0.1f - 0.1f/40.0f*((abs(aspeed)));//
    }
    k_sp = constrain_float(k_sp, 0.005f,0.1f);

    // hal.console -> printf("%f\n",k_sp);

    angle_err_deg = angle_err * 0.01;
    float desired_rate = angle_err_deg * 40 * k_sp;//飞翼40



    return _get_rate_out(desired_rate, scaler, disable_integrator,  aspeed, ground_mode);
}

void AP_PitchController::reset_I()
{
    rate_pid.reset_I();
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_PitchController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured()) {
        return;
    }

    float old_ff=0, old_p=1.0, old_i=0.3, old_d=0.08;
    int16_t old_imax = 3000;
    bool have_old = AP_Param::get_param_by_index(this, 1, AP_PARAM_FLOAT, &old_p);
    have_old |= AP_Param::get_param_by_index(this, 3, AP_PARAM_FLOAT, &old_i);
    have_old |= AP_Param::get_param_by_index(this, 2, AP_PARAM_FLOAT, &old_d);
    have_old |= AP_Param::get_param_by_index(this, 8, AP_PARAM_FLOAT, &old_ff);
    have_old |= AP_Param::get_param_by_index(this, 7, AP_PARAM_FLOAT, &old_imax);
    if (!have_old) {
        // none of the old gains were set
        return;
    }

    const float kp_ff = MAX((old_p - old_i * gains.tau) * gains.tau  - old_d, 0);
    rate_pid.ff().set_and_save(old_ff + kp_ff);
    rate_pid.kI().set_and_save_ifchanged(old_i * gains.tau);
    rate_pid.kP().set_and_save_ifchanged(old_d);
    rate_pid.kD().set_and_save_ifchanged(0);
    rate_pid.kIMAX().set_and_save_ifchanged(old_imax/4500.0);
}

/*
  start an autotune
 */
void AP_PitchController::autotune_start(void)
{
    if (autotune == nullptr) {
        autotune = new AP_AutoTune(gains, AP_AutoTune::AUTOTUNE_PITCH, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed pitch allocation");
            }
            failed_autotune_alloc = true;
        }
    }
    if (autotune != nullptr) {
        autotune->start();
    }
}

/*
  restore autotune gains
 */
void AP_PitchController::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
    }
}

Vector3f AP_PitchController::multiplyMatrixVector(const float matrix[3][3], const Vector3f& vec) {
    Vector3f result;
    result.x = matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z;
    result.y = matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z;
    result.z = matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z;
    return result;
}
