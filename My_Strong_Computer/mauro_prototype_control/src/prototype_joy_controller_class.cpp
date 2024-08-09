#include "mauro_prototype_control/prototype_joy_controller_class.hpp"

SerialCom PrototypeControl::compute_control(double throttle, double steering, bool is_high_gear) 
{
    SerialCom ser_com;

    // Parameters
    double lin_sp;   // Robot's desired linear speed
    double ang_sp;   // Robot's desired angular speed
    double omega_l;  // Left wheel's angular speed
    double omega_r;  // Right wheel's angular speed
    double omega_max;// Maximum angular speed
    double max_sp;   // Maximum linear speed for the is_high_gear

    int pwm_l;       // PWM output left wheel
    bool is_reverse_dir_l;       // is_reverse_direction output left wheel : 0(forward) / 1(backward) -> false (forward) / true (backwards)
    int pwm_r;       // PWM output right wheel
    bool is_reverse_dir_r;       // is_reverse_direction output right wheel : 0(forward) / 1(backward) -> false (forward) / true (backwards)

    // Max linear speed calculation
    if(is_high_gear) { // is_high_gear == true means high gear
        
        if (throttle > thres)
            max_sp = wheel_r * 2 * pi * 29.0; // forward high gear
        else if ((-throttle) > thres)
            max_sp = wheel_r * 2 * pi * 6.16; // reverse high gear
        else if (std::abs(throttle) < thres)
            max_sp = 0; // Static rotation    
        
    }

    else {

        if (throttle > thres)
            max_sp = wheel_r * 2 * pi * 14.5; // forward low gear
        else if ((-throttle) > thres)
            max_sp = wheel_r * 2 * pi * 6.16; // reverse low gear
        else if (std::abs(throttle) < thres)
            max_sp = 0; // Static rotation

    }

    // Desired linear speed
    lin_sp = max_sp * throttle * 0.01;

    // Max angular speed calculation
    if (lin_sp != 0)
        omega_max = max_rad_acc / lin_sp;
    else
        omega_max = 12.32 * wheel_r / d; // Static rotation

    // Desired angular speed ; turning left = steering is positive -> angular spd around z axis = positive
    ang_sp = omega_max * steering * 0.01;

    // Wheels' angular speed
    omega_l = (lin_sp - (d * ang_sp / 2)) / wheel_r; // left wheel rotation speed
    omega_r = (lin_sp + (d * ang_sp / 2)) / wheel_r;

    // Opposite is_reverse_directions spinning
    if (lin_sp != 0) {
        if (omega_l * omega_r < 0) {
            if (omega_l < 0) {
                omega_l = 0;
                omega_r = ang_sp * d / wheel_r;
            } else {
                omega_r = 0;
                omega_l = ang_sp * d / wheel_r;
            }
        }
    }

    // PWM output calculation
    if(is_high_gear) {
        
        if (omega_l < 0)
            pwm_l = SpeedCtrl(std::abs(omega_l), 3);
        else
            pwm_l = SpeedCtrl(omega_l, 1);

        if (omega_r < 0)
            pwm_r = SpeedCtrl(std::abs(omega_r), 3);
        else
            pwm_r = SpeedCtrl(omega_r, 1);
            
    }
    else {

        if (omega_l < 0)
            pwm_l = SpeedCtrl(std::abs(omega_l), 2);
        else
            pwm_l = SpeedCtrl(omega_l, 0);

        if (omega_r < 0)
            pwm_r = SpeedCtrl(std::abs(omega_r), 2);
        else
            pwm_r = SpeedCtrl(omega_r, 0);
            
    }

    if (omega_l < 0)
        is_reverse_dir_l = true;
    else
        is_reverse_dir_l = false;

    if (omega_r < 0)
        is_reverse_dir_r = true;
    else
        is_reverse_dir_r = false;

    ser_com.pwm_A = pwm_l;
    ser_com.pwm_B = pwm_r;
    ser_com.is_reverse_dir_A = is_reverse_dir_l;
    ser_com.is_reverse_dir_B = is_reverse_dir_r;
    ser_com.G = is_high_gear;

    return ser_com;


}

int PrototypeControl::SpeedCtrl(double omega, int config) {
    int pwm_outp;
    if (omega == 0)
        pwm_outp = 0;
    else {
        switch (config) {
            case 0: // forward low
                if (omega <= 14.5)
                    pwm_outp = 7.2374 * omega + 64.5066;
                else
                    pwm_outp = 168;
                break;
            case 1: // forward high
                if (omega <= 29.0)
                    pwm_outp = 3.7325 * omega + 61.2301;
                else
                    pwm_outp = 168;
                break;
            case 2: // Backward low
                if (omega <= 6.16)
                    pwm_outp = 6.921 * omega + 65.9686;
                else
                    pwm_outp = 168;
                break;
            case 3: // Backward high
                if (omega <= 6.16)
                    pwm_outp = 3.6837 * omega + 61.8913;
                else
                    pwm_outp = 168;
                break;
            default:
                pwm_outp = 0;
                break;
        }
    }
    return pwm_outp;
}
