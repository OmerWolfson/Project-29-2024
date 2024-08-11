#include "mauro_prototype_control/prototype_joy_controller_class.hpp"

SerialCom PrototypeControl::compute_control(double throttle, double steering, bool is_low_gear) 
{
    SerialCom ser_com;

    int pwm_L;              // PWM output left wheel
    bool is_reverse_dir_L;  // is_reverse_direction output left wheel : 0(forward) / 1(backward) -> false (forward) / true (backwards)
    int pwm_R;              // PWM output right wheel
    bool is_reverse_dir_R;  // is_reverse_direction output right wheel : 0(forward) / 1(backward) -> false (forward) / true (backwards)
    double omega_L;
    double omega_R;
        
    // Input (ratio) -1:1
    if (abs(throttle) < thres){
        throttle = 0;
    }

    if (abs(steering) < thres){
        steering = 0;
    }

    if(throttle < 0){     // Reverse correction
        throttle = throttle * (1 / thr_sens);
    }

    if (throttle == 0){     // Static rotation
        steering = steering * 1.6;    // Static rotation correcion

        if(steering < 0){    // Right
            omega_L = 6.16 * steering;
            omega_R = -6.16 * steering;
        }
        else{                // Left
            omega_L = -6.16 * steering;
            omega_R = 6.16 * steering;
        }
    }

    else if (throttle > 0){    // Forward
        if (is_low_gear==true){   // Forward low
        
            if(steering == 0){      // Straight
            omega_L = 15.56 * throttle;
            omega_R = 15.56 * throttle;
            }
            else if (steering < 0){  // Right
            omega_L = 15.56 * throttle * (1 + steering);
            omega_R = 15.56 * throttle;
            }
            else if (steering > 0) { // Left
            omega_L = 15.56 * throttle;
            omega_R = 15.56 * throttle * (1 - steering);
            }

        }
        else{    // Forward high

            if(steering == 0){        // Straight
            omega_L = 31.35 * throttle;
            omega_R = 31.35 * throttle;
            }
                if (steering < 0){     // Right
            omega_L = 31.35 * throttle * (1 + steering);
            omega_R = 31.35 * throttle;
            }
            else if (steering > 0) {   // Left
            omega_L = 31.35 * throttle;
            omega_R = 31.35 * throttle * (1 - steering);
            }

        }
    }

    else if (throttle < 0){    // Backward

        if(steering == 0){        // Straight
            omega_L = 6.16 * throttle;
            omega_R = 6.16 * throttle;
        }
        else if(steering < 0){      // Left
            omega_L = 6.16 * throttle * (1 + steering);
            omega_R = 6.16 * throttle;
        }
        else {                 // Right
            omega_L = 6.16 * throttle;
            omega_R = 6.16 * throttle * (1 - steering);
        }  

    }


    if (is_low_gear==true){               // Low gear

        if (omega_L == 0){
            pwm_L = 0;
        }
        else if (omega_L < 0){
            pwm_L = SpeedCtrl(abs(omega_L), 2); // Backward low
        }
        else if (omega_L > 0){
            pwm_L = SpeedCtrl(omega_L, 0);      // Forward low
        }

        if (omega_R == 0){
            pwm_R = 0;
        }
        else if (omega_R < 0){
            pwm_R = SpeedCtrl(abs(omega_R), 2); // Backward low
        }
        else if (omega_R > 0){
            pwm_R = SpeedCtrl(omega_R, 0);      // Forward low
        }

    }
    else if (is_low_gear==false){            //High gear

        if(omega_L == 0){
            pwm_L = 0;
        }
        else if (omega_L < 0){
            pwm_L = SpeedCtrl(abs(omega_L), 3); // Backward high
        }
        else if (omega_L > 0){
            pwm_L = SpeedCtrl(omega_L, 1);      // Forward high
        }

        if(omega_R == 0){
            pwm_R = 0;
        }
        else if (omega_R < 0){
            pwm_R = SpeedCtrl(abs(omega_R), 3); // Backward high
        }
        else if (omega_R > 0){
            pwm_R = SpeedCtrl(omega_R, 1);      // Forward high
        }

    }

    if ( omega_L < 0){
    is_reverse_dir_L = true;
    }
    else{
    is_reverse_dir_L = false;
    }

    if ( omega_R < 0){
    is_reverse_dir_R = true;
    }
    else{
    is_reverse_dir_R = false;
    }
        

    ser_com.pwm_L = pwm_L*100/255;
    ser_com.pwm_R = pwm_R*100/255;
    ser_com.is_reverse_dir_L = is_reverse_dir_L;
    ser_com.is_reverse_dir_R = is_reverse_dir_R;
    ser_com.gear = is_low_gear;

    return ser_com;

}

int PrototypeControl::SpeedCtrl(double omega, int config) {
    int pwm_outp;
    if (omega == 0)
        pwm_outp = 0;
    else {
        switch (config) {
        case 0: // Forward low
            if (omega <= 15.56){
                pwm_outp = int( 7.2374 * omega + 64.5066 );
                break;
            }
            else{
                pwm_outp = 178;
                break;
            }
        case 1: // Forward high
            if (omega <= 31.35){
                pwm_outp = int( 3.7325 * omega + 61.2301 );
                break;
            }
            else{
                pwm_outp = 178;
                break;
            }
        case 2: // Backward low
            if (omega <= 6.4){
                pwm_outp = int(( 6.921 * omega + 65.9686 ) * 1.4);
                break;
            }
            else{
                pwm_outp = int(( 6.921 * 6.4 + 65.9686 ) * 1.4);
                break;
            }
        case 3: // Backward high
            if (omega <= 6.4){
                pwm_outp = int(( 3.6837 * omega + 61.8913 ) * 1.4);
                break;
            }
            else{
                pwm_outp = int(( 3.6837 * 6.4 + 61.8913 ) * 1.4);
                break;
            }
        }
    }
    return pwm_outp;
}