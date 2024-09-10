#include "mauro_prototype_control/prototype_joy_controller_class.hpp"

<<<<<<< HEAD
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
=======
SerialCom PrototypeControl::compute_control(double throttle, double steering, bool is_high_gear) 

// אפשר להוריד את ההילוך המפרמטרים, בינתיים משאיר את זה ככה שנוכל להפעיל את זה עם שלט אבל אם נחליט שנבטל את זה אז אפשר למחוק


{
    SerialCom ser_com;


        // לא בטוח א חייבים להיות משתנים מסוג אינטגר אבל מקסימום אפשר לעשות המרה או לשנות את סוג המשתנה
    int pwm_L;       // PWM output left wheel
    bool is_reverse_dir_L;       // is_reverse_direction output left wheel : 0(forward) / 1(backward) -> false (forward) / true (backwards)
    int pwm_R;       // PWM output right wheel
    bool is_reverse_dir_R;       // is_reverse_direction output right wheel : 0(forward) / 1(backward) -> false (forward) / true (backwards)
        // המקסימום לא בהרח 100 ואולי נרצה להגביל את המצערת בשני הכיוונים

    // מניח ששתי הכניסות הן ערכים מ- 1שלילי (אחורה או שמאלה) עד ל- 1 חיובי (קדימה או ימינה) ואם לא אז צריכים להתאים את זה

    // Input (ratio) -1:1
    if (abs(throttle) < THRES){
        throttle = 0;
    }

    // Input (ratio) -1:1 (left:right)
    if (abs(steering) < THRES){
        steering = 0;
>>>>>>> pure_throttle_meth
    }
    is_reverse_dir_L = false; // initialize forward motion
    is_reverse_dir_R = false; // initialize forward motion

    if (throttle == 0){     // Static rotation
<<<<<<< HEAD
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
=======
	    if(steering < 0){    // Left
            pwm_L = MAX_PWM_B * steering * SENS_B; // SENS_B - Static rotation correction; MAX_PWM_B - Maximum PWM output - forward
            pwm_R = MAX_PWM_F * steering * SENS_F; // SENS_F - Static rotation correction; MAX_PWM_F - Maximum PWM output - backward
		    is_reverse_dir_L = true; 
		    is_reverse_dir_R = false;
	    }
	    else{                // Right
		    pwm_L = MAX_PWM_B * steering * SENS_B;
            pwm_R = MAX_PWM_F * steering * SENS_F;
		    is_reverse_dir_L = false;
		    is_reverse_dir_R = true;
	    }
    }

    else if (throttle > 0){    // Forward
        if (steering < 0){    // Left
            pwm_L = MAX_PWM_F * throttle * (1 + steering);
            pwm_R = MAX_PWM_F * throttle;
            }
            else {                 // Right - straight
            pwm_L = MAX_PWM_F * throttle;
            pwm_R = MAX_PWM_F * throttle * (1 - steering);
            }
>>>>>>> pure_throttle_meth
    }

    else if (throttle < 0){    // Backward
	    if(steering < 0){      // Left
            pwm_L = MAX_PWM_B * throttle * (1 + steering);
            pwm_R = MAX_PWM_B * throttle;
            is_reverse_dir_L = true;
            is_reverse_dir_R = true;
        }
        else {                 // Right - straight
            pwm_L = MAX_PWM_B * throttle;
            pwm_R = MAX_PWM_B * throttle * (1 - steering);
            is_reverse_dir_L = true;
            is_reverse_dir_R = true;
        }  
    }

    // --------- Temporary fix for negative PWM, Mauro to your attention ------------------------ 
    if (pwm_L < 0)
        pwm_L = -pwm_L;
    if (pwm_R < 0)
        pwm_R = -pwm_R;

<<<<<<< HEAD
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
=======

    ser_com.pwm_L = pwm_L;
    ser_com.pwm_R = pwm_R;
>>>>>>> pure_throttle_meth
    ser_com.is_reverse_dir_L = is_reverse_dir_L;
    ser_com.is_reverse_dir_R = is_reverse_dir_R;
    ser_com.gear = is_low_gear;

    return ser_com;

}