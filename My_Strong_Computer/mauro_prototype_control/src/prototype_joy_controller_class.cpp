#include "mauro_prototype_control/prototype_joy_controller_class.hpp"

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
    }
    is_reverse_dir_L = false; // initialize forward motion
    is_reverse_dir_R = false; // initialize forward motion

    if (throttle == 0){     // Static rotation
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


    ser_com.pwm_L = pwm_L;
    ser_com.pwm_R = pwm_R;
    ser_com.is_reverse_dir_L = is_reverse_dir_L;
    ser_com.is_reverse_dir_R = is_reverse_dir_R;
    ser_com.gear = is_high_gear;

    return ser_com;

}