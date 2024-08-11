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
    int max_pwm_f;		// Maximum PWM output - forward
    int max_pwm_b;		// Maximum PWM output - backward
    int sens_f = 1;		// Static rotation correction
    int sens_b = 1;		// Static rotation correction
        

    // מניח ששתי הכניסות הן ערכים מ- 1שלילי (אחורה או שמאלה) עד ל- 1 חיובי (קדימה או ימינה) ואם לא אז צריכים להתאים את זה

    // Input (ratio) -1:1
    if (abs(throttle) < thres){
        throttle = 0;
    }

    if (abs(steering) < thres){
        steering = 0;
    }

    if (throttle == 0){     // Static rotation
	    if(steering < 0){    // Left
            pwm_L = max_pwm_b * steering * sens_b;
            pwm_R = max_pwm_f * steering * sens_f;
		    is_reverse_dir_L = true;
		    is_reverse_dir_R = false;
	    }
	    else{                // Right
		    pwm_L = max_pwm_b * steering * sens_b;
            pwm_R = max_pwm_f * steering * sens_f;
		    is_reverse_dir_L = false;
		    is_reverse_dir_R = true;
	    }
    }

    else if (throttle > 0){    // Forward
        if (steering < 0){    // Left
            pwm_L = max_pwm_f * throttle * (1 + steering);
            pwm_R = max_pwm_f * throttle;
            }
            else {                 // Right - straight
            pwm_L = max_pwm_f * throttle;
            pwm_R = max_pwm_f * throttle * (1 - steering);
            }
    }

    else if (throttle < 0){    // Backward
	if(steering < 0){      // Left
            pwm_L = max_pwm_b * throttle * (1 + steering);
            pwm_R = max_pwm_b * throttle;
        }
        else {                 // Right - straight
            pwm_L = max_pwm_b * throttle;
            pwm_R = max_pwm_b * throttle * (1 - steering);
        }  
    }

    ser_com.pwm_L = pwm_L;
    ser_com.pwm_R = pwm_R;
    ser_com.is_reverse_dir_L = is_reverse_dir_L;
    ser_com.is_reverse_dir_R = is_reverse_dir_R;
    ser_com.gear = is_high_gear;

    return ser_com;

}