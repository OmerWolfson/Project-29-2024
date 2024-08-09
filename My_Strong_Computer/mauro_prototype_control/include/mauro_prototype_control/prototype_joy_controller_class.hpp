#ifndef PROTOTYPE_DRIVER_HPP
#define PROTOTYPE_DRIVER_HPP

#include <cmath>

class SerialCom {
public:
    int pwm_L;
    bool is_reverse_dir_L; // if true - going reverse
    int pwm_R;
    bool is_reverse_dir_R;
    bool G; // if true - high gear
};

class PrototypeControl {
public:
    SerialCom compute_control(double throttle, double steering, bool is_high_gear);

    int SpeedCtrl(double omega, int config);

private:
    const double wheel_r = 0.175; // Wheel's radius [m]
    const double d = 0.485;       // Wheels' distance [m]
    const double max_rad_acc = 50;// Maximum radial acceleration constant(max cent force / m)[m / s^2]
    const double pi = 3.14159265;
    const int thres = 5;          // Threshold for mode switch (%)
};


#endif // PROTOTYPE_DRIVER_HPP
