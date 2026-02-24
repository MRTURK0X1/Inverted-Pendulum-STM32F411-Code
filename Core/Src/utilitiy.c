#include "utility.h"
#include "math.h"

float wrap_to_pi(float x) {
    while (x > PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;
    return x;
}



float utility_sin(float x) {
    x = wrap_to_pi(x);
    float x2 = x * x;

    return x * (1
                - x2 / 6.0f
                + x2 * x2 / 120.0f
                - x2 * x2 * x2 / 5040.0f
                + x2 * x2 * x2 * x2 / 362880.0f);
}

float utility_cos(float x) {
    x = wrap_to_pi(x);
    float x2 = x * x;

    return 1
           - x2 / 2.0f
           + x2 * x2 / 24.0f
           - x2 * x2 * x2 / 720.0f
           + x2 * x2 * x2 * x2 / 40320.0f;
}


float v_12_saturate(float value) {
    if (value > 12) return 12;
    if (value < -12) return -12;
    return value;
}

int16_t v_12_to_arr(float v_12){
	static const int16_t arr_max = 6000;
	static const float max_voltage_ref = 12;
    if (fabs(v_12) < 0.1f) {
        return 0;
    }

	return arr_max*v_12/max_voltage_ref;
}
