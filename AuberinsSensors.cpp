#include "Globals.h"

// Function to calculate pressure from voltage
int calculatePressure5BAR(float mV) {
    const float offset = 0.5;
    const float sensitivity = 0.0533;
    float bar (((mV / 1000) - offset) / sensitivity);
    return int(bar * INT_SCALING);
};

int calculatePressure7BAR(float mV) {
    const float offset = 0.5;
    const float sensitivity = 0.04;
    float bar (((mV / 1000) - offset) / sensitivity);
    return int(bar * INT_SCALING);
};

