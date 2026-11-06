#pragma once
// BNO055: [0]=X, [1]=Y, [2]=Z unless noted otherwise.
struct BNOData {
    float euler[3];   // Heading, Roll, Pitch (deg)
    float quat[4];    // W, X, Y, Z
    float linAcc[3];  // m/s^2, gravity removed
    float grav[3];    // m/s^2
    float acc[3];     // m/s^2, total
    float gyro[3];    // deg/s
    float mag[3];     // uT
    int8_t temp;      // deg C
    uint8_t cal[4];   // System, Gyro, Accel, Mag (0..3)
};
