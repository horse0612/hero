#pragma once

// 0: normal, 1: gyro calibration, 2: accelerometer calibration.
#define IMU_CALIBRATION_MODE 0

#if IMU_CALIBRATION_MODE < 0 || IMU_CALIBRATION_MODE > 2
#error Invalid IMU_CALIBRATION_MODE
#endif
