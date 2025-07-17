
// #include <stdint.h>
#include "ComplementaryFilter.h"
#include "TemporalFilter.h"
#include "Logger.h"
#include "mpu6050.hpp"
#include <cmath>
#include <array>


namespace {
    void ReadIMU(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
    {
        MPU6050_Read_All(I2Cx, DataStruct);

        // IMU is Left-Handed, and has X fwd, Y up, Z left
        // Remap to my convention: X fwd, Y left, Z up
        // Also, gyroscope reports -ve angle. Negate all of them.

        // Gyroscope:
        const auto gx = -DataStruct->Gx;
        const auto gy = DataStruct->Gy;
        const auto gz = DataStruct->Gz;
        DataStruct->Gx = gx;
        DataStruct->Gy = gz;
        DataStruct->Gz = gy;

        // Accelerometer:
        const auto ax = DataStruct->Ax;
        const auto ay = -DataStruct->Ay;
        const auto az = -DataStruct->Az;
        DataStruct->Ax = ax;
        DataStruct->Ay = az;
        DataStruct->Az = ay;
    }

    float ToDeg(float rad)
    {
        constexpr float toDeg = 180.0f / M_PI;
        return rad * toDeg;
    }

    float ToRad(float deg)
    {
        constexpr float toRad = M_PI / 180.0f;
        return deg * toRad;
    }
}

ComplementaryFilter::ComplementaryFilter(I2C_HandleTypeDef* mpu) : m_mpuHandle(mpu), m_filter(TemporalFilter<double, 1u>{}), m_prevTick_ms(HAL_GetTick())
{
    // Initialize first-order temporal filter for gyro pitch readings
    const auto biases = std::array<double,1u>{m_config.gyroBiasY};
    m_filter.Initialize(m_config.temporalFilterAlpha, biases);
}

void ComplementaryFilter::Step(){

    const auto tick_ms = HAL_GetTick();
    auto dt_s = (tick_ms - m_prevTick_ms)/1000.0f;

    m_prevTick_ms = tick_ms;

    if (dt_s > 0.1)
    {
        // debugLogFmt("WARNING: CF iteration too long. dt=%.3fs.", dt_s);
        dt_s = 0.1;
    }

    // Get angular velocity wx, wy, wz
    ReadIMU(m_mpuHandle, &m_mpu6050Data);

    auto gyroReadings = std::array<double,1u>{m_mpu6050Data.Gy};
    m_filter.Apply(gyroReadings);

    // Calculate angle from accelerometer
    // Convention is +x,+y,+z --> fwd, left, up
    // Negate z measurement to keep atan2 in the first quadrant (around 0 deg)
    // Angle does not roll over to 360. It can become negative. This is good.
    // For now, use small angle approximation atan2(x,z) ~= x/z
    const auto accelerometerAngle = m_mpu6050Data.Ax/(-m_mpu6050Data.Az);

    // Calculate angle from gyroscope
    const auto gyroAngle = gyroReadings[0] * dt_s + m_pitchAngle;

    // Update angle estimate
    m_pitchAngle = m_config.complementaryFilterAlpha * gyroAngle + (1 - m_config.complementaryFilterAlpha) * accelerometerAngle;
    m_pitchAngleDot = gyroReadings[0];

    logFmt("gyr_x:%.6f\n", m_mpu6050Data.Gx - m_config.gyroBiasX);
    logFmt("gyr_y:%.6f\n", m_mpu6050Data.Gy - m_config.gyroBiasY);
    logFmt("gyr_z:%.6f\n", m_mpu6050Data.Gz - m_config.gyroBiasZ);

    // TODO: Debug: These are being used temporarily to symbilize the accel angle and gyro angle for debugging
    debugLogFmt("acc_x:%.6f\n", ToDeg(accelerometerAngle));
    debugLogFmt("acc_y:%.6f\n", ToDeg(gyroAngle));

    // debugLogFmt("acc_x:%.6f\n", m_mpu6050Data.Ax);
    // debugLogFmt("acc_y:%.6f\n", m_mpu6050Data.Ay);
    // debugLogFmt("acc_z:%.6f\n", m_mpu6050Data.Az);
}

float ComplementaryFilter::GetPitchAngle()
{
    return m_pitchAngle + m_pitchAngleOffset;
}
float ComplementaryFilter::GetPitchAngleDot()
{
    return m_pitchAngleDot;
}