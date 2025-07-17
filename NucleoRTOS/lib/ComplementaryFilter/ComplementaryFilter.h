#pragma once

#ifndef _CF_H_
#define _CF_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "TemporalFilter.h"
#include "mpu6050.hpp"
#include <cmath>


class ComplementaryFilter{
    public:
        ComplementaryFilter(I2C_HandleTypeDef* mpu);
        void Step();
        float GetPitchAngle();
        float GetPitchAngleDot();

    private:

        struct CFConfig
        {
            // Variances
            double rAcc = 0.02f;
            double rGyro = 0.0016f;

            double complementaryFilterAlpha = 0.99f;
            double temporalFilterAlpha = 0.9;
            double gyroBiasX = -0.016387442468f;
            double gyroBiasY = 0.022382848249f;
            double gyroBiasZ = 0.03170903502f;
        };

        I2C_HandleTypeDef* m_mpuHandle;

        MPU6050_t m_mpu6050Data;
        TemporalFilter<double, 1u> m_filter;
        uint32_t m_prevTick_ms;

        CFConfig m_config;

        float m_pitchAngle = 0.0f;
        float m_pitchAngleDot = 0.0f;

        float m_pitchAngleOffset = -4.9f * M_PI / 180.0f;
};


#endif /* _CF_H_ */