#include "Logger.h"
#include "peripheral_config.h"
#include "string.h"
#include <stdarg.h>
#include <stdio.h>

void debugLog(const char* str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
}

void debugLogFmt(const char* format, ...)
{
    if (false)
        return;

    va_list args;
    va_start(args, format);

    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    const auto len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 10);

    va_end(args);
}

void logFmt(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    const auto len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 10);

    va_end(args);
}