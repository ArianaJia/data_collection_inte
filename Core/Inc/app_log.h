#ifndef APP_LOG_H
#define APP_LOG_H
#include "can.h"
#define APP_DEBUG_LOG_ENABLED 1U
#define APP_MODEM_LOG_ENABLED 0U
#ifdef __cplusplus
extern "C" {
#endif
/* Task context only; queue full => drop, never block a producer. */
void App_DebugLogString(const char *text);
/* Formats into module-owned storage; drops on contention, no large task buffer. */
void App_LogPrintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void App_LogProcess(void);
void App_LogModem(const char *text);
void App_LogBootImmediate(const char *text);
void app_can_debug_log_rx_context(const char *prefix, const CAN_ControllerContext_t *ctx);
#ifdef __cplusplus
}
#endif
#endif
