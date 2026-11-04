#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Local acquisition/CAN validation: no modem startup, transmission or uplink
 * queueing. CAN routing, sensor acquisition and debug logging are independent. */
#ifndef APP_UPLINK_ENABLED
#define APP_UPLINK_ENABLED 0U
#endif

#endif
