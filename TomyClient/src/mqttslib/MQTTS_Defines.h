#ifndef  MQTTS_DEFINES_H_
#define  MQTTS_DEFINES_H_

#ifndef ARDUINO
/*=================================
 * Device Type Default LINUX
 ==================================*/
#define LINUX
//#define MBED

//#define XBEE_FLOWCTL_CRTSCTS

/*=================================
 *      Debug Condition
 ==================================*/
//#define XBEE_DEBUG
//#define MQTT_DEBUG

/*=================================
 *      print defs
 ==================================*/
#ifdef XBEE_DEBUG
#define D_ZBSTACKF(...)    printf(__VA_ARGS__)
#define D_ZBSTACKW(...)   printf(__VA_ARGS__)
#define D_ZBSTACKLN(...)
#define D_ZBSTACK(...)
#else
#define D_ZBSTACK(...)
#define D_ZBSTACKLN(...)
#define D_ZBSTACKW(...)
#define D_ZBSTACKF(...)
#endif

#ifdef MQTT_DEBUG
#define D_MQTTF(...)    printf(__VA_ARGS__)
#define D_MQTTW(...)   printf(__VA_ARGS__)
#define D_MQTTLN(...)
#define D_MQTT(...)
#else
#define D_MQTT(...)
#define D_MQTTLN(...)
#define D_MQTTW(...)
#define D_MQTTF(...)
#endif

/*=================================
 *    Data Type
 ==================================*/
#ifndef MBED
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;
#endif /* MBED */


#endif /* ARDUINO */

#endif   /* MQTTS_DEFINES_H_ */
