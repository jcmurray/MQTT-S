#ifndef  MQTTS_ARDUINO_DEFS_H_
#define  MQTTS_ARDUINO_DEFS_H_

#ifdef ARDUINO

/*=================================
 *    Debug Condition
 ==================================*/
//#define XBEE_DEBUG
//#define MQTT_DEBUG

/*=================================
 *    Print Defs for DEBUG
 ==================================*/
#ifdef XBEE_DEBUG
#define D_ZBSTACK(...)    debug.print(__VA_ARGS__)
#define D_ZBSTACKLN(... ) debug.println(__VA_ARGS__)
#define D_ZBSTACKW(...)   debug.print(__VA_ARGS__)
#define D_ZBSTACKF(...)
#else
#define D_ZBSTACK(...)
#define D_ZBSTACKLN(...)
#define D_ZBSTACKW(...)
#define D_ZBSTACKF(...)
#endif

#ifdef MQTT_DEBUG
#define D_MQTT(...)    debug.print(__VA_ARGS__)
#define D_MQTTW(...)   debug.print(__VA_ARGS__)
#define D_MQTTLN(...)  debug.println(__VA_ARGS__)
#define D_MQTTF(...)
#else
#define D_MQTT(...)
#define D_MQTTLN(...)
#define D_MQTTW(...)
#define D_MQTTF(...)
#endif

#endif /* ARDUINO */
#endif /* MQTTS_ARDUINO_DEFS_H_*/
