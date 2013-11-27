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
 *    Debug Condition
 ==================================*/
#define XBEE_DEBUG

/*=================================
 *    print defs
 ==================================*/
#ifdef XBEE_DEBUG
#define DPRINTF(...)  printf(__VA_ARGS__)
#else
#define DPRINTF(...)
#endif

#define DPRINT(...)
#define DPRINTLN(...)

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
