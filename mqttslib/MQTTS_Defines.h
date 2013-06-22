#ifndef  MQTTS_DEFINES_H_
#define  MQTTS_DEFINES_H_

/*=================================
 *    Debug Condition
 ==================================*/
#define DEBUG_ZBEESTACK
#define DEBUG_MQTTS

#ifndef ARDUINO
/*=================================
 *    Device Selection
 ==================================*/
#define LINUX
//#define MBED

//#define XBEE_FLOWCTL_CRTSCTS

/*=================================
 *    Data Type
 ==================================*/
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;

#endif   /* ARDUINO */

#endif   /* MQTTS_DEFINES_H_ */
