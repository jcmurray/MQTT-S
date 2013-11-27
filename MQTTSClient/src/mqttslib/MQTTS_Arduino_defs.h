#ifndef  MQTTS_ARDUINO_DEFS_H_
#define  MQTTS_ARDUINO_DEFS_H_

#ifdef ARDUINO
/*=================================
 *    Debug Condition
 ==================================*/
#define XBEE_DEBUG

/*=================================
 *    Print defs for DEBUG
 ==================================*/
#ifdef XBEE_DEBUG
#define DPRINT(...)    debug.print(__VA_ARGS__)
#define DPRINTLN(... ) debug.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

#define DPRINTF(...)

#endif /* ARDUINO */
#endif /* MQTTS_ARDUINO_DEFS_H_*/
