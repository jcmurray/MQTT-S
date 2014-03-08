/*
 * MqttsClientApplication.h
 *
 *                    The MIT License (MIT)
 *
 *               Copyright (c) 2013, Tomoaki YAMAGUCHI
 *                       All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.*
 *
 *
 *  Created on: 2013/06/28
 *    Modofoed: 2013/11/30
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */

#ifndef MQTTSCLIENTAPPLICATION_H_
#define MQTTSCLIENTAPPLICATION_H_

#ifdef ARDUINO


#include <MqttsClient.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define MQ_LED_PIN  13
#define MQ_INT0_PIN 2
#define MQ_SLEEP_PIN 3  // Connect to XBee DTR for hibernation mode
#define MQ_ERROR_RECOVERY_DURATION_ON 8
#define MQ_WAKEUP  0
#define MQ_SLEEP   1
#define MQ_ON      1
#define MQ_OFF     0

#define MQ_WDT_ERR   (B01100000)  // Error Indication time

//#define MQ_WDT_TIME (B01000111)   // 2 Sec

//#define MQ_WDT_TIME (B01100000)     // 4 Sec
 
#define MQ_WDT_TIME (B01100001)   // 8 Sec


#define MQ_MODE_NOSLEEP 0
#define MQ_MODE_SLEEP   1

typedef struct {
	long prevTime;
	long interval;
	int (*callback)(void);
}MQ_TimerTbl;

enum MQ_INT_STATUS{ WAIT, INT0_LL, INT0_WAIT_HL, INT_WDT};

/*======================================
               Class WdTimer
========================================*/
class WdTimer {
public:
	WdTimer(void);
	uint8_t registerCallback(long sec, int (*proc)(void));
	void refleshRegisterTable();
	void start(void);
	void stop(void);
	bool wakeUp(void);

private:	
	MQ_TimerTbl *_timerTbls;
	uint8_t _timerCnt;
};

/*======================================
       Class MqttsClientApplication
========================================*/
class MqttsClientApplication{
public:
	MqttsClientApplication();
	~MqttsClientApplication();
	void registerInt0Callback(void (*callback)());
	void registerWdtCallback(long sec, int (*callback)(void));
	void refleshWdtCallbackTable();
	void setup(const char* clientId, uint16_t baudrate);
	void begin(long baudrate, int serialPortNum = 0);
	void init(const char* clientNameId = "");
	void setKeepAlive(uint16_t msec);
	void setQos(uint8_t level);
	void setWillTopic(MQString* willTopic);
	void setWillMessage(MQString* willMsg);
	void setRetain(bool retain);
	void setClean(bool clean);
	void setClientId(MQString* id);
	void sleepApp();
	void blinkIndicator(int msec);
	void indicatorOn();
	void indicatorOff();
	void sleepXB();
	void wakeupXB();
	void setSleepMode(uint8_t sleepMode);
	
	int registerTopic(MQString* topic);
	int publish(MQString* topic, const char* data, int dataLength);
	int publish(uint16_t predefinedId, const char* data, int dataLength);
	int subscribe(MQString* topic, TopicCallback callback);
	int subscribe(uint16_t predefinedId, TopicCallback callback);
	int unsubscribe(MQString* topic);
	int disconnect(uint16_t duration);

	void startWdt();
	void stopWdt();
	int  exec();
	void setUnixTime(MqttsPublish* msg);
	long getUnixTime();
	void reboot();

private:
	void checkInterupt();
	void interruptHandler();
    void setInterrupt();
	
	MqttsClient _mqtts;
	bool _txFlag;
	long    _unixTime;
	uint32_t _epochTime;
	uint8_t _sleepMode;

	WdTimer _wdTimer;

	void (*_intHandler)(void);

};

extern MqttsClientApplication* theApplication;

#else

#endif /*ARDUINO*/




#endif /* MQTTSCLIENTAPPLICATION_H_ */
