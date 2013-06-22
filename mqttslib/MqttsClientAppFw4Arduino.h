/*
 * MqttsClientApplication.h
*               Copyright (c) 2013 Tomoaki YAMAGUCHI  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  Created on: 2013/06/15
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.4.0
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

//#define MQ_WDT_TIME  (B01000110)  // 1 Sec
//#define MQ_WDT_TIME_SEC   1       // 1 Sec

#define MQ_WDT_TIME (B01000111)   // 2 Sec
#define MQ_WDT_TIME_SEC   2       // 2 Sec

//#define MQ_WDT_TIME (B01100000)     // 4 Sec
//#define MQ_WDT_TIME_SEC   4         // 4 Sec  
 
//#define MQ_WDT_TIME (B01100001)   // 8 Sec
//#define MQ_WDT_TIME_SEC   9       // 8 Sec


#define MQ_WAKEUP_COUNT   8

typedef struct {
	uint16_t cnt;
	uint16_t cntReg;
	bool flg;
	void (*callback)(void);;
}MQ_TimerTbl;

enum MQ_INT_STATUS{ WAIT, INT0_LL, INT0_WAIT_HL, INT_WDT};

/*======================================
               Class WdTimer
========================================*/
class WdTimer {
public:
	WdTimer(void);
	uint8_t registerCallback(double sec, void (*proc)());
	void start(void);
	void stop(void);
	void timeUp(void);

private:	
	MQ_TimerTbl *_timerTbls;
	uint8_t _timerCnt;
	uint8_t _wdtTime;
	//uint16_t _resolutionMilisec;
};

/*======================================
       Class MqttsClientApplication
========================================*/
class MqttsClientApplication{
public:
	MqttsClientApplication();
	~MqttsClientApplication();
	void registerInt0Callback(void (*callback)());
	void registerWdtCallback(double sec, void (*callback)());
	void setup(const char* clientId, uint16_t baudrate);
	void begin(long baudrate);
	void init(const char* clientNameId);
	void setKeepAlive(uint16_t msec);
	void setQos(uint8_t level);
	void setWillTopic(MQString* willTopic);
	void setWillMessage(MQString* willMsg);
	void setRetain(bool retain);
	void setClean(bool clean);
	void setClientId(MQString* id);
	void sleepApp();
	void blinkIndicator(int msec);
	void sleepXB();
	void wakeupXB();
	
	int connect();
	int registerTopic(MQString* topic);
	int publish(MQString* topic, const char* data, int dataLength);
	int publish(uint16_t predefinedId, const char* data, int dataLength);
	int subscribe(MQString* topic, TopicCallback callback);
	int subscribe(uint16_t predefinedId, TopicCallback callback);
	int unsubscribe(MQString* topic);
	int disconnect(uint16_t duration);

	void startWdt();
	void stopWdt();
	void run();
	void runConnect();
	void runLoop();

private:
	uint8_t getMsgRequestType();
	uint8_t getMsgRequestStatus();
	uint8_t getMsgRequestCount();
	void clearMsgRequest();
	int  execMsgRequest();
	bool isGwConnected();
	void setMsgRequestStatus(uint8_t stat);
	void checkInterupt();
	void wdtHandler();
	void interruptHandler();
    void setInterrupt();
	
	MqttsClient _mqtts;
	bool _txFlag;
	uint8_t _wdtCnt;
	uint8_t _wakeupCnt;
	uint8_t _wakeupCntReg; 
	WdTimer _wdTimer;

	void (*_intHandler)(void);

};

extern MqttsClientApplication* theApplication;

#else

#endif /*ARDUINO*/




#endif /* MQTTSCLIENTAPPLICATION_H_ */
