/*
 * MqttsClientApplication.cpp
 *
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
 *  Created on: 2013/06/17
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */
 
#ifdef ARDUINO
 
#include <MQTTS_Defines.h>
#include <MqttsClientAppFw4Arduino.h>


using namespace std;

volatile uint8_t MQ_ErrFlag = MQ_OFF;
volatile uint8_t MQ_cnt = 0;

MqttsClientApplication* theApplication = NULL;
enum MQ_INT_STATUS MQ_intStat;
enum MQ_INT_STATUS MQ_wdtStat;

#if defined(DEBUG_ZBEESTACK) || defined(DEBUG_MQTTS)
	#include <SoftwareSerial.h>
	extern SoftwareSerial debug;
#endif

/*-------------------------------
 * Set WDT Interrupt procedure
 --------------------------------*/
ISR (WDT_vect) {
	cli();
	MCUSR = 0;
	wdt_reset();
	WDTCSR |= B00011000;   // WDCE:on, WDE:on
	WDTCSR = 0x00;
	MQ_wdtStat = INT_WDT;
	sei();
}

/*-------------------------------
 * INT0 Interrupt procedure
 --------------------------------*/
void MQInt0(){
	detachInterrupt(0);
	MQ_intStat = INT0_LL;
}

/*-------------------------------
 * Set WDT
 --------------------------------*/
void MQwatchdogEnable(){  // Turn on WDT
	cli();
	MCUSR = 0;
	wdt_reset();
	WDTCSR |= B00011000;   // WDCE:on, WDE:on
	WDTCSR = MQ_WDT_TIME_SEC;
	sei();
}

/*--------------------------------
   Dummy function
---------------------------------*/
void MQdummy(){
}

/*========================================
		Class MqttsClientApplication
=========================================*/
MqttsClientApplication::MqttsClientApplication(){
	_txFlag = false;
	_wdtCnt = 0;
	_intHandler = MQdummy;

}

MqttsClientApplication::~MqttsClientApplication(){

}

void MqttsClientApplication::setup(const char* clientId, uint16_t baurate){
	_mqtts.begin(baurate);
	_mqtts.init(clientId);
	pinMode(MQ_INT0_PIN,INPUT_PULLUP);
	pinMode(MQ_SLEEP_PIN, OUTPUT);
	sleepXB();
	MQ_intStat = WAIT;
	MQ_wdtStat = WAIT;
	_wdtCnt = 0;
	setInterrupt();
}


void MqttsClientApplication::registerInt0Callback(void (*callback)()){
  _intHandler = callback;
}


void MqttsClientApplication::registerWdtCallback(double sec, void (*callback)()){
  _wdTimer.registerCallback(sec, callback);
}


void MqttsClientApplication::checkInterupt(){

	// WDT event
	if (MQ_wdtStat == INT_WDT){
		wdtHandler();
	}

	// interrupt Event
	if (MQ_intStat == INT0_LL){
		MQ_intStat = INT0_WAIT_HL;
		interruptHandler();
		setInterrupt();
	}
}

void MqttsClientApplication::wdtHandler(){
	_wdTimer.timeUp();    // Check All callbacks& exec the count up callBack 
    _wdTimer.start();     // Restart WDT
}

void MqttsClientApplication::interruptHandler(){
	_intHandler();
}

void MqttsClientApplication::setInterrupt(){
	if (MQ_intStat == INT0_WAIT_HL){
		while(digitalRead(MQ_INT0_PIN) == 0){
			// wait LL to HL
		}
	}
	MQ_intStat = WAIT;
	attachInterrupt(0,MQInt0,LOW);
}

void MqttsClientApplication::blinkIndicator(int msec){
	digitalWrite(MQ_LED_PIN,MQ_ON);
	delay(msec);
	digitalWrite(MQ_LED_PIN,MQ_OFF);
}

void MqttsClientApplication::sleepXB(){
	digitalWrite(MQ_SLEEP_PIN, MQ_WAKEUP);
}

void MqttsClientApplication::wakeupXB(){
	digitalWrite(MQ_SLEEP_PIN, MQ_SLEEP);
	blinkIndicator(100);
}

void MqttsClientApplication::sleepApp(){
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	wdt_reset();
	MQwatchdogEnable();
	sleep_enable();
	MQ_wdtStat = WAIT;
	sleep_mode();
	sleep_disable();
}

void MqttsClientApplication::begin(long baudrate){
	_mqtts.begin(baudrate);
}

void MqttsClientApplication::init(const char* clientNameId){
	_mqtts.init(clientNameId);
}
	
int MqttsClientApplication::connect(){
	return _mqtts.connect();
}

int MqttsClientApplication::registerTopic(MQString* topic){
	return _mqtts.registerTopic(topic);
}

int MqttsClientApplication::publish(MQString* topic, const char* data, int dataLength){
	return _mqtts.publish(topic, data, dataLength);
}

int MqttsClientApplication::subscribe(MQString* topic, TopicCallback callback){
	return _mqtts.subscribe(topic, callback);
}

int MqttsClientApplication::subscribe(uint16_t predefinedId, TopicCallback callback){
	return _mqtts.subscribe(predefinedId, callback);
}

int MqttsClientApplication::publish(uint16_t predefinedId, const char* data, int dataLength){
	return _mqtts.publish(predefinedId, data, dataLength);
}

int MqttsClientApplication::unsubscribe(MQString* topic){
	return _mqtts.unsubscribe(topic);
}

int MqttsClientApplication::disconnect(uint16_t duration){
	return _mqtts.disconnect(duration);
}

void MqttsClientApplication::setKeepAlive(uint16_t msec){
	_mqtts.setKeepAlive(msec);
}
void MqttsClientApplication::setQos(uint8_t level){
	_mqtts.setQos(level);
}
void MqttsClientApplication::setRetain(bool retain){
	_mqtts.setRetain(retain);
}
void MqttsClientApplication::setClean(bool clean){
	_mqtts.setClean(clean);
}

uint8_t MqttsClientApplication::getMsgRequestType(){
	return _mqtts.getMsgRequestType();
}

uint8_t MqttsClientApplication::getMsgRequestStatus(){
	return _mqtts.getMsgRequestStatus();
}

void MqttsClientApplication::clearMsgRequest(){
	_mqtts.clearMsgRequest();
}

uint8_t MqttsClientApplication::getMsgRequestCount(){
	return _mqtts.getMsgRequestCount();
}

int MqttsClientApplication::execMsgRequest(){
	return _mqtts.execMsgRequest();
}

bool MqttsClientApplication::isGwConnected(){
	return _mqtts.isGwConnected();
}

void MqttsClientApplication::setMsgRequestStatus(uint8_t stat){
	return _mqtts.setMsgRequestStatus(stat);
}

void MqttsClientApplication::startWdt(){
	_wdTimer.start();
}

void MqttsClientApplication::stopWdt(){
	_wdTimer.stop();
}
	
void MqttsClientApplication::run(){
	_mqtts.run();
}

void MqttsClientApplication::runConnect(){
	_mqtts.runConnect();
}

void MqttsClientApplication::runLoop(){
	while(true){
        int rc = execMsgRequest();
		if ((rc != MQTTS_ERR_NO_ERROR || getMsgRequestCount() != 0) && 
			getMsgRequestStatus() != MQTTS_MSG_REQUEST){
			clearMsgRequest();
		}
		checkInterupt();
    }
}

/*======================================
               Class WdTimer
========================================*/
WdTimer::WdTimer(void) {
	_timerTbls = 0;
	_timerCnt = 0;
	_wdtTime = MQ_WDT_TIME_SEC;

}

void WdTimer::start(void) {    
	cli();
	MQ_wdtStat = WAIT;
	MCUSR = 0;
	wdt_reset();
    WDTCSR |= B00011000;     // WDCE:ON, WDE:ON 
    WDTCSR = MQ_WDT_TIME;    
    sei();
}


void WdTimer::stop(void){
	cli();
	MCUSR = 0;
	wdt_reset();
	WDTCSR |= B00011000;   // WDCE:on, WDE:on
	WDTCSR = 0x00;
	MQ_wdtStat = WAIT;
	sei();
}


void WdTimer::timeUp(void){

	for(uint8_t i = 0; i < _timerCnt; i++) {
		if ( _timerTbls[i].cnt-- <= 1 ) {
			(_timerTbls[i].callback)();
			_timerTbls[i].cnt = _timerTbls[i].cntReg;
		}
	}
}


uint8_t WdTimer::registerCallback(double sec, void (*callback)()){
	MQ_TimerTbl *savTbl = _timerTbls;
 	MQ_TimerTbl *newTbl = (MQ_TimerTbl*)calloc(_timerCnt + 1,sizeof(MQ_TimerTbl));

	if ( newTbl != NULL ) {
		_timerTbls = newTbl;
		for(uint8_t i = 0; i < _timerCnt; i++ ){
			_timerTbls[i].cnt = savTbl[i].cnt;
			_timerTbls[i].cntReg = savTbl[i].cntReg;
			_timerTbls[i].callback = savTbl[i].callback;
		}
		free(savTbl);

		_timerTbls[_timerCnt].cnt = (uint16_t)(sec / MQ_WDT_TIME_SEC + 0.5 );
		_timerTbls[_timerCnt].cntReg = _timerTbls[_timerCnt].cnt;
		_timerTbls[_timerCnt].callback = callback;
		_timerCnt++;
		return MQTTS_ERR_NO_ERROR;
	}
	return MQTTS_ERR_OUT_OF_MEMORY;	
} 
#else

#endif  /* ARDUINO */
