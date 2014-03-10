/*
 * MqttsClientApplication.cpp
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
 *    Modified: 2013/11/30
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */

#ifdef ARDUINO
 
#include <MQTTS_Arduino_defs.h>
#include <MqttsClientAppFw4Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>

using namespace std;
using namespace tomyClient;

volatile uint8_t MQ_ErrFlag = MQ_OFF;
volatile uint8_t MQ_cnt = 0;

MqttsClientApplication* theApplication = NULL;

enum MQ_INT_STATUS MQ_intStat;
enum MQ_INT_STATUS MQ_wdtStat;

#if defined(MQTT_DEBUG) || defined(ZBEE_DEBUG)
	#include <SoftwareSerial.h>
	extern SoftwareSerial debug;
#endif

extern long int getLong(uint8_t* pos);

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
	WDTCSR = MQ_WDT_TIME;
	MQ_wdtStat = WAIT;
	sei();
}

/*--------------------------------
   Dummy function
---------------------------------*/
void IntHandleDummy(){
}

/*--------------------------------
        reset Arduino
---------------------------------*/
void (*resetArduino)(void) = 0;


/*========================================
		Class MqttsClientApplication
=========================================*/
MqttsClientApplication::MqttsClientApplication(){
    _txFlag = false;
    _intHandler = IntHandleDummy;
    _unixTime = 0;
    _epochTime = 0;
    _sleepFlg = false;
    _deviceType = ZB_ROUTER_DEVICE;
}

MqttsClientApplication::~MqttsClientApplication(){

}


void MqttsClientApplication::startWdt(){
    _wdTimer.start();
}

void MqttsClientApplication::stopWdt(){
    _wdTimer.stop();
}

/*------------ Client execution --------------*/
int MqttsClientApplication::exec(){
	wakeupXB();
	checkInterupt();   // WDT routine was executed here

	int rc = _mqtts.exec();
	if(rc == MQTTS_ERR_NO_ERROR){
		sleepXB();
		sleepApp();
	}
	return rc;
}


/*----------- Sleep related functions ------------*/
void MqttsClientApplication::sleepApp(){
	if(_deviceType == ZB_PIN_HIBERNATE && _sleepFlg){
		set_sleep_mode(SLEEP_MODE_PWR_SAVE);
		MQwatchdogEnable();
		sleep_enable();
		MQ_wdtStat = WAIT;
		sleep_mode();      // waiting WDT interrupt
		sleep_disable();
	}
}

void MqttsClientApplication::sleepXB(){
	if(_deviceType == ZB_PIN_HIBERNATE && _sleepFlg){
		pinMode(MQ_SLEEP_PIN, INPUT);
		digitalWrite(MQ_SLEEP_PIN, HIGH);
		indicatorOn();
	}
}

void MqttsClientApplication::wakeupXB(){
	if(_deviceType == ZB_PIN_HIBERNATE){
		pinMode(MQ_SLEEP_PIN, OUTPUT);
		digitalWrite(MQ_SLEEP_PIN, LOW);
		indicatorOff();
	}
}

void MqttsClientApplication::setSleepMode(){
	if(_deviceType == ZB_PIN_HIBERNATE){
		_sleepFlg = true;
		MQ_intStat = WAIT;
		MQ_wdtStat = WAIT;
		setInterrupt();
	}else{
		_sleepFlg = false;
	}
}

void MqttsClientApplication::setZBPinHibernate(){
	_deviceType = ZB_PIN_HIBERNATE;
	pinMode(MQ_INT0_PIN,INPUT_PULLUP);
	wakeupXB();
}

/*-------------- Interrupt related functions ------*/
void MqttsClientApplication::checkInterupt(){

    // interrupt Event
    if (MQ_intStat == INT0_LL){
        MQ_intStat = INT0_WAIT_HL;
        interruptHandler();
        setInterrupt();
    }
    // WDT event
    if (MQ_wdtStat == INT_WDT){
    	D_MQTTLN("WDT wakeup");  //DEBUG
        _wdTimer.wakeUp();    // Check Callback's intervals & execute
        _wdTimer.start();     // WDT restart
    }
}

void MqttsClientApplication::interruptHandler(){
    wakeupXB();
    _intHandler();
    sleepXB();
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


/*--------------------  MQTT-SN functions ---------------*/
void MqttsClientApplication::begin(long baudrate, int serialPortNum){
    _mqtts.begin(baudrate, serialPortNum);
}

void MqttsClientApplication::init(const char* clientNameId){
    _mqtts.init(clientNameId);
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

void MqttsClientApplication::setKeepAlive(uint16_t sec){
    _mqtts.setKeepAlive(sec);
}
void MqttsClientApplication::setQos(uint8_t level){
    _mqtts.setQos(level);
}

void MqttsClientApplication::setWillTopic(MQString* willTopic){
    _mqtts.setWillTopic(willTopic);
}

void MqttsClientApplication::setWillMessage(MQString* willMsg){
    _mqtts.setWillMessage(willMsg);
}

void MqttsClientApplication::setRetain(bool retain){
    _mqtts.setRetain(retain);
}

void MqttsClientApplication::setClean(bool clean){
    _mqtts.setClean(clean);
}


/*------------- UTC functions -------------*/
void MqttsClientApplication::setUnixTime(MqttsPublish* msg){
    _epochTime = millis();
    _unixTime = getLong(msg->getData());
}

long MqttsClientApplication::getUnixTime(){
    uint32_t tm = millis();
    if (_epochTime > tm ){
        return _unixTime + long(0xffffffff - tm - _epochTime) / 1000;
    }else{
        return _unixTime + long(tm - _epochTime) / 1000;
    }
}

void MqttsClientApplication::reboot(){
	resetArduino();
}

/*-------------- Indicator --------------*/
void MqttsClientApplication::indicatorOn(){
    digitalWrite(MQ_LED_PIN,MQ_ON);
}

void MqttsClientApplication::indicatorOff(){
    digitalWrite(MQ_LED_PIN,MQ_OFF);
}

void MqttsClientApplication::blinkIndicator(int msec){
    digitalWrite(MQ_LED_PIN,MQ_ON);
    delay(msec);
    digitalWrite(MQ_LED_PIN,MQ_OFF);
}


/*-------------- Callback related functions ---------------*/
void MqttsClientApplication::registerInt0Callback(void (*callback)()){
    _intHandler = callback;
}


void MqttsClientApplication::registerWdtCallback(long sec, int (*callback)()){
    _wdTimer.registerCallback(sec, callback);
}

void MqttsClientApplication::refleshWdtCallbackTable(){
  _wdTimer.refleshRegisterTable();
}

/*======================================
               Class WdTimer
========================================*/
WdTimer::WdTimer(void) {
    _timerTbls = 0;
    _timerCnt = 0;

}

void WdTimer::start(void) {    
	MQwatchdogEnable();
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


bool WdTimer::wakeUp(void){
    bool rcflg = false;
    for(uint8_t i = 0; i < _timerCnt; i++) {
        //if ((_timerTbls[i].prevTime + _timerTbls[i].interval < theApplication->getUnixTime())){
            int rc = (_timerTbls[i].callback)();
            if(rc == MQTTS_ERR_REBOOT_REQUIRED || rc == MQTTS_ERR_INVALID_TOPICID){
            	resetArduino();
            }
            _timerTbls[i].prevTime = theApplication->getUnixTime();
            rcflg = true;
        //}
    }
    return rcflg;
}


uint8_t WdTimer::registerCallback(long sec, int (*callback)(void)){
    MQ_TimerTbl *savTbl = _timerTbls;
    MQ_TimerTbl *newTbl = (MQ_TimerTbl*)calloc(_timerCnt + 1,sizeof(MQ_TimerTbl));

    if ( newTbl != NULL ) {
        _timerTbls = newTbl;
        for(uint8_t i = 0; i < _timerCnt; i++ ){
			_timerTbls[i].prevTime = savTbl[i].prevTime;
			_timerTbls[i].interval = savTbl[i].interval;
			_timerTbls[i].callback = savTbl[i].callback;
        }
        free(savTbl);

        _timerTbls[_timerCnt].prevTime = theApplication->getUnixTime();
        _timerTbls[_timerCnt].interval = sec;
        _timerTbls[_timerCnt].callback = callback;
        _timerCnt++;
        return MQTTS_ERR_NO_ERROR;
    }
    return MQTTS_ERR_OUT_OF_MEMORY;
} 

void WdTimer::refleshRegisterTable(){
    for(uint8_t i = 0; i < _timerCnt; i++) {
        _timerTbls[i].prevTime = theApplication->getUnixTime();
    }
}


#endif  /* ARDUINO */
