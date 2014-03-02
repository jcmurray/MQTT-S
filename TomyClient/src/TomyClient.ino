/*
 * MqttsClientFwApp.ino
 *
 *               Copyright (c) 2013 tomy-tech.com  All rights reserved.
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
 * THE SOFTWARE.
 *
 *  Created on: 2013/11/30
      Modified: 2013/12/12
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */

#include <MQTTS_Arduino_defs.h>
#include <MqttsClientAppFw4Arduino.h>

#if defined(MQTT_DEBUG) || defined(XBEE_DEBUG)
    #include <SoftwareSerial.h>
    SoftwareSerial debug(8, 9);
#endif


/* ----------  Create Application ----------*/
MqttsClientApplication app = MqttsClientApplication();

/*-----------  Callback of subscribe to set Current Time ----------*/

int  setTime(MqttsPublish* msg){
  app.setUnixTime(msg);
  return 0;
}

int  blinkIndicator(MqttsPublish* msg){
  if( strncmp("on", (const char*)msg->getData(),2)){
    app.indicatorOn();
  }else if( strncmp("off", (const char*)msg->getData(),3)){
    app.indicatorOff();
  }
  return 0;
}


/*----------- Create Topics -----------------*/
MQString* tp1 = new MQString("dev/indicator");

/*----------  Functions for WDT interuption -------*/

int wdtFunc0(){
  char payload[4];
  uint32_t tm = app.getUnixTime();
  memcpy(payload, &tm, 4);
  return app.publish(MQTTS_TOPICID_PREDEFINED_TIME, (const char*)payload, 4);
}


/*---------- Function for INT0 interuption --------*/
void intFunc(){
  
}

void setup(){
#if defined(MQTT_DEBUG) || defined(XBEE_DEBUG)
  debug.begin(9600);
#endif

/* -- Register Callback for INT0 --*/
  
  app.registerInt0Callback(intFunc);

/*-- Register Callbacks for WDT (Sec、callback)  --*/
  app.registerWdtCallback(900,wdtFunc0);

  app.begin(9600);         // Set XBee Serial baudrate
  app.init("Node-02");      // Initialize application
  app.setQos(1);            // Set QOS level 1

  app.setKeepAlive(300);            // Set PINGREQ interval
  app.setSleepMode(MQ_MODE_NOSLEEP);
  app.setClean(true);
  app.blinkIndicator(1000);  
  
  
  app.subscribe(MQTTS_TOPICID_PREDEFINED_TIME, setTime); // Set  date callback
  app.subscribe(tp1, blinkIndicator); 
  
  app.startWdt();          // Start Watch dog timer interruption
}
 
void loop(){  
    app.exec();
}