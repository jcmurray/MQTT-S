/*
 * MqttsClientApp.cpp
 *                       The MIT License (MIT)
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
 * THE SOFTWARE.
 *
 *  Created on: 2013/12/15
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */

#ifdef ARDUINO
    #include <MQTTS_Defines.h>
#else
    #include "mqttslib/MQTTS_Defines.h"
    #include "mqttslib/MqttsClient.h"
#endif

#ifdef MBED
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

MQString* willtopic = new MQString("willtopic");
MQString* willmsg   = new MQString("willmsg");
MQString* on = new MQString("on");
MQString* off = new MQString("off");
MQString *tp1 = new MQString("dev/indicator");

DigitalOut indicator(LED1);

int  blinkIndicator(MqttsPublish* msg){
  MQString sw = MQString();
  sw.readBuf(msg->getData());
  if( sw == *on){
    indicator = 1;
  }else if( sw == *off){
    indicator = 0;
  }
  return 0;
}

int main(int argc, char **argv){

    MqttsClient mqtts = MqttsClient();

    mqtts.init("mbed-01");
    mqtts.begin(9600);

    mqtts.setQos(1);
    mqtts.setClean(true);
    mqtts.setWillTopic(willtopic);
    mqtts.setWillMessage(willmsg);
    mqtts.setKeepAlive(60);

    mqtts.subscribe(tp1, blinkIndicator);

    XTimer tm = XTimer();

    while(true){

        for(int i = 0; i < 10; i++){
            tm.start(10000);
            while(!tm.isTimeUp()){
                mqtts.exec();
            }
        }
   }
}

#endif   /* MBED */
