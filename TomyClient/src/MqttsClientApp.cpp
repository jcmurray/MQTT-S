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
 *  Created on: 2013/11/30
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.1.0
 *
 */

#ifdef ARDUINO
  #include <MQTTS_Defines.h>
  #include <MqttsClient.h>
#else
  #include "mqttslib/MQTTS_Defines.h"
  #include "mqttslib/MqttsClient.h"
#endif

#if defined(LINUX) || defined(MBED)
  #include <stdio.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <string.h>
  #include <iostream>
#endif

MQString* willtopic = new MQString("willtopic");
MQString* willmsg   = new MQString("willmsg");


int fnTp1(MqttsPublish* msg){
  printf("****** Callback routine fnTp1() was executed.*****\n");
  return 0;
}


int main(int argc, char **argv){

    MqttsClient mqtts = MqttsClient();

    mqtts.begin(argv[1], B38400);
    mqtts.init("Node-03");
    mqtts.setQos(1);
    mqtts.setClean(true);
    mqtts.setWillTopic(willtopic);
    mqtts.setWillMessage(willmsg);
    mqtts.setKeepAlive(60);

    MQString *topic0 = new MQString("a/bcd/ef");
    MQString *topic1 = new MQString("dev/indicator");

    mqtts.registerTopic(topic0);

    mqtts.unsubscribe(topic0);
    //mqtts.subscribe(topic1,&fnTp1);

    MQString* on = new MQString("on");
    MQString* off = new MQString("off");


    while(true){

		for(int i = 0; i < 10; i++){
			mqtts.publish(topic1,(i % 2 ? on : off));
			mqtts.recvMsg(10000);
		}
		//mqtts.setClean(false);
		//mqtts.disconnect();
   }
}
