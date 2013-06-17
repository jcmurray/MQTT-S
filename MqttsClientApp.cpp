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
 *  Created on: 2013/06/17
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
 *
 */

#ifdef ARDUINO
  #include <MQTTS_Defines.h>
  #include <MqttsClient.h>
#else
  #include "MQTTS_Defines.h"
  #include "MqttsClient.h"
#endif

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


MQString* willtopic = new MQString("willtopic");
MQString* willmsg   = new MQString("willmsg");


int fnTp1(MqttsPublish* msg){
  printf("Execute fnTp1\n");
  return 0;
}




int main(int argc, char **argv){

    MqttsClient mqtts = MqttsClient();

    mqtts.begin(argv[1], B38400);
    mqtts.init("Node-02");
    mqtts.setQos(1);
    //mqtts.setWillTopic(willtopic);
    //mqtts.setWillMessage(willmsg);
    mqtts.setKeepAlive(60000);


    fprintf(stdout,"Connect\n");
    mqtts.connect();
    mqtts.runConnect();

    MQString *topic0 = new MQString("a/bcd/ef");

    mqtts.registerTopic(topic0);
    mqtts.run();

    /*
    mqtts.disconnect();
    mqtts.run();
    */

    MQString *topic1 = new MQString("g/hij/kl");

    mqtts.subscribe(topic1, fnTp1);
    mqtts.run();

    /*
    mqtts.unsubscribe(topic1);
    mqtts.run();
    */

    while(true){
      int rc = mqtts.publish(topic0,"123456",6);
      mqtts.run();
      fprintf(stdout,"%d\n", rc);
    }

    mqtts.runLoop();

    fprintf(stdout,"__end__\n");


}
