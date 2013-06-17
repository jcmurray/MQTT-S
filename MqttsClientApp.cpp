/*
 * MqttsClientApp.cpp
 *                       The MIT License (MIT)
 *
 *               Copyright (c) 2013, Tomoaki YAMAGUCHI
 *                       All rights reserved.
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
