MQTT-S
======

MQTT-S Client via XBee  (running on linux and Arduino)

Supported functions
-------------------

*  QOS Level 0 and 1
*  Automatic SEARCHGW, GWINFO, ADVERTISE
*  Automatic WILLTOPICREQ, WILLTOPIC, WILLMSGREQ, WILLMSG
*  Automatic PINGREQ, PINGRESP
*  CONNECT, REGISTER, SUBSCRIBE, PUBLISH, UNSUBSCRIBE, DISCONNECT
*  CONNACK, REGACK, SUBACK, PUBACK, UNSUBACK
  
Module descriptions
-------------------
  
####1) MqttsClientApp.cpp  
Client application sample which is used for debugging.

####2) MqttsClient.cpp
  MQTT-S Client Engine class. This is used by application.
 
  Usages are shown as follows.
  
    MqttsClient mqtts = MqttsClient();  // declare the client object
    mqtts.begin(argv[1], B9600);  // argv[1] is a serial device for XBee. ex) /dev/ttyUSB0 
    mqtts.init("Node-02");  // Get XBee's address64, short address and set XBee Node ID, 
    mqtts.setQos(1);  // set QOS level.  0 or 1
    mqtts.setWillTopic(willtopic);  // set WILLTOPIC.   
    mqtts.setWillMessage(willmsg);  // set WILLMSG  those are sent automatically. 
    mqtts.setKeepAlive(60000);   // PINGREQ interval time
    mqtts.connect();   // Before CONNECT, SEARCHGW is sent automatically, if the gateway is lost.
    mqtts.subscribe(topic, callback); // Execute the callback, when the subscribed topic's data is published.
    mqtts.registerTopic(topic);  // Register topic and aquire a Topic ID 
    mqtts.publish(topic, payload, payload_length); // publish the data, topic is converted into ID automatically.
    mqtts.unsubscribe(topic);  
    mqtts.disconnect();
    
####3) MQTTS.cpp 
  MQTT-S messages classes and some classes for client and Gateway.
    
####4) ZBeeStack.cpp
  XBee control classes base on https://code.google.com/p/xbee-arduino/ 
    
####5) Mqtts_Defines.h
  Default setting is for Arduino.  (comment out both systems)
    
  //#define LINUX 
  //#define MBED
    
  select the system and uncoment it.
    
####6) MqttsGateway.cpp
  Gateway dummy Class for debug. Not real gateway.
    
####7) MqttsGatewayApp.cpp  
  Gateway application for Debug. 
  
####8) MqttsClientAppFw4Arduino.cpp
  Application framework for Arduino.
  Interupt and  watch dog timer are supported.
  
   
   
  
XBee configuratios
----------------------
  Serial interfacing  
  [BD] 0-7   
  [D7] 1 
  [D6] 0 or 1  
  [AP] 2

  Other values are defaults. Baudrate is used by  mqtts.begin(device, baudrate) function. 
  If you set D6 to 1, uncomment a line //#define XBEE_FLOWCTL_CRTSCTS in Mqtts_Defines.h
  

How to Compile (Requiered sources)
-----------
####1) Linux Client
  MQTTS_Defines.h
  ZBeeStack.h
  ZBeeStack.cpp
  Mqtts.cpp
  Mqtts.h
  MqttsClient.cpp
  MqttsClient.h
  MqttsClientApp.cpp

####2) Linux Gateway
  MQTTS_Defines.h
  ZBeeStack.h
  ZBeeStack.cpp
  Mqtts.cpp
  Mqtts.h
  MqttsGateway.cpp
  MqttsGateway.h
  MqttsGatewayApp.cpp
  
####3) Arduino Client
  MQTTS_Defines.h
  ZBeeStack.h
  ZBeeStack.cpp
  Mqtts.cpp
  Mqtts.h
  MqttsClient.cpp
  MqttsClient.h
  MqttsClientAppFwArduino.cpp
  MqttsClientAppFwArduino.h
  Copy those files into Aruino Librally directory/
  
  
  
Contact
-------

* Author:    Tomoaki YAMAGUCHI
* Email:     tomoaki@tomy-tech.com
* Twitter:   [@ty4tw]



[@ty4tw]:                     http://twitter.com/ty4tw

