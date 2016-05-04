#include <Arduino.h>

#include "IRCommunication.h"


#define MSG_SIZE 2
byte msg[MSG_SIZE];

byte myID = 0x01;
byte otherID = 0x11;

void setup() {
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  
  // This should be called if planning to send 2-byte (payload) messages
  ResetIR(MSG_SIZE);

  Serial.begin(9600);
  randomSeed(analogRead(0));
  RestartTimer();
}

void sendExample() {
  // Simple 2-byte message sending exampled
  ResetIR(MSG_SIZE);
  byte targetID = otherID;   // ID of message target

  // Message payload
  msg[0] = 0xA0;
  msg[1] = 0xA4;

  //OnEyes(0,50,50); // use eye color to indicate what the Ringo is doing, or state, etc. (this is just a placeholder)

  // Send a message to device with ID targetID
  for (int i = 0; i < 5; i++) {
    SendIRMsg(myID, targetID, msg, MSG_SIZE); delay(100);
  }

  // To receive above message on ID 0x02 Ringo, issue ReceiveIRMsg(senderID, 0x02, msg, MSG_SIZE);
  ResetIR(MSG_SIZE); // Important!
  delay(random(100,150));          // delay 1 second (1000 milliseconds = 1 second)
}

void receiveExample() {
  byte sender;
  bool done = 0;
  // put your code here inside the loop() function.  Here's a quick example that makes the eyes alternate colors....
  //if (!done) OnEyes(50,0,0);      // you can remove this stuff and put your own code here
  if (ReceiveIRMsg(sender, myID, msg, MSG_SIZE)) {
#ifdef __DEBUG
    Serial.print(myID, HEX);
    Serial.print(" received IR message from "); Serial.println(sender, HEX);
    // Print just the payload
    Serial.println(msg[0],HEX);
    Serial.println(msg[1],HEX);
#endif

    if (msg[0] == 0xA0) {
      Serial.println("Message accepted!");
      done = 1;
    }
    ResetIR(MSG_SIZE);
  }
  delay(random(50,150));
}

void loop(){

  // Uncomment either send or receive before upload to two different devices for testing.
  sendExample();
  //receiveExample();
}
