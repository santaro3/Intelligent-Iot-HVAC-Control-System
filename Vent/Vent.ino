#include <SPI.h>  
#include "RF24.h"
#include <RF24Network.h>
#include <Servo.h>
#include <EEPROM.h> //EEPROM functions

RF24 myRadio (9, 10); //changed
RF24Network network(myRadio);

Servo myservo;
int thisNode;

//Variables
float lowBat = 2.8; //voltage value for low battery
int analogValue = 0;
float voltage = 0;
bool timer = true;
unsigned long tInterval = 10000; //Time between sending temp and battery to Coordinator (In Milliseconds)
unsigned long tStart;
bool Turned_LED_Off = false;

bool pairDone = false; // To signify that the device is paired and has an address from the coordinator
bool settingsSet = false; // Used so that you only run network.begin and router.node begin once in normal operation loop
const unsigned short int pairingAddress = 05; // Default Address of End Device
bool PairedwithCoordinator = false;
int Node01Address = 00; //This is just to make sure all end devices without 01 sent 00 in their end device payload


//Structures
struct pairPayload  
{
  int newNodeAddress;
};

struct ServoPayload
{
  int degreesToTurn;
};

struct payload_e  
{
  bool batState;
  int  Node01address;
};



void setup()
{
  Serial.begin(115200);
  delay(1000);
  myRadio.begin();  
  myRadio.setChannel(90); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS ) ;
  pinMode(7, OUTPUT);
  myservo.attach(3);
  myservo.write(90); //Vents closed by default
  digitalWrite(7,HIGH);
  delay(1000);
  digitalWrite(7,LOW);
  myservo.detach();
}

void loop()
{        
    //Normal Operation
    if(digitalRead(8) == LOW && pairDone == true) { //Digital pin 8 is low AND the end device has paired with a router
        Serial.println("Normal Operation");
        
        if (!Turned_LED_Off){
            digitalWrite(7,LOW);
            Turned_LED_Off = true;
        }
        
        if (settingsSet == false) { // This is just to avoid calling router.nodeBegin and network.begin each loop()
            EEPROM.get(1,thisNode); //This must be called before network.begin but only needs to be called once
            network.begin(90, thisNode); //Note that router.thisNode gets the node addrress of this node from EEPROM
            //digitalWrite(7,LOW); // The LED will be on after successful pair so we turn it off to tell the user we made it to normal operation
            settingsSet = true;
        } 
        
        if(!endDeviceSendSensorData()) {
          //If this is true transmit failed
          Serial.println("Failed to transmit end device packet to cordinator");
          Serial.println("Sending on: ");
          Serial.println(thisNode,OCT);
        }
        
        network.update();
        RF24NetworkHeader header1;
        
        if (network.available()) {
          // Checking for Servo packets
          while( network.available() ) {
            
            network.peek(header1); // give it the header of the next message (this saves us from reading the whole message)
            switch (header1.type) {
              case 'S':           // An 'S' type header is the cordinator telling the vents that open/close 
                handle_S(header1);
                break;
              default:
                Serial.println("Unknown Pairing Packet Received of type: \n");
                Serial.println(header1.type); // Lets you know what packet type was recieved
                network.read(header1,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                break;
            };
          }
       }
    }

    
    // Pairing with router or Coordinator
    if (digitalRead(8)==HIGH && pairDone == false) { //If D8 is high pair with a router
        network.begin(90, 05); //
        delay(1000);
        //Dont ned a network.update because we are just writing to the router, we dont even have an address yet so no need to listen to packets 
        
        RF24NetworkHeader header(05,'P'); // Create Pair Packet for router to Listen to, address 05 type P.
        if (network.write(header, 0, 0)) { // Make blank packet to send to router (its just to tell the router that an end device needs an address) 
              //digitalWrite(7,LOW); //transmit was successfully received so make sure status LED is off
              Serial.println("Router Knows We Want to Connect"); // So it is sending "L" packets
              //Now we check if we got any L packets
              network.update();
              RF24NetworkHeader header2; //create header object to store header of next message in
              
              if(network.available()){
                while(network.available() ) {
                    //Lets look at the header type to determine how to handle it
                    network.peek(header2); // give it the header of the next message (this saves us from reading the whole message)
                    switch (header2.type) {
                      case 'L':          
                        handle_L(header2);
                        break;
                      default:
                        Serial.println("Unknown Pairing Packet Received of type: \n");
                        Serial.println(header2.type); // Lets you know what packet type was recieved
                        network.read(header2,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                        break;
                    };
                  }
              }
              
              else {
                  Serial.println("Router knows we want to Pair but we aren't getting its L Packets");
                }
          }
          else {
              Serial.println("Pairing Failed, Router did not Receive P packets");
              Serial.println("Checking to see if its the Coordinator that Wants to Pair, not a Router");
              
              network.update();
              RF24NetworkHeader header5; //create header object to store header of next message in
   
              if(network.available()){
                while(network.available() ) {
                    //Lets look at the header type to determine how to handle it
                    network.peek(header5); // give it the header of the next message (this saves us from reading the whole message)
                    switch (header5.type) {
                      case 'C':          
                        handle_C(header5);
                        break;
                      default:
                        Serial.println("Unknown Pairing Packet Received of type: ");
                        Serial.println(header5.type); // Lets you know what packet type was recieved
                        network.read(header5,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                        break;
                    };
                  }
              }
          }
        }
  delay(2000);
}


bool endDeviceSendSensorData() {
  bool xMit = true; //this stay true if transmit was good  
  if(timer){
      payload_e payload = { checkBatteryVolt(), Node01Address};
      RF24NetworkHeader header3(00,'E'); //Create transmit header. This goes in transmit packet to help route it where it needs to go, in this case it is the coordinator
      //send data onto network and make sure it gets there
      
      if (network.write(header3,&payload,sizeof(payload))) {
        //digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
        Serial.print("Successfully Transmitted Packet");
      }
      
      else  { //transmit failed, try again
          if (!network.write(header3,&payload,sizeof(payload))) {
            //PIND |= (1<<PIND7); //this toggles the status LED at pin seven to show transmit failed
            xMit = false; //transmit failed so let the user know
          }
          else  {
            //digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
            Serial.print("Successfully Transmitted Packet on 2nd Try");
          }
       }
    }
    timer = checkTimer();
    return xMit;
}


void handle_L(RF24NetworkHeader& header) {
  pairPayload routerPayload; // create pairPayload object to read received payload (the new address for this node)
  network.read(header, &routerPayload, sizeof(routerPayload));
  
  RF24NetworkHeader header4(05,'M'); // Tell router we got the address
  if (network.write(header4,&routerPayload,sizeof(routerPayload))) {
      int val;
      val = routerPayload.newNodeAddress;
      EEPROM.put(1, val);
      Serial.println("Router knows we have the address: ");
      Serial.println(val,OCT);
      pairDone = true; // Let the normal operation of the device begin since we have an address now.
      digitalWrite(7,HIGH); // This turns the LED on so you know you are done pairing
      delay(3000);
  } else {
      Serial.println("Router Did not get confirmation packet");
    }
}

void handle_S (RF24NetworkHeader& header) {
    myservo.attach(3);
    ServoPayload servoPayload1; // create servo object to read received payload
    network.read(header, &servoPayload1, sizeof(servoPayload1));
    int val;
    val = servoPayload1.degreesToTurn;
    myservo.write(val);
    delay(1000);
    myservo.detach();
    Serial.println("I was told to move servo to: ");
    Serial.println(val);
    //Serial.println(typeof(val));
    
}

void handle_C(RF24NetworkHeader& header) {
  if(PairedwithCoordinator == false){
    
     RF24NetworkHeader header5(00,'Q'); // Tell coordinator an end device wants to pair
      if (network.write(header5,0,0)) {
        
         network.update();
         RF24NetworkHeader header6;
         
         if(network.available()){
              while(network.available() ) {
                network.peek(header6); // give it the header of the next message (this saves us from reading the whole message)
                switch (header6.type) {
                  case 'T':           // A 'T' type header is an end device wanting to pair with a router
                    handle_T(header6);
                    break;
                  default:
                    network.read(header6,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                    break;
                };
            }
          }
         
         else {
            Serial.println("Vent did not get T packets from Coordinator");
          }
      } 
      
      else {
            Serial.println("Coordinator Did not Receive Q packets");
        }

  } 
  else {
    Serial.println("Already Paired with Coordinator");
    network.read(header,0,0); //remove packet from buffer
    //delay(1000);
  }
}


void handle_T(RF24NetworkHeader& header){
     
     pairPayload CoordinatorPayload; // create pairPayload object to read received payload
     network.read(header, &CoordinatorPayload, sizeof(CoordinatorPayload));
      
     RF24NetworkHeader header8(00,'V');

     if (network.write(header8,&CoordinatorPayload,sizeof(CoordinatorPayload))) {
         
         Node01Address = CoordinatorPayload.newNodeAddress;
         EEPROM.put(1, 01);
         Serial.println("I received my address from Coordinator:");
         Serial.println(Node01Address, OCT);
         PairedwithCoordinator = true;
         pairDone = true;
         digitalWrite(7,HIGH);
         delay(3000);
     }
     else {
        Serial.print("Coordinator did not get V packets confirming I got the address");
      }
     
}

bool checkBatteryVolt()
{
  analogValue = analogRead(A1);
  voltage = 0.020796*analogValue;
  if(voltage < lowBat) return false; //This will alert the coordinator that the battery power is low
  return true; //battery is fine 
}

bool checkTimer()
{
  unsigned long now = millis(); //get timer value
  if ( now - tStart >= tInterval  ) //check to see if it is time to transmit based on set interval
  {
    tStart = now; //reset start time of timer
    return true;
  }
  else return false;
}


