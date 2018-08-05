#include <SPI.h>  
#include "RF24.h"
#include <RF24Network.h>
#include <EEPROM.h> //EEPROM functions

RF24 myRadio (9, 10);
RF24Network network(myRadio); 

// Variables
float lowBat = 2.8; //voltage value for low battery
int analogValue = 0;
float voltage = 0;
bool timer = true;
unsigned long tInterval = 10000; //Time between sending temp and battery to Coordinator (In Milliseconds)
unsigned long tStart;

const unsigned short int pairingAddress = 05; // Default Address of End Device for pairing
int endDeviceAddress; // Address of End Device
bool settingsSet = false; // Used so that you only run network.begin and router.node begin once in normal operation loop
int NodesConnected[5] = {0,0,0,0,0};
const short max_active_nodes = 4; //maximum allowed nodes connected
short num_active_nodes = 0; //index for NodeConnected array
int next_available_address;
bool PairedwithCoordinator=false;
bool Turned_LED_Off = false;


//changes these back after demo
int thisNode;  // This should just be delcared not equal to anything, I use 02 for demo purposes
bool pairDone = false; // To signify that the device is paired and has an address from the coordinator
//bool pairDone = true; //This should be false in real program, I have it to true for testing

//Structures

struct pairPayload  
{
  int newNodeAddress;
};
typedef struct pairPayload PairPayload;


struct payload_t
{
  int  LM355ADCRead; //temperature from onboard sensor
  bool  batState; //bool to communicate battery power level, true is good and false means battery needs to be replaced
};

  
struct ServoPayload
{
  int degreesToTurn;
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
  digitalWrite(7,HIGH);
  delay(1000);
  digitalWrite(7,LOW);
  
  // check if pairDone is true in EERPROM
  // Grab Your Assigned Address (thisNode) from EEPROM
  // You also need to Store the NodesConnected[] array and num_active_nodes and next_available_address
  
}

void loop()
{
  //Normal Operation
  if( (digitalRead(8) == LOW) && pairDone == true) {
      Serial.println("Normal Operation");

      if (!Turned_LED_Off){
            digitalWrite(7,LOW);
            Turned_LED_Off = true;
        }

      if (settingsSet == false) {
          EEPROM.get(1,thisNode);//This must be called before network.begin but only needs to be called once
          next_available_address = thisNode + 8; //address for the end nodes
          settingsSet = true;
         }
       network.begin(90, thisNode); //Note that thisNode gets the node addrress of this node from EEPROM
       sendSensorData();
       network.update();
       RF24NetworkHeader header1;
      if(network.available()){
          while( network.available() ) {
            network.peek(header1); // give it the header of the next message (this saves us from reading the whole message)
            switch (header1.type) {
              case 'S':           // An 'S' type header is the cordinator telling the vents that open/close 
                handle_S(header1);
                break;
              default:
                Serial.println("Unknown Packet Received of type: \n");
                Serial.println(header1.type); // Lets you know what packet type was recieved
                network.read(header1,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                break;
            };
          }
        }
    }

    // Pairing Process
    if( digitalRead(8)==HIGH) { //You cant have pairDone==false in this because otherwise when you get an address from coordinator pairDone==true and you would never be able to pair with an end device
        Serial.println("Waiting to Pair");
        network.begin(90, 05); // This is called a bunch of times (could be a problem)
        //delay(1000);
        network.update();
        RF24NetworkHeader header1; //create header object to store header of next message in
        if(network.available()){
          
            while(network.available() ) {
              
              network.peek(header1); // give it the header of the next message (this saves us from reading the whole message)
              switch (header1.type) {
                case 'P':           // A 'P' type header is an end device wanting to pair with a router
                  handle_P(header1);
                  break;
                case 'C':           // A 'C' type header is when a coordinator wants to pair with a router
                  handle_C(header1);
                  break;
                default:
                  Serial.println("Unknown Pairing Packet Received of type: \n");
                  Serial.println(header1.type); // Lets you know what packet type was recieved
                  network.read(header1,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                  break;
              };
          }
        }
        
        else {
              Serial.println("Not Paired Yet");
            }
    }
      
 delay(2000);
}

//Functions

bool sendSensorData() {
 bool xMit = true; //this stay true if transmit was good
  if(timer){
      network.update(); //check to see if there is any network traffic that needs to be passed on, technically an end device does not need this 
      payload_t payload = { getTemp(), checkBatteryVolt()};
      RF24NetworkHeader header(00,'R'); //Create transmit header. This goes in transmit packet to help route it where it needs to go, in this case it is the coordinator
      
      //send data onto network and make sure it gets there
      if (network.write(header,&payload,sizeof(payload))) {
        Serial.println("Transmission to Coordinator Successful");
        //digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
      }
      else  { //transmit failed, try again
        if (!network.write(header,&payload,sizeof(payload))) {
          //PIND |= (1<<PIND7); //this toggles the status LED at pin seven to show transmit failed
          Serial.println("Transmission to Coordinator Unsuccesful");
          xMit = false; //transmit failed so let the user know
        }
        else  {
          //digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
        }
       }
   }
    timer = checkTimer();
    return xMit;
}

void handle_S (RF24NetworkHeader& header) {
  // pass the packet to all vents in NodesConnected
  ServoPayload endDevicePayload1; //create payload to read degreesToTurn from coordinator
  network.read(header, &endDevicePayload1, sizeof(endDevicePayload1)); //Read the packet with Degrees to turn from coordinator
  int val_1;
  val_1 = endDevicePayload1.degreesToTurn;

  for (int a=0; a < 5; a++ ) {
     ServoPayload servoPayload1;
     servoPayload1.degreesToTurn = val_1;
     endDeviceAddress = 012;
     RF24NetworkHeader header3(endDeviceAddress,'S');
    
    if (network.write(header3, &servoPayload1, sizeof(servoPayload1))) {
        //digitalWrite(7,LOW);//transmit was successful so make sure status LED is off
        //Serial.println("Sent Packet to End Device: ");
        //Serial.print(endDeviceAddress,OCT);
      }
      
      else  { //transmit failed, try again
            if (!network.write(header3, &servoPayload1, sizeof(servoPayload1))) {
                //Serial.print("Sent Packet to an End Device on 2nd Try");
                //PIND |= (1<<PIND7); //this toggles the status LED at pin seven to show transmit failed
            }
            else  {
             // digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
            }
      }
  }
}

void handle_P(RF24NetworkHeader& header) {
  if(pairDone){          //Need to be paired with coordinator first before pairing with a router
        // Send the end device its new address
        pairPayload endDevicePayload;

        endDevicePayload.newNodeAddress = next_available_address;
        RF24NetworkHeader header2(05,'L'); // Create Pair Packet for End Device to Listen to, address 05 type L.
        
        if (network.write(header2, &endDevicePayload, sizeof(endDevicePayload))) {
            //Check for M packets to confrim vent got its address then add to nodes connected
              network.update();
              RF24NetworkHeader header5;
              
             if(network.available()){
                  while(network.available() ) {
                    
                    network.peek(header5); // give it the header of the next message (this saves us from reading the whole message)
                    switch (header5.type) {
                      case 'M':           // A 'P' type header is an end device wanting to pair with a router
                        handle_M(header5);
                        break;
                      default:
                        //Serial.println("Unknown Pairing Packet Received of type: \n");
                        //Serial.println(header5.type); // Lets you know what packet type was recieved
                        network.read(header5,0,0); // Now we clear the packet from frame buffer since before we just peeked at it we didnt clear it
                        break;
                    };
                }
              }
             else {
                Serial.println("Vent did not sent M packets");
              }
            }
        
        else  { //transmit failed, try again
                Serial.println("Failed to transmit L packets to Vent");
              }
     } else {
        Serial.println("I need to pair with a Cordinator first");
      }
  
}

void handle_M(RF24NetworkHeader& header) {
  pairPayload confirmAddress;
  network.read(header, &confirmAddress, sizeof(confirmAddress));
  add_node(confirmAddress.newNodeAddress); //transmit was successful so add node to ConnectedNodes
}


void handle_C(RF24NetworkHeader& header) {
  if(PairedwithCoordinator == false){
     pairPayload CoordinatorPayload; // create pairPayload object to read received payload
     network.read(header, &CoordinatorPayload, sizeof(CoordinatorPayload));
     int val;
     val = CoordinatorPayload.newNodeAddress;
     next_available_address = CoordinatorPayload.newNodeAddress + 8;
     EEPROM.put(1, val);
     Serial.println("I received my address:");
     Serial.println(val,OCT); 
     digitalWrite(7,HIGH); // turn LED on to indicate pairing is done.
     Turned_LED_Off = false;
     
     RF24NetworkHeader header3(00,'D');
     //send respond to coordinator so it knows we got the address
     if (network.write(header3,&CoordinatorPayload, sizeof(CoordinatorPayload))) {
         Serial.println("Coordinator Knows I Received My Address");
         PairedwithCoordinator = true; //
         pairDone = true; // Let the normal operation of the device begin since we have an address now.
      }
      else  { 
          Serial.println("Coordinator Did not get Confirm Message");
        }
  } else {
    Serial.println("Already Paired with Coordinator");
    network.read(header,0,0);
    //delay(1000);
  }
}


void add_node(uint16_t node)
{
  // Do we already know about this node?
  short i = num_active_nodes; // update i to equal the current number of active nodes
  while (i--)
  {
      // if we do break (don't add it)
    if ( NodesConnected[i] == node ){
      Serial.println("Already Added this Node");
      break;
      }
  }
  // If not, add it to the array
  if ( i == -1 && num_active_nodes < max_active_nodes )
  {
    NodesConnected[num_active_nodes++] = node;
    next_available_address = node + 8;
    Serial.println("Paired with Vent");
    Serial.println("It has the Address: ");
    Serial.println(node, OCT);
    for(int j=0; j < sizeof(NodesConnected)/sizeof(int);j++){
        Serial.println(NodesConnected[j]);
      }
      
  } else {
    // there are too many nodes connected
  }
}

int getTemp(){
  return analogRead(A0);
}

bool checkBatteryVolt()
{
  analogValue = analogRead(A1);
  voltage = 0.020796*analogValue;
  if(voltage < lowBat) return false; //This will alert the coordinator that the battery power is low
  else return true; //battery is fine 
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


