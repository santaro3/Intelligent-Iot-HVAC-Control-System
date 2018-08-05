#define SERIAL_BUFFER_SIZE 50
#include <SPI.h>  
#include "RF24.h"
#include <RF24Network.h> 
String serial_buffer;
String param1 = "address:";
String param2 = "degrees:";
RF24 myRadio (7, 8);
RF24Network network(myRadio);


//Variables
// Holds the index of where to store the next received address
short num_active_nodes = 0;
short num_01_nodes = 0;
short max_active_nodes = 4;
int NodesConnected[4] = {}; // Array to hold all connected nodes (You can only have at most 5 end devices connected)
int endNodesConnected[4] = {}; 
int next_avaiable_address = 02; //The directly paired end devices all have 01 as their address (hard coded after pairing with Coordinator)so the next available address is 02 for the first router
int next_01_address = 011; //This is just used to keep track of the number of 01 nodes connected to me

struct pairPayload  
{
  int newNodeAddress;
};


struct payload_t
{
  int  LM355ADCRead; //temperature from onboard sensor
  bool  batState; //bool to communicate battery power level, true is good and false means battery needs to be replaced
};

struct payload_e  
{
  bool  batState;
  int  Node01address;
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
  Serial.println("Coordinator is online.....");
  network.begin(90, 00);
}

void loop()
{    
//        ServoPayload servoPayload1;
//        RF24NetworkHeader header5(01,'S'); 
//        servoPayload1.degreesToTurn = 60;
//
//        //Serial.println("Sending Servo Packets");
//        
//        if(network.write(header5, &servoPayload1, sizeof(&servoPayload1))) {
//            Serial.println("Sent Servo Packets Successfully");
//            // Might have to add another packet from router to confirm it got the address
//            
//        } else {
//            //Serial.println("Failed to Send Servo Packets");
//            // probably need to add loop that loops a couple times to confirm packets were sent (clear that variable at end of main arduino loop()) because you have multiple nodes on 01
//        }

  
    //Check Serial Communication from Eric
    if(Serial.available()){

        serial_buffer = Serial.readString();
    
        // Extract useful data from the message
        // -1 added to accomodate for the comma
        int address = serial_buffer.substring(serial_buffer.indexOf(param1) + param1.length(), serial_buffer.indexOf(param2) - 1).toInt();
        int degree_val = serial_buffer.substring(serial_buffer.indexOf(param2) + param2.length(), serial_buffer.indexOf("}")).toInt();
        
        // Clear serial buffer
        serial_buffer[0] = '\0';
        
        ServoPayload servoPayload1;
        RF24NetworkHeader header5(address,'S'); 
        servoPayload1.degreesToTurn = degree_val;

        //Serial.println("Sending Servo Packets");
        
        if(network.write(header5, &servoPayload1, sizeof(&servoPayload1))) {
            //Serial.println("Sent Servo Packets Successfully");
            // Might have to add another packet from router to confirm it got the address
            
        } else {
            //Serial.println("Failed to Send Servo Packets");
            // probably need to add loop that loops a couple times to confirm packets were sent (clear that variable at end of main arduino loop()) because you have multiple nodes on 01
        }
    }
   
   //Normal Operation
    if ( digitalRead(2)== LOW) { //Digital pin 8 is low then normal operation 
      Serial.println("Normal Operation");
      network.update();
      RF24NetworkHeader header1;
      
      if(network.available()){
          while ( network.available()) { 
            network.peek(header1); // give it the header of the next message (this saves us from reading the whole message)
            switch (header1.type) {
              case 'E':           // An 'E' type header is an end device sending coordinator its bat level
                handle_E(header1);
                break;
              case 'R':           // A 'R' type header is a router sending its temp and bat level
                handle_R(header1);
                break;
              default:
                //Serial.println("Unknown Pairing Packet Received of type: \n");
                //Serial.println(header1.type); // Lets you know what packet type was recieved
                network.read(header1,0,0); // Now we clear the packet from frame buffer before we just peeked at it
                break;
            };
          } 
        }
    }

    // Pairing Process
    if ((digitalRead(2)== HIGH) && (num_active_nodes < max_active_nodes)) { //D8 is high and we don't have more than 5 routers connected
      //Send C type packets to 05 with its new address
      pairPayload routerPayload1;
      routerPayload1.newNodeAddress = next_avaiable_address;
      Serial.println("Sending Pairing Packets");
      RF24NetworkHeader header(05,'C'); // Create Pair Packet for router to Listen to, address 05 type C.
      if (network.write(header, &routerPayload1, sizeof(&routerPayload1))) { 
             //delay(2000); //wait a little so that the router has time to send us a D packet to confirm it got the address
              network.update();
              RF24NetworkHeader header2;
              
              if(network.available()){
                  while ( network.available()) { 
                    network.peek(header2); // give it the header of the next message (this saves us from reading the whole message)
                    switch (header2.type) {
                      case 'D':           // Router wants to Pair
                        handle_D(header2);
                        break;
                      case 'Q':           // End Device Wants to Pair
                        handle_Q(header2);
                        break;
                      default:
                       // Serial.println("Unknown Pairing Packet Received of type: \n");
                        //Serial.println(header2.type); // Lets you know what packet type was recieved
                        network.read(header2,0,0); // Now we clear the packet from frame buffer before we just peeked at it
                        break;
                    };
                  } 
              }
          }
      else  { //transmit failed, try again
            if (!network.write(header, &routerPayload1, sizeof(&routerPayload1))) {
               //Serial.println("Failed to Pair");
              }
            else  {
                //transmit was successfuly on 2nd try
                network.update();
                RF24NetworkHeader header3;
                
                if(network.available()){
                    while ( network.available()) { 
                      network.peek(header3); // give it the header of the next message (this saves us from reading the whole message)
                      switch (header3.type) {
                        case 'D':           // Router wants to Pair
                          handle_D(header3);
                          break;
                        case 'Q':           // End Device Wants to Pair
                          handle_Q(header3);
                          break;
                        default:
                          //Serial.println("Unknown Pairing Packet Received of type: \n");
                          //Serial.println(header3.type); // Lets you know what packet type was recieved
                          network.read(header3,0,0); // Now we clear the packet from frame buffer before we just peeked at it
                          break;
                      };
                    } 
                }
            }
         }
  }

  delay(2000);
}

void handle_R (RF24NetworkHeader& header) {

  payload_t payload; // Used to read the received router payload
  network.read(header,&payload,sizeof(payload));

  
    //Serial.print("The node this is from: ");
    //Serial.println(header.from_node,OCT); //tells you the address of the sending node
  
    //Serial.print("Temperature: ");
    //Serial.print(payload.LM355ADCRead); //tells you the temperature reading of the sending node
    
    //Serial.print(" Battery status: ");
   // Serial.println(payload.batState);
   
      //Send Data to Eric
      Serial.print("{address:");
      Serial.print(header.from_node);
      Serial.print(",batState:");
      Serial.print(payload.batState);
      Serial.print(",temp:");
      Serial.print(payload.LM355ADCRead); // 0 for end devices
      Serial.println("}");
    
}

void handle_E (RF24NetworkHeader& header) {
  payload_e payload2;
  RF24NetworkHeader header9;
  
  network.read(header9,&payload2,sizeof(payload2));
  if(payload2.Node01address == 0) {
     // Serial.print("The node this is from: ");
     // Serial.println(header.from_node); //tells you the address of the sending node
    //  Serial.print(" Battery status: ");
      //Serial.println(payload.batState); // battery status of the sending node

     // if(Serial) {
        //Send Info to Eric
        Serial.print("{address:");
        Serial.print(header9.from_node,OCT);
        Serial.print(",batState:");
        Serial.print(payload2.batState);
        Serial.print(",temp:");
        Serial.print(0); // 0 for end devices
        Serial.println("}");
     // }
    
  }
  else if (header9.from_node == 01) {
      //Serial.println("This is actually from: ");
      //Serial.println(payload.Node01address,OCT);
     // Serial.print(" Battery status: ");
      //Serial.println(payload.batState); // battery status of the sending node

      //Send Info to Eric
      //if(Serial) {
        Serial.print("{address:");
        Serial.print(payload2.Node01address); //This is the logical 011 address of the 01 end devices;
        Serial.print(",batState:");
        Serial.print(payload2.batState);
        Serial.print(",temp:");
        Serial.print(0); // 0 for end devices
        Serial.println("}");
     // }
    }
    
  else {
    //Serial.print("Check this");
    }
}

void handle_D(RF24NetworkHeader& header) {
  pairPayload confirmedAddress;
  network.read(header,&confirmedAddress,sizeof(confirmedAddress));
  //digitalWrite(7,HIGH); //transmit was successfully received so make sure we keep the LED on to let the user know to stop pushing button
  //TurnLightOff = true; // Tells the coordinator to turn off the light in normal operation
  add_node(confirmedAddress.newNodeAddress);
}

void handle_Q(RF24NetworkHeader& header) {
  RF24NetworkHeader header6(05,'T');
  pairPayload endDeviceAddress;
  endDeviceAddress.newNodeAddress = next_01_address;

  if (network.write(header6,&endDeviceAddress,sizeof(endDeviceAddress))) {
       
      network.update();
      RF24NetworkHeader header7;
      
      if(network.available()){
          while ( network.available()) { 
            network.peek(header7); // give it the header of the next message (this saves us from reading the whole message)
            switch (header7.type) {
              case 'V':           // Router wants to Pair
                 pairPayload confirmedAddress1;
                 network.read(header7,&confirmedAddress1,sizeof(confirmedAddress1));
                 add_01_Node(confirmedAddress1.newNodeAddress);
                break;
              default:
                //Serial.println("Unknown Pairing Packet Received of type: \n");
                //Serial.println(header7.type); // Lets you know what packet type was recieved
                network.read(header7,0,0); // Now we clear the packet from frame buffer before we just peeked at it
                break;
            };
          } 
      }
  }
  else {
    //Serial.println("End Device Did not Get Address");
    }
}

void add_node(uint16_t node)
{
      // Do we already know about this node?
      short i = num_active_nodes; // update i to equal the current number of active nodes
      while (i--)
      {
          // if we do break (don't add it)
        if ( NodesConnected[i] == node )
          //Serial.println("Already Have Router Connected");
          break;
      }
      // If not, add it to the array
      if ( i == -1 && num_active_nodes < max_active_nodes )
      {
        NodesConnected[num_active_nodes++] = node;
        next_avaiable_address = node + 1;
        //Serial.println("Router has its address");
      } else {
        //Serial.print("Too many Nodes Connected"); //You Shouldn't ever get here since we check num_active_nodes in Pairing Loop
      }
 
}
void add_01_Node(uint16_t node){
  short i = num_01_nodes; // update i to equal the current number of active nodes
      while (i--)
      {
          // if we do break (don't add it)
        if ( endNodesConnected[i] == node )
          //Serial.println("Already Have Router Connected");
          break;
      }
      // If not, add it to the array
      if ( i == -1 && num_01_nodes < max_active_nodes )
      {
        endNodesConnected[num_01_nodes++] = node;
        next_01_address = node + 8;
        //Serial.println("Router has its address: ");
        //Serial.println(node,OCT);
        //Serial.println("Next Address to Give out is: ");
        //Serial.println(next_01_address,OCT);
      } else {
        //Serial.println("Too many Nodes Connected01 or Already Added this Address");
        //Serial.println(i);
      }
}

