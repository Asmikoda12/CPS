#include <LoRa.h>

int counter = 0;
//char paket;

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

#define BAND 923E6

void setup() {
  Serial.begin(115200); // Initialize the hardware serial port
    //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
}
void loop() {
  
  if (Serial.available()>0) {
    String paket = Serial.readString();
    // Process the received data here
    // Echo back the data to the serial port
    Serial.print(paket);
    //paket = data;
    
 
   //Send LoRa packet to receiver
  LoRa.beginPacket();
  //LoRa.print("LoraData:");
  LoRa.print(paket);
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  }

}