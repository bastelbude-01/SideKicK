//Include Libraries
#include <SPI.h>
#include <RF24.h>

//create an RF24 object
RF24 radio(8, 9);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "00001";

void setup()
{
  radio.begin();
  
  //set the address
  radio.openWritingPipe(address);
  
  //Set module as transmitter
  radio.stopListening();
}
void loop()
{
  //Send message to receiver
  const char text[] = "Hallo Welt";
  radio.write(text, sizeof(text));
  
  delay(1000);
}
