#include "Arduino.h"
#include "Wire.h"

const char I2C_ADDR = 0x55; // Set to desired i2c-adress
#undef DEBUG                // Define for various debug outputs (#undef to disable) - !!!ENABLING SLOWS DOWN CODE SIGNIFICANTLY!!!

#define Module1 0x12
#define Module2 0x13
#define Module3 0x14

// defines GPIOs 18,19,21 for MUX Adressing
#define MUX_SELECT_PIN_0 18
#define MUX_SELECT_PIN_1 19
#define MUX_SELECT_PIN_2 21

#define BUILTIN_LED 25 // GPIO of BUILTIN_LED for pico
#ifdef esp32dev
#undef BUILTIN_LED
#define BUILTIN_LED 2 // GPIO of BUILTIN_LED for esp32dev
#endif

void sendData(float data1, float data2); // Function to send data back to the master
void sendData(int data1, int data2);     // Overload to accept int as argument
void sendData(char data1, char data2);   // Overload to accept char as argument
void onRequest();                        // Code to execute when master requests data from the slave
void onReceive(int len);                 // Code to execute when master sends data to the slave

#pragma region sendData

int roundToNearestMultipleOf20(int value)
{
  // Berechne den Rest der Division durch 20
  int remainder = value % 20;

  // Wenn der Rest größer als die Hälfte von 20 ist, runde auf die nächsthöhere Zahl, die durch 20 teilbar ist
  if (remainder >= 10)
  {
    return value + (20 - remainder);
  }
  else
  {
    // Andernfalls runde auf die nächstniedrigere Zahl, die durch 20 teilbar ist
    return value - remainder;
  }
}

void setMUXPins(int v)
{
  int value = roundToNearestMultipleOf20(v);
  if (value == 0)
  {
    digitalWrite(MUX_SELECT_PIN_0, LOW);
    digitalWrite(MUX_SELECT_PIN_1, LOW);
    digitalWrite(MUX_SELECT_PIN_2, LOW);
  }
  else if (value == 20)
  {
    digitalWrite(MUX_SELECT_PIN_0, HIGH);
    digitalWrite(MUX_SELECT_PIN_1, LOW);
    digitalWrite(MUX_SELECT_PIN_2, LOW);
  }
  else if (value == 40)
  {
    digitalWrite(MUX_SELECT_PIN_0, LOW);
    digitalWrite(MUX_SELECT_PIN_1, HIGH);
    digitalWrite(MUX_SELECT_PIN_2, LOW);
  }
  else if (value == 60)
  {
    digitalWrite(MUX_SELECT_PIN_0, HIGH);
    digitalWrite(MUX_SELECT_PIN_1, HIGH);
    digitalWrite(MUX_SELECT_PIN_2, LOW);
  }
  else if (value == 80)
  {
    digitalWrite(MUX_SELECT_PIN_0, LOW);
    digitalWrite(MUX_SELECT_PIN_1, LOW);
    digitalWrite(MUX_SELECT_PIN_2, HIGH);
  }
  else if (value == 100)
  {
    digitalWrite(MUX_SELECT_PIN_0, HIGH);
    digitalWrite(MUX_SELECT_PIN_1, LOW);
    digitalWrite(MUX_SELECT_PIN_2, HIGH);
  }
  else
  {
    digitalWrite(MUX_SELECT_PIN_0, LOW);
    digitalWrite(MUX_SELECT_PIN_1, LOW);
    digitalWrite(MUX_SELECT_PIN_2, LOW);
  }
}

void sendData(float data1 = 0, float data2 = 0)
{ // Function to send data back to the master
  // Pointer to the float
  uint8_t *bytePointer1 = reinterpret_cast<uint8_t *>(&data1);

  // Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i)
  {
    Wire.write((*bytePointer1));
    bytePointer1++;
  }

  // Pointer to the second float
  uint8_t *bytePointer2 = reinterpret_cast<uint8_t *>(&data2);

  // Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i)
  {
    Wire.write((*bytePointer2));
    bytePointer2++;
  }
}

void sendData(int data1 = 0, int data2 = 0)
{ // Overload to accept int as argument
  sendData((float)data1, (float)data2);
}

void sendData(char data1 = 0, char data2 = 0)
{ // Overload to accept char as argument
  sendData((float)data1, (float)data2);
}

#pragma endregion

#ifdef DEBUG
void blink()
{
  for (char i = 0; i < 10; i++)
  {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
  }
}
#endif

void onRequest()
{ // Code to execute when master requests data from the slave
#ifdef DEBUG
  Serial.println("OnRequest");
  Serial.println(Wire.peek());
  blink();
#endif
  char module = Wire.read(); // Read from which sensor/module the master wants data
  switch (module)
  {
  case Module1:
#ifdef DEBUG
    Serial.println("Module 1 called");
#endif
    // Code to execute when Module1 is being called
    break;
  case Module2:
#ifdef DEBUG
    Serial.println("Module 2 called");
#endif
    // Code to execute when Module2 is being called
    break;
  case Module3:
#ifdef DEBUG
    Serial.println("Module 3 called");
#endif
    // Code to execute when Module3 is being called
    break;
  default:
// Code to execute when unkown module is being called
#ifdef DEBUG
    Serial.println("Unknown module called");
#endif
    break;
  }
}

void onReceive(int len)
{
#ifdef DEBUG
  Serial.println("OnReceive");
  blink();
#endif
  // Code to execute when master sends data to the slave
  char module = Wire.read(); // Read from which sensor/module the master wants to change
  char data = Wire.read();   // Read the data the master wants to send
  setMUXPins(data);

  switch (module)
  {
  case Module1:
#ifdef DEBUG
    Serial.println("Module 1 called");
#endif
    // Code to execute when Module1 is being called
    break;
  case Module2:
#ifdef DEBUG
    Serial.println("Module 2 called");
#endif
    // Code to execute when Module2 is being called
    break;
  case Module3:
#ifdef DEBUG
    Serial.println("Module 3 called");
#endif
    // Code to execute when Module3 is being called
    break;
  default:
// Code to execute when unkown module is being called
#ifdef DEBUG
    Serial.println("Unknown module called");
#endif
    break;
  }
}

void setup()
{
// put your setup code here, to run once:
#ifdef DEBUG
  pinMode(BUILTIN_LED, OUTPUT);
#endif
  Serial.begin(115200);

  pinMode(MUX_SELECT_PIN_0, OUTPUT);
  pinMode(MUX_SELECT_PIN_1, OUTPUT);
  pinMode(MUX_SELECT_PIN_2, OUTPUT);

  Wire.onReceive(onReceive);     // Function to be called when a master sends data to the slave
  Wire.onRequest(onRequest);     // Function to be called when a master requests data from the slave
  Wire.begin((uint8_t)I2C_ADDR); // Register this device as a slave on the i2c-bus (on bus 0)
}

void loop()
{
}
