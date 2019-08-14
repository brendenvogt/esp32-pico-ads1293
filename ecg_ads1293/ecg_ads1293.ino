/*

  Simplest 3-Lead 24-bit ECG with an ads1293 and esp32

  SPI conection:

  ads1293     Esp32 Pico
  DRDYB       D4
  SDO         D12 (slave out - master in )
  SDI         D13 (slave in  - master out)
  SCLK        D14
  CSB         D15
  ALARMB      D2

*/

// #include "Plotter.h"
// Plotter p;

#include <SPI.h>

SPIClass hspi(HSPI);
static const int spiClk = 1000000; // 1 MHz

const int pin_DRDYB = 4;  // data ready
const int pin_ALARMB = 2; // alarm
const int pin_MISO = 12;  // MISO
const int pin_MOSI = 13;  // MOSI
const int pin_SCLK = 14;  // SCLK
const int pin_SS = 15;    // CSB

int32_t c1;
int32_t c2;
int32_t c3;

double cd1;
double cd2;
double cd3;

int32_t getValFromChannel(int channel)
{
  byte x1;
  byte x2;
  byte x3;

  switch (channel)
  {
  case 1:
    x1 = 0x37;
    x2 = 0x38;
    x3 = 0x39;
    break;
  case 2:
    x1 = 0x3A;
    x2 = 0x3B;
    x3 = 0x3C;
    break;
  case 3:
    x1 = 0x3D;
    x2 = 0x3E;
    x3 = 0x3F;
    break;
  }
  int32_t val;

  // 3 8-bit registers combination on a 24 bit number
  val = readRegister(x1);
  val = (val << 8) | readRegister(x2);
  val = (val << 8) | readRegister(x3);

  return val;
}

void setup_ECG_2_channel()
{
  // datasheet ads1293
  //Follow the next steps to configure the device for this example, starting from default registers values.
  //1. Set address 0x01 = 0x11: Connect channel 1’s INP to IN2 and INN to IN1.
  writeRegister(0x01, 0x11);
  //2. Set address 0x02 = 0x19: Connect channel 2’s INP to IN3 and INN to IN1.
  writeRegister(0x02, 0x19);
  //3. Set address 0x0A = 0x07: Enable the common-mode detector on input pins IN1, IN2 and IN3.
  writeRegister(0x0A, 0x07);
  //4. Set address 0x0C = 0x04: Connect the output of the RLD amplifier internally to pin IN4.
  writeRegister(0x0C, 0x04);
  //5. Set address 0x12 = 0x04: Use external crystal and feed the internal oscillator's output to the digital.
  writeRegister(0x12, 0x04);
  //6. Set address 0x14 = 0x24: Shuts down unused channel 3’s signal path.
  writeRegister(0x14, 0x24);
  //7. Set address 0x21 = 0x02: Configures the R2 decimation rate as 5 for all channels.
  writeRegister(0x21, 0x02);
  //8. Set address 0x22 = 0x02: Configures the R3 decimation rate as 6 for channel 1.
  writeRegister(0x22, 0x02);
  //9. Set address 0x23 = 0x02: Configures the R3 decimation rate as 6 for channel 2.
  writeRegister(0x23, 0x02);
  //10. Set address 0x27 = 0x08: Configures the DRDYB source to channel 1 ECG (or fastest channel).
  writeRegister(0x27, 0x08);
  //11. Set address 0x2F = 0x30: Enables channel 1 ECG and channel 2 ECG for loop read-back mode.
  writeRegister(0x2F, 0x30);
  //12. Set address 0x00 = 0x01: Starts data conversion.
  writeRegister(0x00, 0x01);
}

void setup_ECG_3_channel()
{
  // datasheet ads1293
  //Follow the next steps to configure the device for this example, starting from default registers values.
  //1. Set address 0x01 = 0x11: Connect channel 1’s INP to IN2 and INN to IN1.
  writeRegister(0x01, 0x11);
  //2. Set address 0x02 = 0x19: Connect channel 2’s INP to IN3 and INN to IN1.
  writeRegister(0x02, 0x19);

  writeRegister(0x03, 0x2E); //diff

  //3. Set address 0x0A = 0x07: Enable the common-mode detector on input pins IN1, IN2 and IN3.
  writeRegister(0x0A, 0x07);
  //4. Set address 0x0C = 0x04: Connect the output of the RLD amplifier internally to pin IN4.
  writeRegister(0x0C, 0x04);

  writeRegister(0x0D, 0x01); //diff
  writeRegister(0x0E, 0x02); //diff
  writeRegister(0x0F, 0x03); //diff

  writeRegister(0x10, 0x01); //diff

  //5. Set address 0x12 = 0x04: Use external crystal and feed the internal oscillator's output to the digital.
  writeRegister(0x12, 0x04);
  // //6. Set address 0x14 = 0x24: Shuts down unused channel 3’s signal path.
  // writeRegister(0x14, 0x24);
  //7. Set address 0x21 = 0x02: Configures the R2 decimation rate as 5 for all channels.
  writeRegister(0x21, 0x02);
  //8. Set address 0x22 = 0x02: Configures the R3 decimation rate as 6 for channel 1.
  writeRegister(0x22, 0x02);
  //9. Set address 0x23 = 0x02: Configures the R3 decimation rate as 6 for channel 2.
  writeRegister(0x23, 0x02);

  writeRegister(0x24, 0x02); //diff

  //10. Set address 0x27 = 0x08: Configures the DRDYB source to channel 1 ECG (or fastest channel).
  writeRegister(0x27, 0x08);
  //11. Set address 0x2F = 0x30: Enables channel 1 ECG and channel 2 ECG for loop read-back mode.
  writeRegister(0x2F, 0x70); //diff
  //12. Set address 0x00 = 0x01: Starts data conversion.
  writeRegister(0x00, 0x01);
}

//===========SPECIALIZED SPI OPTION 1
byte readRegister(byte reg)
{
  byte data;
  reg |= 1 << 7;
  hspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_SS, LOW);
  hspi.transfer(reg);
  data = hspi.transfer(0);
  digitalWrite(pin_SS, HIGH);
  hspi.endTransaction();
  return data;
}

void writeRegister(byte reg, byte data)
{
  reg &= ~(1 << 7);
  hspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_SS, LOW);
  hspi.transfer(reg);
  hspi.transfer(data);
  digitalWrite(pin_SS, HIGH);
  hspi.endTransaction();
}
//===========SPECIALIZED SPI

void setup()
{
  pinMode(pin_DRDYB, INPUT);
  pinMode(pin_ALARMB, INPUT);
  pinMode(pin_SS, OUTPUT);

  Serial.begin(115200);

  //option 1: use hspi specific spi channel
  hspi.begin(pin_SCLK, pin_MISO, pin_MOSI, pin_SS);
  //option 2: use default spi class methods
  // SPI.begin(pin_SCLK, pin_MISO, pin_MOSI, pin_SS);

  setup_ECG_3_channel();

  // p.Begin();
  // p.AddTimeGraph("3 channel graph", 1000, "c1 label", cd1, "c2 label", cd2, "c3 label", cd3);
}

void loop()
{
  // if (digitalRead(pin_ALARMB) == false)
  // {
  //   Serial.println("alarm raised");
  // }
  // if (digitalRead(pin_DRDYB) == false)
  // {

  // sampled data is located at 3 8-bit
  //--CHANNEL 1
  c1 = getValFromChannel(1);
  // cd1 = (double)c1;
  Serial.print(c1);

  Serial.print(",");
  //--CHANNEL 2
  c2 = getValFromChannel(2);
  // cd2 = (double)c2;
  Serial.print(c2);

  Serial.print(",");
  //--CHANNEL 3
  c3 = getValFromChannel(3);
  // cd3 = (double)c3;
  Serial.println(c3);

  // p.Plot();
  // }
  delay(5); //20ms delay
}