/*
 * This work is licensed under the Creative Commons 
 * Attribution-NonCommercial 4.0 International License. 
 * http://creativecommons.org/licenses/by-nc/4.0/ 
 * 
 * Digital Volume Control with NJR MUSES72323  
 * Implementation for ESP32 using Arduino IDE
 * 
 * The connections: (standard SPI pins on ESP)
 *   D_REF   (MUSES72323 pin 32) - to ESP32 GND via a 10kOhm resistor
 *   D_IN   (MUSES72323 pin 16) - to +5V via a 10kOhm resistor
 *   CLOCK  (MUSES72323 pin 19) - to ESP32 pin 18 (SCK pin)
 *   DATA   (MUSES72323 pin 18) - to ESP32 pin 23 (MOSI pin)
 *   LATCH  (MUSES72323 pin 20) - to ESP32 pin  5 (SS pin)
 *   
 *  created 05 Jan 2024         
 *  by alexcp
 *  https://github.com/jerrylong-km/Muses72323_ESP32
 *
 *  Modified for ESP32 by Jerry Long
 *  5 Jan 2024  Shared on diyaudio.com
 *  Tested with ESP32 S3 DevKit-C1
 *
 *  Changes:
 *  只包含衰减功能，因为用于OFZZ RF5 胆前级，不需要增益.
 *  通过旋转编码器控制音量.
 */

// include the SPI library:
#include <SPI.h>

// Chip address must correspond to the state of MUSES72323 pins 30..31
const uint8_t musesChipAddress = 0;

// define SPI pins
#define MUSES_CLK  18  // Clock         (SCK)
#define MUSES_MOSI 23  // MOSI          (MOSI) 
#define MUSES_CS    5  // Chip Select   (SS)

// define zcenb and csb pins for the MUSES72323
const int zcenPin = 27;  // zero crossing enable, active low
//const int csbPin = MUSES_CS;  // chip select, active low

// define the default volume after reset, as well as max and min limits
const long defaultVolume = -105;  // start up low volume (for demo)
const long minVolume = -230;      // 203 = -50.5dB
const long maxVolume = -1;        // = 0 dB

// MUSES72323 register addresses
// Each register is 16 bit
// Bits 15..8 contain the data (volume setting or control bits)
// Bits 7..4 are register address
// Bits 3..0 are chip address, corresponding to the state of MUSES72320 pins 29..31.
const uint8_t musesLeftAtt   = 0;       // 左声道衰减
const uint8_t musesRightAtt  = 0x02<<4; // 右声道衰减（100000）
const uint8_t musesControl   = 0x04<<4; // 控制位（1000000）
const uint8_t musesRegMask   = 0x07<<4; // 寄存器地址掩码（1110000）
const uint8_t musesZcBit     = 0x01<<5; // set bit 5 LOW in the data word and set zcenPin LOW to enable zero crossing detection
const uint8_t musesLRAtt     = 0x01<<7; // set bit 7 HIGH to link right channel attenuation to left channel control register

void setup() {
  Serial.begin(115200);
  // set the CSB as an output   	it works without this! ?
  pinMode(MUSES_CS, OUTPUT);
  digitalWrite(MUSES_CS, HIGH);  
  // set the ZCEN pin as an output and activate it   	it works without this! ?
  pinMode(zcenPin, OUTPUT);
  digitalWrite(zcenPin, LOW);  
  // initialize SPI
  SPI.begin();
  // MUSES72323 SPI interface is rated at 250kHz maximum
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  // on reset, MUSES72320 is muted, so we set the default volume
  musesWriteVolume(defaultVolume, defaultVolume);
}

void loop()
{
  delay(3000); 
  musesWriteVolume(-80, -80);
}

uint8_t volumeToAtt(long volume) {
  uint8_t att;
  volume = constrain(volume, minVolume, maxVolume);
  if (0 < volume) { 
    // no attenuation
    att = 0x10; // 0dB
  } else if (minVolume == volume) { 
    // mute        maybe don't do this - handle mute elsewhere
    att = 0xFF; // Mute
  } else { 
    // attenuation from 0 db (volume = 0) to -115.5 dbB (volume == -447)
    volume--;
    volume >>= 1;
    volume = ~volume;
    att = volume & 0xFF; 
    att += 0x10;
  }
  return att;
}

void musesWriteVolume(long rightVolume, long leftVolume) {  
  uint8_t address, data;
  Serial.print("Vol = "); Serial.print(leftVolume);      // not needed
  address = musesLeftAtt | (musesChipAddress & ~musesRegMask);
  data = volumeToAtt(leftVolume);
  musesWriteRaw(address, data);
  Serial.print(" att = "); Serial.println(data);         // not needed
  address = musesRightAtt | (musesChipAddress & ~musesRegMask);
  data = volumeToAtt(rightVolume);
  musesWriteRaw(address, data); 
}

void musesWriteRaw(uint8_t address, uint8_t data) {
  // take the CSB pin low to select the chip
  digitalWrite(MUSES_CS, LOW);
  delayMicroseconds(1);   // this can be improved
  //  send in the address and value via SPI
  SPI.transfer(data);
  uint8_t result = SPI.transfer(address);  
  // take the CSB pin high to de-select the chip
  delayMicroseconds(1);   // this can be improved
  digitalWrite(MUSES_CS, HIGH);
}

