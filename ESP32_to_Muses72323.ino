/*
   使用ESP32-S3-DevKitC-1模块控制Muses72323电子音量芯片
   By Jerry Long， 10 Feb 2024

   旋转编码器接线：
      CLK（A）——>GPIO37
      DT（B） ——>GPIO36
      SW     ——>GPIO35

   Muses72323芯片SPI接线：
      Latch(20#)——>SDL——>GPIO10(SS)
      Clock(19#)——>SCL——>GPIO12(SCK)
      Data(18#) ——>SDA——>GPIO11(MOSI)

   Muses72323芯片数字供电：
      D_IN(16#) ——>5V
      D_REF(32#)——>D_GND

   Muses72323芯片模拟供电：
      外接±15V电源，地A_GND
*/
#include <SPI.h>

//#define MAX_ATTENUATION -472 //最大衰减量
//#define STEP            0.25 //每步衰减量

//定义Muses72323芯片的地址（根据ADR0-#30和ADR1-#31脚位的高低电位）
const uint8_t MUSES_ADDRESS = 0;


//定义SPI脚位
#define MUSES_CLK 13
#define MUSES_MOSI 11
#define MUSES_CS 10

//定义默认音量，最大和最小音量
const long defaultVolume = 232; //重启后的默认音量 232=-50dB
long currentVolume = 32; //当前音量，从32=0dB开始
const long minVolume = 479; //最小音量479=-111.75dB
const long maxVolume = 32;  //最大音量 32=0dB

//控制信号寄存器参数
const uint8_t musesLeftAtt  = 0b0010000; //控制左声道，Soft step功能打开
const uint8_t musesRightAtt = 0b0010100; //控制右声道，Soft step功能打开
const uint16_t setInternalClock = 0b0000001000001100; //使用内部时钟
const uint16_t setZeroCrossingOn = 0b0000000000001000; //开启Zero Cross检测电路


void setup() {
  //设置串口波特率
  Serial.begin(115200);
  
  //设置CS脚位状态
  pinMode(MUSES_CS, OUTPUT);
  digitalWrite(MUSES_CS, HIGH);
  
  //初始化Muses72323
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));

  //启用Muses72323内部时钟
   // 拉低CS脚位电平，准备写入
  digitalWrite(MUSES_CS, LOW);
  delayMicroseconds(1);   // this can be improved
  //  send in the address and value via SPI
  SPI.transfer(highByte(setInternalClock));
  SPI.transfer(lowByte(setInternalClock));
  
  // 拉高CS脚位电平，完成写入
  delayMicroseconds(1);   // this can be improved
  digitalWrite(MUSES_CS, HIGH);
  SPI.endTransaction();

  //启用Muses72323的Zero Cross检测电路
  // 拉低CS脚位电平，准备写入
  digitalWrite(MUSES_CS, LOW);
  delayMicroseconds(1);   // this can be improved
  //  send in the address and value via SPI
  SPI.transfer(highByte(setZeroCrossingOn));
  SPI.transfer(lowByte(setZeroCrossingOn));
  
  // 拉高CS脚位电平，完成写入
  delayMicroseconds(1);   // this can be improved
  digitalWrite(MUSES_CS, HIGH);
  SPI.endTransaction();

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 32; i<200; i++)
  {
    currentVolume = i;
    Serial.print("currentVolume = ");
    Serial.println(currentVolume);
    musesWriteVolume(currentVolume, currentVolume);
    delay(10);
  }

  for (int i = 200; i>32; i--)
  {
        currentVolume = i;
    Serial.print("currentVolume = ");
    Serial.println(currentVolume);
    musesWriteVolume(currentVolume, currentVolume);
    delay(10);
  }
  // You can continue with the rest of your loop code here.
  delay(100);
}

void musesWriteVolume(long leftVolume, long rightVolume)
{  
  uint8_t address;
  long data;
  //写入左声道寄存器
  address = musesLeftAtt;
  data = leftVolume;
  musesWriteRaw(address, data);
  
  //写入右声道寄存器
  address = musesRightAtt;
  data = rightVolume;
  musesWriteRaw(address, data); 
}

void musesWriteRaw(uint8_t address, long data) {
  uint16_t value = 0; //写入Muses72323芯片的寄存器数值D15~D0；
  uint8_t value1 = 0;
  uint8_t value2 = 0;
  int temp = 0;
    for(int i = 0; i<7; i++) //将地址数值写入data的0~6位
  {
    temp = bitRead(address, i);
    bitWrite(value, i, temp);
  }

  for(int i = 0; i<9; i++) //将音量控制数据写入data的7~15位
  {
    temp = bitRead(data, i);
    bitWrite(value, i+7, temp);
  }
  //通过串口监视器显示写入的数值，测试用
  Serial.print("value:");
  Serial.println(value, BIN);

/*  for(int i=0; i<8; i++)
  {
    temp = bitRead(value, i);
    bitWrite(value1, i, temp);
  }
  Serial.print("value1:");
  Serial.println(value1, BIN);

  for(int i=0; i<8; i++)
  {
    temp = bitRead(value, i+8);
    bitWrite(value2, i, temp);
  }
  Serial.print("value2:");
  Serial.println(value2, BIN);
*/
  // 拉低CS脚位电平，准备写入
  digitalWrite(MUSES_CS, LOW);
  delayMicroseconds(1);   // this can be improved
  //  send in the address and value via SPI
  SPI.transfer(highByte(value));
  SPI.transfer(lowByte(value));
  //SPI.transfer(value2);
  //SPI.transfer(value1);
  
  // 拉高CS脚位电平，完成写入
  delayMicroseconds(1);   // this can be improved
  digitalWrite(MUSES_CS, HIGH);
  SPI.endTransaction();
}
