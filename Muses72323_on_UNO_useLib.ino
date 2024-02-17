/*
   使用Arduino UNO控制Muses72323电子音量芯片
   使用Christoffer Hjalmarsson的Muses72323库
   By Jerry Long， 10 Feb 2024

   Muses72323芯片SPI接线：
      Latch(20#)——>SDL——>D10(SS)
      Clock(19#)——>SCL——>D13(SCK)
      Data(18#) ——>SDA——>D11(MOSI)

   Muses72323芯片数字供电：
      D_IN(16#) ——>5V
      D_REF(32#)——>D_GND

   Muses72323芯片模拟供电：
      外接±15V电源，地A_GND
*/
#include <Muses72323.h>

//定义Muses72323芯片的地址（根据ADR0-#30和ADR1-#31脚位的高低电位）
static const byte MUSES_ADDRESS = 0;

static Muses72323 Muses(MUSES_ADDRESS); //根据上述芯片地址创建Muses72323对象


//定义默认音量，当前音量，最大和最小音量
static Muses72323::volume_t defaultVolume = -50; //重启后的默认音量-50dB
static Muses72323::volume_t currentVolume = -20; //当前音量，-20dB开始
int MaxVolume = 0; //最大音量0dB
int MinVolume = -111.75; //最小音量-111dB



void setup() {
  //设置串口波特率
  Serial.begin(115200);
  
  //初始化Muses72323
  Muses.begin();  
  
  //设置Muses72323状态
  Muses.setExternalClock(false); //不使用外部时钟，使用芯片内置的时钟
  Muses.setZeroCrossingOn(true); //启用ZeroCrossing功能

  //启动时设置为默认音量
  //Muses.setVolume(defaultVolume, defaultVolume);

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = MaxVolume; i>MinVolume; i--)
  {
    currentVolume = i;
    Serial.print("currentVolume = ");
    Serial.println(currentVolume);
    Muses.setVolume(currentVolume, currentVolume);
    delay(30);
  }

  for (int i = MinVolume; i<MaxVolume; i++)
  {
    currentVolume = i;
    Serial.print("currentVolume = ");
    Serial.println(currentVolume);
    Muses.setVolume(currentVolume, currentVolume);
    delay(30);
  }
  // You can continue with the rest of your loop code here.
  delay(100);
}
