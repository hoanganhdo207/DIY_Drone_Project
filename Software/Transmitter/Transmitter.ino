/*
VCC - 3,3V 
GND 
CSN 9
CE 8
SCK 13 
MOSI 11
MISO 12 
*/ 
#include <RF24.h> 
#include <SPI.h> 
#include <nRF24L01.h> 

RF24  radio(8,9) ; //CE,CSN 
const uint64_t diachi = 0xF9E8F0F0E1LL;
struct data_sent
{
  byte ch1 ; //
  byte ch2 ; //phải Y
  byte ch3 ; //trái Y 
  byte ch4 ; //trái X 
} ; 

data_sent data ; 

/*
tại sao cần ajust joystick? vì khoảng cách từ 500-530 rất nhiễu nên cần deadband 
*/
int ajust_joy(int value) 
{
  if (value >= 530)   //deadband
    value = map (value , 530 , 1023 , 127 , 255 ) ; //map giá trị thành 1 byte 

  else if (value <= 500) //deadband
    value = map (value , 0 , 500 , 0 , 127) ; 

  else 
    value = 127 ; 

  return value ; 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600) ; 
  radio.begin();
  radio.openWritingPipe(diachi);  //ghi dữ liệu lên địa chỉ qua kênh 0 
  radio.setPALevel(RF24_PA_MIN); //khuếch đại công suất => cần nguồn 3.3V ổn định mới để mức cao
  radio.setDataRate(RF24_250KBPS); //Tốc độ truyền dữ liệu trong không khí
  radio.setChannel(80); //set kênh nếu có trùng người sử dụng 
  radio.stopListening() ; //cài đặt module này là TX 
  if (!radio.available()) 
    Serial.println("chờ kết nối") ; 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  data.ch1 = ajust_joy(analogRead(A0)) ; //right Y 
  data.ch2 = ajust_joy(analogRead(A1)) ; //right X
  data.ch3 = ajust_joy(analogRead(A3)) ; //left Y 
  data.ch4 = ajust_joy(analogRead(A2)) ; //left X 
  radio.write(&data , sizeof(data_sent)) ; //viết vào bộ nhớ đệm của module 
  Serial.print(data.ch3) ;  
  Serial.println(data.ch4);
}
