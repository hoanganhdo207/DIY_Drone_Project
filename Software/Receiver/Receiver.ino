#include <SPI.h> 
#include <nRF24L01.h> 
#include <RF24.h>
#include <Servo.h> 

Servo ESC1 ; 
Servo ESC2 ; 
Servo ESC3 ; 
Servo ESC4 ; 
int input_1 , input_2 , input_3 , input_4 ; 
RF24 radio(9,10); //CE , CSN 
const uint64_t diachi = 0xF9E8F0F0E1LL ; //địa chỉ truyền 
struct data_receiver 
{
  byte ch1 ; 
  byte ch2 ; 
  byte ch3 ; 
  byte ch4 ; 
};
data_receiver data ;

void setup() {
  Serial.begin(9600) ; 
  if (!radio.begin()) 
    Serial.println("Module chưa khởi động") ; 

  radio.openReadingPipe(1,diachi) ; 
  radio.setPALevel(RF24_PA_MIN) ; 
  radio.setChannel(80) ; 
  radio.setDataRate(RF24_250KBPS) ; 
  radio.startListening() ; 
  if (!radio.available()) 
    Serial.println("chờ kết nối") ; 
  ESC1.attach(4) ; 
  ESC2.attach(5) ; 
  ESC3.attach(6) ; 
  ESC4.attach(7) ; 

  data.ch1 = 127 ; //reset gía trị của joystick 
  data.ch2 = 127 ;
  data.ch3 = 127 ;
  data.ch4 = 127 ;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (radio.available()) //check xem có data gửi đến k 
  {
    radio.read(&data , sizeof(data_receiver)) ; //đọc data từ bộ nhớ đệm NRF
    input_1 = map(data.ch1 , 0 , 255 , 1000 , 2000) ; 
    input_2 = map(data.ch2 , 0 , 255 , 1000 , 2000) ;                                         
    input_3 = map(data.ch3 , 0 , 255 , 1000 , 2000) ; 
    input_4 = map(data.ch4 , 0 , 255 , 1000 , 2000) ; 
  }
  Serial.println(input_3) ; 
  ESC1.writeMicroseconds(input_1) ; 
  ESC2.writeMicroseconds(input_2) ;
  ESC3.writeMicroseconds(input_3) ;
  ESC4.writeMicroseconds(input_4) ;
}
