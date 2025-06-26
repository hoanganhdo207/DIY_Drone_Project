#include <Wire.h>                      
#include <EEPROM.h>                       

//PID gain and limit settings
float pid_p_gain_roll = 1.3;            
float pid_i_gain_roll = 0.04;              
float pid_d_gain_roll = 18.0;             
int pid_max_roll = 400;                   

float pid_p_gain_pitch = pid_p_gain_roll;  
float pid_i_gain_pitch = pid_i_gain_roll;  
float pid_d_gain_pitch = pid_d_gain_roll;  
int pid_max_pitch = pid_max_roll;          

float pid_p_gain_yaw = 4.0;              
float pid_i_gain_yaw = 0.02;               
float pid_d_gain_yaw = 0.0;            
int pid_max_yaw = 400;                 

boolean auto_level = true;             


//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

void setup(){
  //Serial.begin(57600);
  //Copy the EEPROM data for fast access data.
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                
  gyro_address = eeprom_data[32];                                         

  Wire.begin();                                                           

  TWBR = 12;                                                       
  DDRD |= B11110000;                                                        
  DDRB |= B00110000;                                                       

  //Use the led on the Arduino for startup indication.
  digitalWrite(12,HIGH);                                                  
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);
 
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                  

  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                        
    PORTD |= B11110000;                                                   
    delayMicroseconds(1000);                                                
    PORTD &= B00001111;                                                    
    delayMicroseconds(3000);                                                
  }
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                          
    if(cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));               
    gyro_signalen();                                                      
    gyro_axis_cal[1] += gyro_axis[1];                                      
    gyro_axis_cal[2] += gyro_axis[2];                                      
    gyro_axis_cal[3] += gyro_axis[3];                                      
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  gyro_axis_cal[1] /= 2000;                                                
  gyro_axis_cal[2] /= 2000;                                             
  gyro_axis_cal[3] /= 2000;                                               

  PCICR |= (1 << PCIE0);                                                  
  PCMSK0 |= (1 << PCINT0);                                                
  PCMSK0 |= (1 << PCINT1);                                               
  PCMSK0 |= (1 << PCINT2);                                               
  PCMSK0 |= (1 << PCINT3);                                               

  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.

  battery_voltage = (analogRead(0) + 65) * 1.2317;
  loop_timer = micros();                                                    //Set the timer for the next loop.
  digitalWrite(12,LOW);                                                     //Turn off the warning led.
}

void loop(){
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }


  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;                                         
    angle_roll = angle_roll_acc;                                        
    gyro_angles_set = true;                                             

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;
  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                
  pid_roll_setpoint /= 3.0;                                               


  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  
  pid_pitch_setpoint /= 3.0;                                                

  pid_yaw_setpoint = 0;
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  
  calculate_pid();                                                          
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);


  throttle = receiver_input_channel_3;                                      
  if (start == 2){                                                        
    if (throttle > 1800) throttle = 1800;                                 
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
    
  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  while(micros() - loop_timer < 4000);                                     
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  gyro_signalen();

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}

void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                               
    Wire.write(0x3B);                                                   
    Wire.endTransmission();                                                 
    Wire.requestFrom(gyro_address,14);                                    
    
    receiver_input_channel_1 = convert_receiver_channel(1);              
    receiver_input_channel_2 = convert_receiver_channel(2);                
    receiver_input_channel_3 = convert_receiver_channel(3);                 
    receiver_input_channel_4 = convert_receiver_channel(4);            
    
    while(Wire.available() < 14);                                          
    acc_axis[1] = Wire.read()<<8|Wire.read();                              
    acc_axis[2] = Wire.read()<<8|Wire.read();                            
    acc_axis[3] = Wire.read()<<8|Wire.read();                              
    temperature = Wire.read()<<8|Wire.read();                            
    gyro_axis[1] = Wire.read()<<8|Wire.read();                            
    gyro_axis[2] = Wire.read()<<8|Wire.read();                             
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                    
    gyro_axis[2] -= gyro_axis_cal[2];                                     
    gyro_axis[3] -= gyro_axis_cal[3];                                      
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                     
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                         
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                  
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                    
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                          

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                         
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                          
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                      
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                          
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                        
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                            
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                          
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                    
  else reverse = 0;                                                            

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14]; 
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; 
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];  

  if(actual < center){                                                        
    if(actual < low)actual = low;                                             
    difference = ((long)(center - actual) * (long)500) / (center - low);       
    if(reverse == 1)return 1500 + difference;                                  
    else return 1500 - difference;                                            
  }
  else if(actual > center){                                                            
    if(actual > high)actual = high;                                           
    difference = ((long)(actual - center) * (long)500) / (high - center);     
    if(reverse == 1)return 1500 - difference;                                  
    else return 1500 + difference;                                         
  }
  else return 1500;
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                      
    Wire.write(0x6B);                                                    
    Wire.write(0x00);                                                      
    Wire.endTransmission();                                                    

    Wire.beginTransmission(gyro_address);                                    
    Wire.write(0x1B);                                                        
    Wire.write(0x08);                                                       
    Wire.endTransmission();                                                

    Wire.beginTransmission(gyro_address);                                    
    Wire.write(0x1C);                                                      
    Wire.write(0x10);                                                       
    Wire.endTransmission();                                               

    Wire.beginTransmission(gyro_address);                             
    Wire.write(0x1B);                                                         
    Wire.endTransmission();                                                  
    Wire.requestFrom(gyro_address, 1);                                       
    while(Wire.available() < 1);                                              
    if(Wire.read() != 0x08){                                                 
      digitalWrite(12,HIGH);                                                   
      while(1)delay(10);                                                      
    }

    Wire.beginTransmission(gyro_address);                                    
    Wire.write(0x1A);                                                      
    Wire.write(0x03);                                                         
    Wire.endTransmission();                                                   

  }  
}

