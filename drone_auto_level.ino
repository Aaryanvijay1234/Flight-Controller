#include<Wire.h>
const int MPU_addr = 0x68;
double pitch_level_adjust, roll_level_adjust;
double pitchInput, rollInput, yawInput, altitudeInput,acc_total_vector,pitchAngle,rollAngle;
double xAcc, yAcc, zAcc, xGyro, yGyro, zGyro,temp;
double currentGyroMillis, previousGyroMillis, deltaGyroTime;
double pitchInputAcc, rollInputAcc, yawInputAcc;
double pitchInputGyro, rollInputGyro, yawInputGyro;
double rollGyroOffset, pitchGyroOffset, yawGyroOffset, rollAccOffset, pitchAccOffset, yawAccOffset;
long timer[5],pulse_timer,pulse = 1000,pulse_start,timer_1,timer_2,timer_3,timer_4,esc_timer;
bool state[4],dir;
int duration[4],esc_1,esc_2,esc_3,esc_4,start,throttle;
float voltage;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_p_gain_roll = 0.012;               //Gain setting for the roll P-controller //0.67
float pid_i_gain_roll = 0.00;               //Gain setting for the roll I-controller //0.002
float pid_d_gain_roll = 0.71;              //Gain setting for the roll D-controller //0.93
int pid_max_roll = 350;                  //Maximum output of the PID-controller (+/-)
float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0 ;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 350;                     //Maximum output of the PID-controller (+/-)

bool auto_level = true;

void calculate_pid();
void setup(){
  Serial.begin(9600);
//  Serial.println("Start");
  Wire.begin();
    DDRD |= B11110000;
    DDRB |= B00110000;
   PCICR |= (1 << PCIE0);  
  PCMSK0 |= (1 << PCINT0);                                                              // set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                              // set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                              // set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);
  digitalWrite(12,HIGH);
  Serial.println("Step1");
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //gyro config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);
  //accelometer config
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
//  Serial.println("Step2");
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);

    xGyro = Wire.read() << 8 | Wire.read();
    yGyro = Wire.read() << 8 | Wire.read();
    zGyro = Wire.read() << 8 | Wire.read();

    rollGyroOffset += yGyro;
    pitchGyroOffset += xGyro;
    yawGyroOffset += zGyro;
  }
  rollGyroOffset = rollGyroOffset / 2000;
  pitchGyroOffset = pitchGyroOffset / 2000;
  yawGyroOffset = yawGyroOffset / 2000;
   
  float volt_read = analogRead(A0);
  voltage = volt_read*14.701986755;
  if(voltage <= 9000) {
    start = 0;
    digitalWrite(12,HIGH);
  }
//  Serial.println("Step3");
  delay(500);
  digitalWrite(12,LOW);
  pulse_timer = micros();
}

void loop(){
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
  float volt_read = analogRead(A0);
  voltage = volt_read*14.701986755;
//  Serial.print("Battery Voltage");
//  Serial.println(voltage);
  if(voltage <= 7000) {
    digitalWrite(12,HIGH);
  }
  if(duration[2] < 1125 and duration[3] < 1400) start = 1;
  if(start == 1 and duration[2] < 1125 and duration[3] > 1400 and duration[3] < 1500) start = 2;
  if(start == 2 and duration[2] < 1125 and duration[3] > 1500) start = 0;
 
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(duration[0] > 1458)pid_roll_setpoint = duration[0] - 1458;
  else if(duration[0] < 1442)pid_roll_setpoint = duration[0] - 1442;
  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(duration[1] > 1458)pid_pitch_setpoint = duration[1] - 1458;
  else if(duration[1]  < 1442)pid_pitch_setpoint = duration[1] - 1442;
  pid_pitch_setpoint += pitch_level_adjust;
  pid_pitch_setpoint /= -3;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(duration[2] > 1150){ //Do not yaw when turning off the motors.
    if(duration[3]  > 1470)pid_yaw_setpoint = (duration[3] - 1470)/3.0;
    else if(duration[3] < 1430)pid_yaw_setpoint = (duration[3] - 1430)/3.0;
  }
//  Serial.print("Pitch_setpoint: ");
//  Serial.print(pid_pitch_setpoint);
//  Serial.print("\t\tRoll_setpoint: ");
//  Serial.println(pid_roll_setpoint);
 
  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.
  throttle = duration[2];
  if(throttle < 1100) throttle = 1100;
  if(throttle > 1800) throttle = 1800;
  if(start == 2) {                                
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
   
//    if(voltage > 7000 and voltage < 11100) {
//      esc_1 += esc_1*((11100 - voltage)/3700);
//      esc_2 += esc_2*((11100 - voltage)/3700);
//      esc_3 += esc_3*((11100 - voltage)/3700);
//      esc_4 += esc_4*((11100 - voltage)/3700);
//    }
    if(esc_1 < 1150) esc_1 = 1150;
    if(esc_2 < 1150) esc_2 = 1150;
    if(esc_3 < 1150) esc_3 = 1150;
    if(esc_4 < 1150) esc_4 = 1150;

    if(esc_1 > 1850) esc_1 = 1800;
    if(esc_2 > 1850) esc_2 = 1800;
    if(esc_3 > 1850) esc_3 = 1800;
    if(esc_4 > 1850) esc_4 = 1800;
  }
  else {
    esc_1 = 1100;
    esc_2 = 1100;
    esc_3 = 1100;
    esc_4 = 1100;
  }
  if(micros() - pulse_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  while(micros() - pulse_timer < 4000);
  pulse_timer = micros();
  PORTD |= B11110000;
  timer_1 = esc_1 + pulse_timer;
  timer_2 = esc_2 + pulse_timer;
  timer_3 = esc_3 + pulse_timer;
  timer_4 = esc_4 + pulse_timer;
 
  while (PORTD >= 16) {
    esc_timer = micros();
    if(timer_1 <= esc_timer)PORTD &= B11101111;
    if(timer_2 <= esc_timer)PORTD &= B11011111;
    if(timer_3 <= esc_timer)PORTD &= B10111111;
    if(timer_4 <= esc_timer)PORTD &= B01111111;
  }
  gyro();
   
//  Serial.print("Remote: "); print_rem();
//   Serial.print("motors: "); print_esc();
  // Serial.print("GyroAc: "); print_gyr();
  // Serial.print("pidCal: "); print_pid();
  // Serial.print("SetPnt: "); print_psp();
  // Serial.println();
}
ISR(PCINT0_vect) {
//  for channel 1
  timer[0] = micros();
  if(state[0] == 0 && (PINB & B00000001)) {
    state[0] = 1;
    timer[1] = timer[0];
   
  }
  else if(state[0] == 1 && !(PINB & B00000001)) {
    state[0] = 0;
    duration[0] = timer[0]- timer[1];
  }


  //  for channel 2

  if(state[1] == 0 && (PINB & B00000010)) {
    state[1] = 1;
    timer[2] = timer[0];
   
  }
  else if(state[1] == 1 && !(PINB & B00000010)) {
    state[1] = 0;
    duration[1] = timer[0]- timer[2];
  }
    //  for channel 3

  if(state[2] == 0 && (PINB & B00000100)) {
    state[2] = 1;
    timer[3] = timer[0];
   
  }
  else if(state[2] == 1 && !(PINB & B00000100)) {
    state[2] = 0;
    duration[2] = timer[0]- timer[3];
  }
      //  for channel 4
 
  if(state[3] == 0 && (PINB & B00001000)) {
    state[3] = 1;
    timer[4] = timer[0];
   
  }
  else if(state[3] == 1 && !(PINB & B00001000)) {
    state[3] = 0;
    duration[3] = timer[0]- timer[4];
  }
}
void print_psp(){
  Serial.print(pid_pitch_setpoint);
      Serial.print("  -  ");
  Serial.print(pid_roll_setpoint);
       Serial.print("  -  ");
   Serial.println(pid_yaw_setpoint);
}
void print_rem(){
  Serial.print(duration[0]);
      Serial.print("  -  ");
  Serial.print(duration[1]);
       Serial.print("  -  ");
   Serial.print(duration[2]);
        Serial.print("  -  ");
  Serial.println(duration[3]);
}
void print_esc(){
  Serial.print(esc_1);
      Serial.print("  -  ");
  Serial.print(esc_2);
       Serial.print("  -  ");
  Serial.print(esc_3);
        Serial.print("  -  ");
  Serial.println(esc_4);
}
void print_gyr(){
  Serial.print(pitchInputGyro);
      Serial.print("  -  ");
  Serial.print(rollInputGyro);
      Serial.print("  -  ");
  Serial.println(yawInputGyro);
}
void print_pid(){
  Serial.print(pid_output_pitch);
      Serial.print("  -  ");
  Serial.print(pid_output_roll);
      Serial.print("  -  ");
  Serial.println(pid_output_yaw);
}
void gyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  xAcc = (Wire.read() << 8 | Wire.read());
  yAcc = (Wire.read() << 8 | Wire.read());
  zAcc = (Wire.read() << 8 | Wire.read());
  temp = (Wire.read() << 8 | Wire.read());
  xGyro = Wire.read() << 8 | Wire.read();
  yGyro = Wire.read() << 8 | Wire.read();
  zGyro = Wire.read() << 8 | Wire.read();

  xGyro = xGyro - pitchGyroOffset;
  yGyro = yGyro - rollGyroOffset;
  zGyro = zGyro - yawGyroOffset;

  pitchInputGyro = (pitchInputGyro*0.7) + ((xGyro / 65.5)*0.3) ;
  rollInputGyro =  (rollInputGyro * 0.7) + ((yGyro / 65.5) * 0.3);
  yawInputGyro = (yawInputGyro * 0.7) + ((zGyro / 65.5) * 0.3);
 
//  pitchInputGyro*=-1;
//  rollInputGyro*=-1;
  yawInputGyro*=-1;

  pitchInput += xGyro * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  rollInput += yGyro * 0.0000611;  

  pitchInput -= rollInput * sin(zGyro * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  rollInput += pitchInput * sin(zGyro * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
  acc_total_vector = sqrt((xAcc*xAcc)+(yAcc*yAcc)+(zAcc*zAcc));       //Calculate the total accelerometer vector.
 
  if(abs(yAcc) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    pitchInputAcc = asin((float)yAcc/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(xAcc) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    rollInputAcc = asin((float)xAcc/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
 
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  pitchInputAcc -= 2.025;                                                  //Accelerometer calibration value for pitch.
  rollInputAcc += 4.974;                                                    //Accelerometer calibration value for roll.
  if(rollInputAcc<0.4 and rollInputAcc>-0.4) rollInputAcc=0;
  pitchInputAcc = (pitchInputAcc<0.4 and pitchInputAcc>-0.4)? 0: pitchInputAcc;
 
  pitchInput = pitchInput * 0.9996 + pitchInputAcc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  rollInput = rollInput * 0.9996 + rollInputAcc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = pitchInput * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = rollInput * 15;                                      //Calculate the roll angle correction

//  Serial.print("Angle_Pitch: ");
//  Serial.print(pitchAngle);
//  Serial.print("\t");
//  Serial.print("Angle_Roll: ");
//  Serial.println(rollAngle);
//
  Serial.print("Pitch_Input: ");
  Serial.print(pitchInput);
  Serial.print("\t");
  Serial.print("Roll_Input: ");
  Serial.println(rollInput);
//
//  Serial.print("Acc_Pitch: ");
//  Serial.print(pitchInputAcc);
//  Serial.print("\t");
//  Serial.print("Acc_Roll: ");
//  Serial.println(rollInputAcc);
 
  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }
//
//  //complementary filter
//  rollInput = 0.98 * (rollInput + rollInputGyro) + 0.02 * (rollInputAcc);
//  pitchInput = 0.98 * (pitchInput + pitchInputGyro) + 0.02 * (pitchInputAcc);
//  yawInputAcc = atan2((sin(rollInput) * cos(pitchInput) * xAcc + sin(pitchInput) * yAcc + cos(rollInput) * cos(pitchInput) * zAcc), sqrt(pow(sin(rollInput) * sin(pitchInput) * xAcc - cos(rollInput) * sin(pitchInput) * zAcc, 2) + pow(cos(pitchInput) * xAcc, 2))) - 1;
//  yawInput = 0.98 * (yawInput + yawInputGyro) + 0.02 * (yawInputAcc);
}
void calculate_pid(){
  //Roll calculations
  pid_error_temp = rollInputGyro - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitchInputGyro - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = yawInputGyro - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
