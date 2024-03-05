// revised by jsh MAY 19 2023
// implement MPU 6500
// all magnetometer contexts are deleted
// add start trigger
// add Compensator for Roll and Pitch motion

#include "mbed.h"
#include "PwmIn.h"
#include <cstdint>

Serial pc(PB_6, PB_7); //D5, D4 //PB_6, PB_7 tx rx
I2C i2c(PB_4, PA_7); //sda, scl D12, A6
//RF24 NRF24L01(D2, A5, A4, A3, A0);

PwmOut servo1(PA_8); //FastPWM
PwmOut servo2(PA_9); //
PwmOut servo3(PA_10);

//PwmIn Ch1(PB_1);
PwmIn Ch2(PB_0); //pitch PB_0
PwmIn Ch3(PB_1); //yaw                  PwmIn Ch3(PA_3);
PwmIn Ch4(PA_3); //PA_4 PB_7 roll       PwmIn Ch4(PB_7);

int servo_temp = 0; 
float servo_period = 5000; //20000 //5000
float servo_width1 = 1500, servo_width2 = 1500, servo_width3 = 1500;
int servo_width1b = 1500, servo_width2b = 1500, servo_width3b = 1500;

//MPU
float MPU_temp_read;
int16_t MPU_temp_read2;
int32_t t_fine;
int16_t gyro_raw[3], acc_raw[3], mag_raw[3];
float gyro_offset[3]={0.0,0.0,0.0}, acc_offset[3]={0.0,0.0,0.0};
float mag_factory[3];
int16_t angle[2];

//LPS22HHTR

float press_out = 0.0;
float temp_out = 0.0;


float PITCH, ROLL, YAW;
float heading_angle;
float initial_heading;

Timer timer1;
Timer timer2;

DigitalOut MCU_LED(PB_5); //D11
DigitalOut EEPROM_LED(PA_12);

int16_t loop_count = 0, loop_count2 = 0, loop_count3 = 0, loop_count4 = 0, loop_count5 = 0, loop_count_led = 0, temp_led = 1;
int16_t loop_count_led_max = 100;
//PID
float u_p, u_r, u_p2, u_r2, u_y, u_y2;
float e_p = 0, e_r = 0, e_y = 0;
float prev_e_p, prev_e_r, prev_e_y;


//heading control

float yaw_P = 0.0, yaw_D = 0.0; //magnetometer disabled - jsh

//PID Gain Tuning ================================================================================

float KP_p = 1.800 , KD_pout = 0.0, KP_r = 0.840, KD_rout = 0.0; // can fly well without compensator
float KD_pi = 0.200, KD_ri = 0.080;
float KD2_r = 0.01;
float KD2_p = 0.01;

float KD_yi = 0.080, KDD_yi = 0.001;
float KP_yi = 0.200;

float servo_offset_r = +0.0;
float servo_offset_p = +0.0;
float servo_offset_y = 0.0;
float controller_offset[3] = {0.0,0.0,0.0};
//==============================================================================================++

float acc_raw0, acc_raw1, acc_raw2;
float acc_raw0b, acc_raw1b, acc_raw2b;
float gyro_raw0, gyro_raw1, gyro_raw2;
float gyro_raw0b, gyro_raw1b, gyro_raw2b;
float comp_roll, comp_pitch;
float comp_alpha;

float tau = 0.1, delta_t = 0.0025; //tau=0.1
float alpha1, alpha2 = 0.4, alpha3 = 1.0, alpha2b = 0.03, alpha2c = 0.4; //low pass filter coefficient 1 acc raw //2 gyro roll //3 acc angle //2b gyro yaw //2c gyro pitch
float alpha4 = 1;
//float alpha4 = 0.5; //command LPF 0.025
float acc_roll_rad, acc_pitch_rad, acc_roll_deg, acc_pitch_deg, acc_roll_degb, acc_pitch_degb;
float angle_f[2];
float gyro_roll_deg, gyro_pitch_deg, gyro_yaw_deg;
 
float PITCH3, ROLL3;
int16_t  angleY2= 0;
int16_t gpitch, groll, gyaw;


char MPU6500_WhoAmI();
void MPU6500_GET_GYRO(int16_t * destination);
void MPU6500_GET_ACC(int16_t * destination);
int16_t MPU6500_GET_TEMP();
void MPU6500_STBY();
void MPU6500_INIT();
void MPU6500_READ();
void offset_compensate();
void low_pass_filter(void);
void kalman_filter_IMU(void);
void calculate_angle(void);
//void calculate_heading();
//void calculate_complementary_heading();
void gyro_heading();

void kalman_filter_ALT();
void PWM_read();
void PID_control();
void servo_drive_yaw();
void RF_TRANS();
void led_blink(int count);


char LPS22HHTR_WhoAmI();
void LPS22HHTR_INIT();
int32_t LPS22HHTR_GET_PRESSURE(int32_t *pressure);
int16_t LPS22HHTR_GET_TEMP(int16_t *temp);
//-----------------------------------------------------------------------------------------
//MPU6500 register      
int   MPU6500_ADDRESS                   = (0x68<<1), //0b01101000 (AD0 low)
      MPU6500_ADDRESS2                  = (0x69<<1), //0b01101001 (AD0 high)
      WHO_AM_I_MPU6500                  = 0x75,

      ACCEL_XOUT_H                       = 0x3B,
      GYRO_XOUT_H                        = 0x43,
      TEMP_OUT_H                         = 0x41,
      PWR_MGMT_1                         = 0x6B,
      GYRO_CONFIG                        = 0x1B,
      ACCEL_CONFIG                       = 0x1C,
      ACCEL_CONFIG2                      = 0x1D,
      SMPLRT_DIV                         = 0x19,
      CONFIG                             = 0x1A,

      GYRO_ACCEL_CONFIG0                 = 0x52;


//-----------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------
//Barometer register (LPS22HHTR)

int LPS22HHTR_ADDRESS = (0x5C<<1),

    WHO_AM_I_LPS22HHTR = 0x0F,

    INTERRUPT_CFG = 0x0B,

    CTRL_REG1 = 0x10,

    
    PRESSURE_OUT_XL = 0x28,
    PRESSURE_OUT_L = 0x29,
    PRESSURE_OUT_H = 0x2A,

    LPS22HHTR_TEMP_OUT_H = 0x2C,
    LPS22HHTR_TEMP_PUT_L = 0x2B,    
    
    PRESSURE_Sensitivity = 4096, // LSB/hPa

    TEMPERATURE_Sensitivity = 100;

//-----------------------------------------------------------------------------------------


int16_t constrain_int16(int16_t x, int16_t min, int16_t max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

int32_t constrain_int32(int32_t x, int32_t min, int32_t max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

float constrain_float(float x, float min, float max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

char MPU6500_WhoAmI(){
    char cmd[1], data_out[1];
    cmd[0] = WHO_AM_I_MPU6500;
    i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
    i2c.read(MPU6500_ADDRESS, data_out, 1, 0);
    return (data_out[0]);
}

void MPU6500_GET_GYRO(int16_t * destination)
{
    char cmd[6];
    cmd[0] = GYRO_XOUT_H;
    
    i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
    i2c.read(MPU6500_ADDRESS, cmd, 6, 0);
    
    destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | (int16_t)cmd[1]);
    destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | (int16_t)cmd[3]);
    destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | (int16_t)cmd[5]);
    
}

void MPU6500_GET_ACC(int16_t * destination)
{
    char cmd[6];
    cmd[0] = ACCEL_XOUT_H;
    
    i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
    i2c.read(MPU6500_ADDRESS, cmd, 6, 0);
    
    destination[0] = ((int16_t)cmd[0] << 8) | (int16_t)cmd[1];
    destination[1] = ((int16_t)cmd[2] << 8) | (int16_t)cmd[3];
    destination[2] = ((int16_t)cmd[4] << 8) | (int16_t)cmd[5];
}

int16_t MPU6500_GET_TEMP()
{
    char cmd[2];
    cmd[0] = TEMP_OUT_H;
    i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
    i2c.read(MPU6500_ADDRESS, cmd, 2, 0);
    return (int16_t)((cmd[0] << 8) | cmd[1]);
}

//-----------------------------------------------------------------------------------------  
void MPU6500_STBY()
{
    char cmd[3];

    cmd[0] = PWR_MGMT_1; 
    cmd[1] = 0b01000000;// sleep mode
    wait(0.1);
    i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
    //pc.printf("MPU 1 \n\r");
    wait(0.1);

}
void MPU6500_INIT() 
{  
  char cmd[3];

  cmd[0] = PWR_MGMT_1;  // reset
  cmd[1] = 0x80;
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 1 \n\r");
  wait(0.2);
  cmd[0] = PWR_MGMT_1;  // 
  cmd[1] = 0x00;
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 1 \n\r");
  wait(0.1);
  cmd[0] = PWR_MGMT_1; // Auto selects the best available clock source –PLL if ready, else use the Internal oscillator
  cmd[1] = 0x01;
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 2 \n\r");
  wait(0.1);

  cmd[0] = CONFIG; //bandwidth 101018
  cmd[1] = 0x03; //3 41 Hz -> 
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 3 \n\r");
  wait(0.1);
  cmd[0] = SMPLRT_DIV; 
  cmd[1] = 0x04;//0x04 //1 kHz divide by 5
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 4 \n\r");
  wait(0.1);
  
  uint8_t c;
  cmd[0] = GYRO_CONFIG; //status
  i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
  i2c.read(MPU6500_ADDRESS, cmd, 1, 0);
  //pc.printf("MPU 5 \n\r");
  c = cmd[0];
    
  cmd[0] = GYRO_CONFIG; 
  cmd[1] = 0b00010000;//0b00010011; 10 1000 11 2000
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 6 \n\r");
  wait(0.1);
   
  cmd[0] = ACCEL_CONFIG;
  i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
  i2c.read(MPU6500_ADDRESS, cmd, 1, 0);
  //pc.printf("MPU 7 \n\r");
  c = cmd[0];
      
  cmd[0] = ACCEL_CONFIG; 
  cmd[1] = c | 0b00010000;//c | 3 << 3;//0b00010011; 10 8g 11 16g 4g
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 8 \n\r");
  wait(0.1);
  
  cmd[0] = ACCEL_CONFIG2;
  i2c.write(MPU6500_ADDRESS, cmd, 1, 1);
  i2c.read(MPU6500_ADDRESS, cmd, 1, 0);
  //pc.printf("MPU 9 \n\r");
  c = cmd[0];
  
  cmd[0] = ACCEL_CONFIG2; 
  cmd[1] = c | 0b00000101; //0b00000101 011 11/10/18
  i2c.write(MPU6500_ADDRESS, cmd, 2, 0);
  //pc.printf("MPU 10 \n\r");
  wait(0.1);
  
}

//------------------------------------------------------------------------------------------------


char LPS22HHTR_WhoAmI()
{
    char cmd[1], data_out[1];
    cmd[0] = WHO_AM_I_LPS22HHTR;
    i2c.write(LPS22HHTR_ADDRESS, cmd, 1, 1);
    i2c.read(LPS22HHTR_ADDRESS, data_out, 1, 0);
    return (data_out[0]) ;

}

void LPS22HHTR_INIT()
{
    char cmd[3];

    cmd[0] = CTRL_REG1;  // output Hz
    cmd[1] = 0x60;
    i2c.write(LPS22HHTR_ADDRESS, cmd, 2, 0);

    // cmd[0] = INTERRUPT_CFG; 
    // cmd[1] = 0b11000000;
    // i2c.write(LPS22HHTR_ADDRESS, cmd, 2, 0);

}


int32_t LPS22HHTR_GET_PRESSURE(int32_t *pressure)
{

    char cmd[1]; char data[3];
    cmd[0]=PRESSURE_OUT_XL;

    i2c.write(LPS22HHTR_ADDRESS, cmd, 1, 0);
    i2c.read(LPS22HHTR_ADDRESS, data, 3, 1);

    *pressure = ((int32_t)data[2] << 16) | ((int32_t)data[1] << 8) | (int32_t)data[0];

}

int16_t LPS22HHTR_GET_TEMP(int16_t *temp)
{
    char cmd[1]; char data[2];
    cmd[0]=LPS22HHTR_TEMP_PUT_L;

    i2c.write(LPS22HHTR_ADDRESS, cmd, 1, 0);
    i2c.read(LPS22HHTR_ADDRESS, data, 2, 1);

    *temp = ((int16_t)data[1] << 8) | (int16_t)data[0];
}


//------------------------------------------------------------------------------------------------

void MPU6500_READ()
{  
    MPU6500_GET_GYRO(gyro_raw);
    MPU6500_GET_ACC(acc_raw);
}   

int16_t prev_distance_mm;

//------------------------------------------------------------------------------------------------  
void offset_compensate(){
        //x to front; y to right
    gyro_raw[0] = gyro_raw[0] - (int16_t)gyro_offset[0]; //roll
    gyro_raw[1] = gyro_raw[1] - (int16_t)gyro_offset[1]; //pitch // -
    gyro_raw[2] = gyro_raw[2] - (int16_t)gyro_offset[2]; //yaw //clockwise + -
    
    acc_raw[0] = acc_raw[0] - (int16_t)acc_offset[0]; 
    acc_raw[1] = acc_raw[1] - (int16_t)acc_offset[1]; //-
    acc_raw[2] = acc_raw[2] - (int16_t)acc_offset[2] + 4096;// ; //going up acc positive 2071
                            //offset makes it zero or gravity gone
                            
    }

//low pass filter
void low_pass_filter(void)
{
    acc_raw0 = (float)acc_raw[0];// / 3;
    acc_raw1 = (float)acc_raw[1];// / 3;
    acc_raw2 = (float)acc_raw[2];// / 3; //change direction?
    
    acc_raw0b = alpha1 * acc_raw0 + (1 - alpha1) * acc_raw0b;
    acc_raw1b = alpha1 * acc_raw1 + (1 - alpha1) * acc_raw1b;
    acc_raw2b = alpha1 * acc_raw2 + (1 - alpha1) * acc_raw2b;
    
    gyro_raw0 = (float)gyro_raw[0];
    gyro_raw1 = (float)gyro_raw[1];
    gyro_raw2 = (float)gyro_raw[2]; //change direction?
    
    gyro_raw0b = alpha2 * gyro_raw0 + (1 - alpha2) * gyro_raw0b;
    gyro_raw1b = alpha2c * gyro_raw1 + (1 - alpha2c) * gyro_raw1b;
    gyro_raw2b = alpha2b * gyro_raw2 + (1 - alpha2b) * gyro_raw2b;
    
    acc_roll_rad = atan2f(acc_raw1b, acc_raw2b); //roll pitch yaw ???
    acc_pitch_rad = atan2f(-acc_raw0b, sqrt((pow(acc_raw1b,2) + pow(acc_raw2b,2))));

    acc_roll_deg = acc_roll_rad * 180.0 / 3.1415f;
    acc_pitch_deg = acc_pitch_rad * 180.0 / 3.1415f; //3.14159265359;

    acc_roll_degb = alpha3 * acc_roll_deg + (1 - alpha3) * acc_roll_degb;
    acc_pitch_degb = alpha3 * acc_pitch_deg + (1 - alpha3) * acc_pitch_degb;

    gyro_roll_deg = ((float)gyro_raw0b) / 32.768f;   //deg/sec //1000 dps / 16.384f;/
    gyro_pitch_deg = ((float)gyro_raw1b) / 32.768f;
    gyro_yaw_deg = ((float)gyro_raw2b) / 32.768f;

}
    
//Kalman Filter
float rateP, newRateP, angleP, P00, P01, P10, P11, yP, newAngleP, sP, K0P, K1P;
float rateR, newRateR, angleR, P200, P201, P210, P211, yR, newAngleR, SR, K20, K21;
float P00temp, P01temp, P200temp, P201temp;
float dt = 0.0025, R_measure = 1.2, Q_angle = 0.01, Q_gyroBias = 0.003, bias_P = 0.00012, biasR = 0.00012; 
void kalman_filter_IMU(void)
{
    newRateP = gyro_pitch_deg;
    newRateR = gyro_roll_deg;
    newAngleP = acc_pitch_degb; //LPF
    newAngleR = acc_roll_degb;
    
    //Kalman Filter (PITCH)
    rateP = newRateP - bias_P; //thetadot - thetabdot //priori estimate
    angleP = angleP + dt * rateP;
    
    P00 = P00 + dt * (dt * P11 - P01 - P10 + Q_angle);
    P01 = P01 - dt * P11;
    P10 = P10 - dt * P11;
    P11 = P11 + Q_gyroBias * dt;

    yP = newAngleP - angleP; //innovation
    sP = P00 + R_measure;

    K0P = P00/sP;
    K1P = P10/sP;

    angleP = angleP + K0P * yP;
    bias_P = bias_P + K1P * yP;
    angle_f[1] = angleP; //estimated pitch angle float

    P00temp = P00;
    P01temp = P01;

    P00 = P00 - K0P * P00temp;
    P01 = P01 - K0P * P01temp;
    P10 = P10 - K1P * P00temp;
    P11 = P11 - K1P * P01temp;

    //Kalman Filter 2 (ROLL)
    rateR = newRateR - biasR; //thetadot - thetabdot
    angleR = angleR + dt * rateR;

    P200 = P200 + dt * (dt * P211 - P201 - P210 + Q_angle);
    P201 = P201 - dt * P211;
    P210 = P210 - dt * P211;
    P211 = P211 + Q_gyroBias * dt;

    yR = newAngleR - angleR;
    SR = P200 + R_measure;

    K20 = P200/SR;
    K21 = P210/SR;

    angleR = angleR + K20 * yR;
    biasR = biasR + K21 * yR;
    angle_f[0] = angleR; //estimated roll float

    P200temp = P200;
    P201temp = P201;

    P200 = P200 - K20 * P200temp;
    P201 = P201 - K20 * P201temp;
    P210 = P210 - K21 * P200temp;
    P211 = P211 - K21 * P201temp;
}
void calculate_angle(void){
    MPU6500_READ();
    offset_compensate();
    low_pass_filter();
    kalman_filter_IMU();
    
    angle[0] = (int16_t)(angle_f[0] * 10); //ROLL
    angle[1] = (int16_t)(angle_f[1] * 10); //PITCH
}

float angleY = 0;
float heading2 = 0.0;

void gyro_heading(){
    
    angleY = angleY + dt * gyro_yaw_deg;
    //if (angleY <= -180) angleY = angleY + 360;
    //if (angleY >= 180) angleY = angleY - 360;
            
    if (YAW > 60) {
        heading2 = heading2 - YAW / 3000;
        //if (heading2 < -175) heading2 = -175;
        //if (heading2 <= -180) heading2 = heading2 + 360;
    } 
    if (YAW < -60) {
        heading2 = heading2 - YAW / 3000;
        //if (heading2 > 175) heading2 = 175;
        //if (heading2 >= 180) heading2 = heading2 - 360;
    }  
}

float delta_p;
float PITCHB, YAWB, ROLLB;

void PWM_read(){
    PITCHB = (Ch2.pulsewidth() * 1000000.0f) - 1500.0f; //ROLL pitch 2
    YAWB = (Ch3.pulsewidth() * 1000000.0f) - 1500.0f; //PITCH yaw 4 
    ROLLB = (Ch4.pulsewidth() * 1000000.0f) - 1500.0f; //YAW roll 3
  
    if (PITCHB >= 500) PITCHB = 500;
    if (YAWB >= 500) YAWB = 0; //change 111119
    if (ROLLB >= 500) ROLLB = 500;
    
    if (PITCHB <= -500) PITCHB = -500;
    if (YAWB <= -500) YAWB = 0;
    if (ROLLB <= -500) ROLLB = -500;
    
    PITCH = alpha4 * PITCHB + (1 - alpha4) * PITCH;
    YAW = alpha4 * YAWB + (1 - alpha4) * YAW;
    ROLL = alpha4 * ROLLB + (1 - alpha4) * ROLL; 
    
    delta_p = abs(angle_f[1]) - 30.0f;
    
    if (delta_p > 0.0f) {
        PITCH = PITCH * (100.0f - 4 * delta_p) / 100.0f;
        }
}


//Control Loop                                    
float temp_e_p;
//float u_y3, state;
float u_y3 = 0.0;
float state = 0.0;


float r_uk0 = 0.0;
float r_uk1 = 0.0;
float r_uk2 = 0.0;
float r_yk0 = 0.0;
float r_yk1 = 0.0;
float r_yk2 = 0.0;

float p_uk0 = 0.0;
float p_uk1 = 0.0;
float p_uk2 = 0.0;
float p_yk0 = 0.0;
float p_yk1 = 0.0;
float p_yk2 = 0.0;

float old_angle_roll = 0.0;
float old_angle_pitch = 0.0;


void PID_control()
{
    //control loop & equation (degree)
    e_p = (float)PITCH * 0.0f - angle_f[1]; //error //angle degree
    e_r = (float)ROLL * 0.0f - angle_f[0]; // 

/*
    //=========compenstaor==============
    // compensator input is including P gain
    // compensator for roll , tf([1 5.04 12.96],[1 0.308 0.0484]) in continuous time
    // discrete time domain : tf([1.0000 -1.98736 0.98744],[1.0000 -1.99923 0.99923])
    r_uk0 = KP_r * e_r;
    r_yk0 = r_uk0 -1.98736*r_uk1 +0.98744*r_uk2 +1.99923*r_yk1 -0.99923*r_yk2;

    // compensator for pitch , tf([1 2.8 4],[1 0.308 0.0484]) in continuous time
    // discrete time domain : tf([1.0000 -1.99300 0.99302],[1.0000 -1.99923 0.99923])
    p_uk0 = KP_p * e_p;
    p_yk0 = p_uk0 -1.99300*p_uk1 +0.99302*p_uk2 +1.99923*p_yk1 -0.99923*p_yk2;

    // update
    r_uk1 = r_uk0;
    r_uk2 = r_uk1;
    r_yk1 = r_yk0;
    r_yk2 = r_yk1;

    p_uk1 = p_uk0;
    p_uk2 = p_uk1;
    p_yk1 = p_yk0;
    p_yk2 = p_yk1;

    u_p = p_yk0 - KD_pi * gyro_pitch_deg;
    u_r = r_yk0 - KD_ri * gyro_roll_deg;
    //=======compenstaor end=========
*/


    //=======original=======
    u_p = KP_p * e_p;// + KD_pout * (e_p - prev_e_p);
    u_p -= KD_pi * gyro_raw1*0.0304878f;
    //u_p -= KD_pi * gyro_pitch_deg; //prev_angle - angle ~1k/16
    //u_p -= 

    u_r = KP_r * e_r;// + KD_rout * (e_r - prev_e_r);
    u_r -= KD_ri * gyro_raw0*0.0304878f;
    //u_r -= KD_ri * gyro_roll_deg;
    //=======original end======


    //(degree/sec)
    e_y = (float)0 - gyro_yaw_deg;// * 0.01744;// rate
    //u_y = KD_yi * e_y + KP_yi * (- heading2 - 0.0);// + KDD_yi * (e_y - prev_e_y); // 
    u_y = KD_yi * e_y + KP_yi * (- heading2 - angleY);// + KDD_yi * (e_y - prev_e_y); //
    u_y3 = u_y * 2;

        
    servo_width1 = 1500.0f + (- (ROLL) - 0) * 0.5f + 2.0 * u_r; // - cause direction -// jsh : *2.0 multiplier on u_r is deleted
    servo_width2 = 1500.0f + (- (PITCH) - 0) * 0.5f - 2.0 * u_p;  //- //change direction because of deltang 0511 // jsh : *2.0 multiplier on u_p is deleted
    servo_width3 = 1500.0f - u_y3; //yaw in degree * 1 not 10
}
    
void servo_drive(){
    
    servo_width1 = constrain_float(servo_width1, 700.0f, 2300.0f); //ROLL
    servo_width2 = constrain_float(servo_width2, 700.0f, 2300.0f); //PITCH
    servo_width3 = constrain_float(servo_width3, 1150.0f, 1850.0f); //YAW 1250 1750
    
    servo_width1b = (int)(servo_width1);
    servo_width2b = (int)(servo_width2);
    servo_width3b = (int)(servo_width3);
    
    servo1.pulsewidth_us(servo_width1b+servo_offset_r); //
    servo2.pulsewidth_us(servo_width2b+servo_offset_p); // pitch
    servo3.pulsewidth_us(servo_width3b+servo_offset_y);
    
    }

void servo_drive_yaw(){
        
        servo_width3 = constrain_float(servo_width3, 1250.0f, 1750.0f); //YAW 1250 1750
        
        //servo_width3b = 5 * ((int)(servo_width3 / 5));
        servo_width3b = ((int)(servo_width3));
            
        servo3.pulsewidth_us(servo_width3b);
    }
    
int p_count, p_st;
float delt_p_count;
int16_t motor_width2;

char BUT1, BUT2;
int rf_fail_count = 0;
int8_t ack_count = 0;
int16_t PITCH2, ROLL2, YAW2;

//------------------------------------------------------------------------------------------------
void led_blink(int count, int interval_ms){
    for(int i = 0; i < count; i++){
        MCU_LED = 0;
        wait_ms(interval_ms);
        MCU_LED = 1;
        wait_ms(interval_ms);
    }
}

void EEPROM_led_blink(int count, int interval_ms)
{
    for(int i = 0; i < count; i++){
        EEPROM_LED = 0;
        wait_ms(interval_ms);
        EEPROM_LED = 1;
        wait_ms(interval_ms);
    }
}

#include "eeprom.h"


 /**
 * @brief Saves sensor data including pressure, roll, pitch, and yaw to EEPROM.
 * 
 * Encodes sensor values into a byte format suitable for EEPROM storage. The 32-bit pressure is split into two 16-bit parts,
 * while the floating-point roll, pitch, and yaw are scaled and converted into 16-bit integers. These are then sequentially
 * stored in the EEPROM. The function manages EEPROM memory addresses to avoid data overlap. If the data write exceeds EEPROM's
 * capacity, it signals an error.
 *
 * Data Layout in EEPROM (each '|' denotes a byte boundary):
 * 
 * | Byte 0-1      | Byte 2-3      | Byte 4-5 | Byte 6-7 | Byte 8-9 |
 * |---------------|---------------|----------|----------|----------|
 * | Pressure High | Pressure Low  | Roll     | Pitch    | Yaw      |
 * 
 * @param pressure 32-bit integer representing the pressure to be saved.
 * @param roll Floating-point representing the roll angle to be saved, scaled and stored as a 16-bit integer.
 * @param pitch Floating-point representing the pitch angle to be saved, scaled and stored as a 16-bit integer.
 * @param yaw Floating-point representing the yaw angle to be saved, scaled and stored as a 16-bit integer.
 * @return int Returns 0 on successful operation, 1 if the next write operation exceeds EEPROM's storage capacity.
 */
// int save_to_eeprom(int32_t pressure, float roll, float pitch, float yaw) {
//     // Static variable to track the current write position in EEPROM.
//     static int eeprom_memory_addr = 0x0000; 

//     // Check if adding another 10 bytes would exceed the EEPROM's addressable range.
//     if (this->eeprom_memory_addr + 10 >= 0x10000) {
//         if (this->tictoc.read_ms()) {
//             this->tictoc.stop();
//             this->tictoc.reset();
//             this->errnum = MEM_FULL;
//         }
//         return -1; // Return an error if there's not enough space left.
//     }

//     if (!this->ready() || this->tictoc.read_ms() < 10) {
//         return -1;
//     }
//     this->tictoc.reset();

//     // Split the 32-bit pressure value into two 16-bit parts.
//     int16_t pressure_high_order = (pressure >> 16) & 0x0000FFFF;
//     int16_t pressure_low_order = pressure & 0x0000FFFF;

//     // Convert floating-point values to 16-bit integers by scaling.
//     int16_t roll_u16 = static_cast<int16_t>(roll * 100);
//     int16_t pitch_u16 = static_cast<int16_t>(pitch * 100);
//     int16_t yaw_u16 = static_cast<int16_t>(yaw * 100);

//     // Use a union to easily convert the array of 16-bit words to an array of bytes.
//     EEPROM::WordToByte wtb;
//     wtb.word_array[0] = pressure_high_order;
//     wtb.word_array[1] = pressure_low_order;
//     wtb.word_array[2] = roll_u16;
//     wtb.word_array[3] = pitch_u16;
//     wtb.word_array[4] = yaw_u16;

//     // page write limitation
//     const int PAGE_SIZE = 128;
//     const int DATA_SIZE = 10;
//     int write_size;

//     if ((this->eeprom_memory_addr / PAGE_SIZE) != ((this->eeprom_memory_addr + DATA_SIZE - 1) / PAGE_SIZE)) {
//         int first_data_size = PAGE_SIZE - (this->eeprom_memory_addr % PAGE_SIZE);
//         int remain_size = DATA_SIZE - first_data_size;
//         int first_ret = 0, second_ret = 0;
//         first_ret = this->write_block_data(
//                         this->eeprom_memory_addr,
//                         wtb.byte_array,
//                         first_data_size);

//         int boundary = (this->eeprom_memory_addr / PAGE_SIZE + 1) * PAGE_SIZE;
//         if (first_ret >= 0) {
//             int loop_count = 0;
//             // max time of write cycle is 5ms
//             while ((second_ret = this->write_block_data(
//                                             boundary,
//                                             wtb.byte_array + first_data_size,
//                                             remain_size)) < 0 
//                 && loop_count++ <= 50) {
//                 wait_us(100);
//             }
//         }
//         write_size = second_ret < 0 ? first_ret : first_ret + second_ret;
//     } else {
//         write_size = this->write_block_data(this->eeprom_memory_addr, wtb.byte_array, 10);
//     }

//     pc.printf("Addr: %05d, pressure: (%5d, %5d), roll_16: %5d, pitch_16: %5d, yaw_16: %5d [ %2d ]\r\n", eeprom_memory_addr, pressure_high_order, pressure_low_order, roll_u16, pitch_u16, yaw_u16, write_size);

//     // Increment the EEPROM memory address for the next write operation.
//     this->eeprom_memory_addr += 10;
//     return 0; // Return 0 to indicate successful operation.
// }

int main() {
    
    pc.baud(230400);
    i2c.frequency(400000); //200k not working

    MCU_LED = 0;
    EEPROM_LED = 0;

    int32_t pressure;
    int16_t temp;

    // EEPROM
    EEPROM eeprom(PB_4, PA_7); 

    // turn on led when eeprom is ready

    // eeprom clear all contents on memory
    eeprom.save_to_eeprom(pc, 1, 2, 3, 4);
    int status = eeprom.get_EEPROM_errnum();
    if (status == EEPROM::NORMAL) {
        EEPROM_led_blink(3, 300);
    } else if (status == EEPROM::NACK) {
        EEPROM_led_blink(1, 300);
    } else {
        EEPROM_led_blink(1, 300);
    }

    // EEPROM_led_blink(5,300);

    //PWM
    servo1.period_us(servo_period); 
    servo2.period_us(servo_period); 
    servo3.period_us(servo_period); 
    
    servo1.pulsewidth_us(servo_width1+servo_offset_r); //
    servo2.pulsewidth_us(servo_width2+servo_offset_p); //
    servo3.pulsewidth_us(servo_width3+servo_offset_y); //
    
    //turning on phase
    // pc.printf("phase 1");    
    
    //MPU6500 initializing
    // pc.printf("phase 2");
    MPU6500_INIT();
    wait_ms(100);
    LPS22HHTR_INIT();
    wait_ms(100);
    
    alpha1 = delta_t/tau;
    /*
    for (int i = 0; i < 300; i++){
    
        MPU6500_READ(); //0.0025
        low_pass_filter();
        // mag filter delted
        for (int k = 0; k < 3; k++){
           
        }
        
        wait_ms(5);
    }
    */
    MCU_LED = 1;
// IMU check
    char who = MPU6500_WhoAmI();
    if(who == 0x70 ||who == 0x71) led_blink(3,300);
    else            led_blink(10,100);

    wait_ms(500);
    MPU6500_STBY();

// waiting for start trigger 
    int trigger_count = 0;
    float trigger_pwm = 0.0;
    int led_trigger = 0;
    while(trigger_count<100){
        trigger_pwm = (Ch4.pulsewidth() * 1000000.0f) - 1500.0f;
        if(trigger_pwm>=150)
            trigger_count +=1;        
        wait(0.01);
        led_trigger++;
        if(led_trigger >= 150) {
            MCU_LED = !MCU_LED;
            led_trigger = 0 ;
        }
    }



    MCU_LED = 1;
    led_blink(5,200);
    wait_ms(500);

    eeprom.init();
    status = eeprom.get_EEPROM_errnum();
    if (status == EEPROM::NORMAL)
        EEPROM_led_blink(3, 300);
    else 
        EEPROM_led_blink(1, 300);

    //please keep controller at zero-state to check controller bias

// pc.printf("phase 3"); // Bias check
    MPU6500_INIT();
    who = MPU6500_WhoAmI();
    if(who == 0x70 ||who == 0x71) led_blink(3,300);
    else            led_blink(10,100);
    for (int k = 0; k < 3; k++){
        gyro_offset[k] = 0.0f;
        acc_offset[k] = 0.0f;
    }    

    initial_heading = 0.0f;
    heading_angle = 0.0;
    
    wait_ms(100);

    LPS22HHTR_INIT();
    wait_ms(100);
    

    for (int i = 0; i < 100; i++){
    
        MPU6500_READ(); 
        low_pass_filter();
        gyro_heading();
        PWM_read();
        initial_heading += angleY; // 아래서 평균값 만들려고 더함
        
        for (int k = 0; k < 3; k++){
            gyro_offset[k] += (float)gyro_raw[k];
            acc_offset[k] += (float)acc_raw[k];
        }

    wait_ms(10);
    // pc.printf("phase 4, ready");
    }

    for (int k = 0; k < 3; k++){
        gyro_offset[k] /= 100.0f;
        acc_offset[k] /= 100.0f;
        controller_offset[k] /=100.0f;
    }
        initial_heading /= 100.0f; // 위에서 100번 더했으니까 100으로 나눔 offset 처럼 
    
    led_blink(4,300);

    //main program starts
    //---------------------------------------------
    int loop_print = 0;
    timer1.start();
    timer2.start(); //timer for print loop
    uint32_t dummy = 0;

    while(1){
        loop_print = loop_print + 1;
        loop_count = loop_count + 1;
        //loop_count2 = loop_count2 + 1;
        loop_count4 = loop_count4 + 1;      // for servo drive RPY - jsh
        loop_count_led = loop_count_led + 1;

        if (loop_count >= 4){
            PWM_read();     //Read remote controller input
            loop_count = 0;
        }
        calculate_angle();
        gyro_heading();
        PID_control();
        LPS22HHTR_GET_PRESSURE(&pressure);
        press_out = (float)pressure/PRESSURE_Sensitivity;
        LPS22HHTR_GET_TEMP(&temp);
        temp_out = (float)temp/TEMPERATURE_Sensitivity;
        

        if (loop_count4 >= 1){ //1//2
            servo_drive();      
            //servo_drive_yaw();
            loop_count4 = 0;
        }
        
        //LED Blinking while program running
        if (loop_count_led >= loop_count_led_max){
            if (temp_led == 1) MCU_LED = 1;
            if (temp_led == -1) MCU_LED = 0;
            temp_led = -1 * temp_led;  

            loop_count_led = 0;
        }
        
        //print section
        if(loop_print >=4){    // 10 --> 40Hz, 5 --> 80Hz, 4--> 100Hz
            //time, R_deg, P_deg, Y_deg, R_cmd, P_cmd, Y_cmd
            // pc.printf("%d,%.2f,%.2f,%.2f\n\r",timer2.read_ms(),angle_f[0],angle_f[1],angleY);
            //pc.printf("%d,%.2f,%.2f,%.2f\n\r", timer2.read_ms(),angle_f[0],angle_f[1],angleY);
            //pc.printf("%d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d\n\r",timer2.read_ms(),angle_f[0],angle_f[1],angleY,gyro_raw[0],gyro_raw[1],gyro_raw[2],acc_raw[0],acc_raw[1],acc_raw[2]);
            //pc.printf("%d, %d, %f, %d, %f\n\r", timer2.read_ms(),pressure,press_out,temp,temp_out);

            // eeprom.save_to_eeprom(pc, timer2.read_ms(), angle_f[0], angle_f[1], angleY);

            eeprom.save_to_eeprom(pc, pressure, angle_f[0], angle_f[1], angleY);

            // pc.printf("time: %10d, roll: %5.2f, pitch: %5.2f, yaw: %5.2f\n\r", timer2.read_ms(),angle_f[0],angle_f[1],angleY);
            // pc.printf("%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n\r", timer2.read_ms(),angle_f[0],angle_f[1],angleY,ROLL,PITCH,YAW,controller_offset[0],controller_offset[1],controller_offset[2]);
            loop_print = 0; 
        }

        

        while (timer1.read_us() < 2500); // main loop iterates for each 2.5ms, 400Hz  
        timer1.reset();
    }
}
