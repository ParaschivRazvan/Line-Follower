//FUNCTION DEFINITION FOR CONVENIENCE

#define dW digitalWrite
#define dR digitalRead

//PIN DEFINITION 

#define BUTTON1  7
#define BUTTON2  6
#define LEDG  13
#define MOT1  2
#define MOT2  3
#define MOT3  4
#define MOT4  5

// GLOBAL CONSTANTS 

#define WHITE  1
#define BLACK  0

#define COLOR   BLACK         //Line color

#define SENSOR_NO  6       //Number of sensors
#define SENSOR_RANGE 1000    //Sensor readings are mapped to this range
#define WEIGHT_UNIT 1000    //Unit for weighted average
#define AHEAD_DIR  1
#define BACKWARDS_DIR 0
#define PRESS  1
#define RELEASE  0
#define CAL_SPEED  400     //Sensor calibration speed
#define CAL_TIME  300     //Calibration time
#define P_LINE_MIN 0.5     //Minimum brightness percentage to consider part of the line

/* GLOBAL VARIABLES */

const float SPEED = 900;
const float KP = 0.4;
const float KD = 12;
const float KI = 0;
unsigned long ms = 0;
const int SENSOR[SENSOR_NO] = { A0, A1, A2, A3, A4, A5 };     //Arduino pins
int sens_max[SENSOR_NO];          //Maximum value each sensor measures (calibrated)
int sens_min[SENSOR_NO];          //Minimum value each sensor measures (calibrated)
int start = 0;
float line_pos = 0;
float last_line_pos = 0;

void setup() {
  
  Serial.begin(9600);
  
  initialize();
  
}

void initialize(){
  
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  pinMode(LEDG, OUTPUT);

  pinMode(MOT1, OUTPUT);
  pinMode(MOT2, OUTPUT);
  pinMode(MOT3, OUTPUT);
  pinMode(MOT4, OUTPUT);

  for (int i = 0; i <= SENSOR_NO; i++){
    
    pinMode(SENSOR[i], INPUT);
    sens_max[i] = 0;
    sens_min[i] = 1023;
  }
}

void loop() {

 
  if (dR(BUTTON1) == PRESS){    //Calibrate sensors
      
      dW(LEDG, HIGH);
      
      while (dR(BUTTON1) == PRESS); 
      
      delay(500); //Wait for hand to get out of the way
      
      calibrate(CAL_TIME, CAL_SPEED, AHEAD_DIR);
      calibrate(CAL_TIME, CAL_SPEED, BACKWARDS_DIR);
      calibrate(CAL_TIME, CAL_SPEED, AHEAD_DIR);
      calibrate(CAL_TIME, CAL_SPEED, BACKWARDS_DIR);
    
  }
  
  if (dR(BUTTON2) == PRESS){              //Start race
      start = 1;
      while(dR(BUTTON2) == PRESS){
          
          dW(LEDG, millis()%100 < 50);
          
          if(dR(BUTTON1) == PRESS){      //Cancel race
              start = 0;              
          }
      }
     
      dW(LEDG, LOW);
      
      if (start == 1){         //Race not cancelled
          
          delay(200);         //Wait for hand to get out of the way
          
          while( dR(BUTTON1) == RELEASE && dR(BUTTON2) == RELEASE ){      //Press any button to exit, otherwise race
              race();
              delay(15);
          }
       
          motor_speed(0, 0);
          
          delay(2000);
      }
  }
}

void calibrate(int cal_time, int cal_speed, int cal_dir){
  
  ms = millis();
  
  dW(LEDG, LOW);
  
  while ((ms + cal_time) > millis()){
    
    dW(LEDG, millis()%100 < 50);        //Blink led
    
    if (cal_dir == AHEAD_DIR)  motor_speed(cal_speed, cal_speed);
    
    if (cal_dir == BACKWARDS_DIR)  motor_speed(-cal_speed, -cal_speed);
    
  int sens_value[SENSOR_NO];
    
  for (int i = 0; i < SENSOR_NO; i++){
      
      sens_value[i] = analogRead(SENSOR[i]);
      sens_min[i] = (sens_value[i] < sens_min[i]) ? sens_value[i] : sens_min[i];
      sens_max[i] = (sens_value[i] > sens_max[i]) ? sens_value[i] : sens_max[i];
    }
  }
  
  motor_speed(0, 0);
  
  dW(LEDG, HIGH);
}


void race(void){
   
    last_line_pos = line_pos;
    line_pos = get_line_pos(COLOR,(last_line_pos>0));
        
    float PID_correction = get_PID_correction(line_pos, last_line_pos, KP, KD, KI);
    float max_correction = SPEED;                   //Can be changed to a lower value in order to limit the correction, needs to be at most SPEED
    
    if(PID_correction > 0){
        PID_correction = (PID_correction > max_correction) ? max_correction : PID_correction;
        motorSpeed(SPEED, SPEED - PID_correction);
    }else{
        PID_correction = (PID_correction < -max_correction) ? -max_correction : PID_correction;
        motorSpeed(SPEED + PID_correction, SPEED);
    }    
}

float get_line_pos(int color, int last_dir){
    
    float line = 0;
    int line_detected = 0;
    float sens_scaled[SENSOR_NO];
    float avg_num = 0;          //Average numerator
    float avg_den = 0;          //Average denominator
    
    for (int i = 0; i < SENSOR_NO; i++){
        //Scale from 0 to R_SENS
        sens_scaled[i] = analogRead(SENSOR[i]) - sens_min[i];
        sens_scaled[i] *= SENSOR_RANGE;
        sens_scaled[i] /= (sens_max[i] - sens_min[i]);
        
        if (color == WHITE){
            sens_scaled[i] = SENSOR_RANGE - sens_scaled[i];     //Reverse scale to go from R_SENS to 0
        }
        if (sens_scaled[i] >= (float) SENSOR_RANGE * ((float)P_LINE_MIN / 100.0)){   //At least one sensor has to detect a line
            line_detected = 1;
        }
      
        avg+=sens_scaled[i] * i * WEIGHT_UNIT;
        avg_den += sens_scaled[i];        
    }
    if (line_detected == 1){
        line = avg_num / avg_den;                           //Weighted average
        line = line - (WEIGHT_UNIT * (SENSOR_NO-1)/2);     //Change scale from 0 _ 4000 to -2000 _ 2000
        dW(LEDG, LOW);
        } 
         else {
          line = WEIGHT_UNIT * (SENSOR_NO - 1) * last_dir;       //Use last direction to calculate error 
          line = line - (WEIGHT_UNIT * (SENSOR_NO-1)/2 );     //Change scale
          dW(LEDG, HIGH);
    }

  return line;
}

float get_PID_correction(float line, float last_line, float kp, float kd, float ki){
  
  float proportional = line;
  float derivative = line - last_line;
  float integral = line + last_line;
  float correction = (kp * proportional + kd * derivative + ki * integral);

  return correction;
}

void motor_speed(int m1, int m2) {           //From -1000 to 1000

  int pwm1 = map(abs(m1), 0, 1000, 0, 255);
  int pwm2 = map(abs(m2), 0, 1000, 0, 255);
  
  pwm1 = (m1>0) ? 255-pwm1 : pwm1;
  pwm2 = (m2>=0) ? pwm2 : 255-pwm2; 
  
  analogWrite(MOT2, pwm1);
  analogWrite(MOT4, pwm2);
  
  digitalWrite(MOT1, (m1 > 0) ? HIGH : LOW);
  digitalWrite(MOT3, (m2 >= 0) ? LOW : HIGH);
}



