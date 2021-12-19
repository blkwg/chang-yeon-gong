#include <Servo.h>     // 서보모터 라이브러리 사용

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9          // LED 핀 정의
#define PIN_SERVO 10       // servo 핀 정의
#define PIN_IR A0      //  analong 핀 정의 

// Framework setting
#define _DIST_TARGET 255  // 멈추려는 목표거리 정의
#define _DIST_MIN 100 // 측정 최소거리, 플로터 범위
#define _DIST_MAX 410    //  측정 최대거리,플로터 범위

// Distance sensor  
#define _DIST_ALPHA 0.4
#define _ITERM_MAX 60
#define EMA_ALPHA 0.1

// Servo range
#define _DUTY_MIN 1375    // 서보 duty 최소값
#define _DUTY_NEU 1465    // 서보 duty 중간값
#define _DUTY_MAX 1565    // 서보 duty 최댓값

// Servo speed control
#define _SERVO_ANGLE 30       // 서보 각도
#define _SERVO_SPEED 60     //  서보의 속도
  
// Event periods
#define _INTERVAL_DIST 25   //거리측정 주기
#define _INTERVAL_SERVO 20    // 서보 조정 주기  
#define _INTERVAL_SERIAL 100  //  시리얼 제어 주기

// PID parameters
#define _KP 0.5   //비례이득
#define _KD 60     // 미분이득
#define _KI 0

//#define a 70
//#define b 450

//////////////////////
// global variables //
//////////////////////

// Servo instance 
Servo myservo;    // 서보 변수명을 myservo로 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;    // 측정값, ema필터 적용값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; //거리, 서보, 시리얼 측정 여부

// Servo speed control
int duty_chg_per_interval; // 한 주기 당 제어할 최대 duty값
int duty_target, duty_curr; // 목표 pulse주기값, 현재 pulse주기값

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

int a = 70;
int b = 450;


float ir_distance(void){ // return value unit: mm

  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (b - a) * (val - a);
  return val;
} // [20213075]

float ema_dist = 0;
float filtered_dist;
float samples_num = 3;

float under_noise_filter(void) {
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) {largestReading = currReading;}
    delayMicroseconds(1000);
  }
  return largestReading;
}

float ir_distance_filtered(void) {
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading;}
  }
  ema_dist = EMA_ALPHA * lowestReading + (1 - EMA_ALPHA) * ema_dist;
  return ema_dist;
}


void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED,OUTPUT);
myservo.attach(PIN_SERVO);

// move servo to neutral position
duty_target = duty_curr = _DUTY_NEU; // 초기화
myservo.writeMicroseconds(duty_curr); // 서보 중립위치

// initialize serial port
Serial.begin(57600); // 

// initialize global variables 
dist_target = _DIST_TARGET;
dist_raw = dist_ema = ir_distance();
error_curr = error_prev = dist_target - dist_raw;

last_sampling_time_dist = 0; // last_sampling_time_dist 초기화
last_sampling_time_servo = 0; // 
last_sampling_time_serial = 0; // 

event_dist = event_servo = event_serial = false;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / ((float)_SERVO_ANGLE) * _INTERVAL_SERVO / 1000; //  한 주기 당 제어할 최대 duty값 초기화
}
  


void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  } 



////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false; 
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered(); 
      dist_ema = (1 - _DIST_ALPHA) * dist_ema + _DIST_ALPHA * dist_raw;  

  // PID control logic
    error_curr = dist_target - dist_ema; 
    pterm = _KP * error_curr; 
    dterm = _KD * (error_curr - error_prev);  
    iterm = _KI * error_curr;

    if(abs(iterm) > _ITERM_MAX) iterm = iterm/2;
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm < -_ITERM_MAX) iterm = -_ITERM_MAX;
    
    control = pterm + dterm + iterm; 

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target < _DUTY_MIN){
    duty_target = _DUTY_MIN;
  } 
  if(duty_target > _DUTY_MAX){
    duty_target = _DUTY_MAX;
  }

   error_prev = error_curr;  // error update
  }
  
  if(event_servo) {
event_servo = false; 

    // adjust duty_curr toward duty_target by duty_chg_per_interval

    // update servo position

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
  duty_curr += duty_chg_per_interval;
  
  if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else if (duty_target < duty_curr) {
  duty_curr -= duty_chg_per_interval;
  if(duty_curr < duty_target) duty_curr = duty_target;
  
  } 
  
  
}
  // update servo position
  myservo.writeMicroseconds(duty_curr); 
  

  if(event_serial) {
    event_serial = false; 
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",iterm:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_chg_per_interval:");
    Serial.print(duty_chg_per_interval);
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410"); 
// Min :측정 최소거리, Max : 최대거리, Low : 목표구역 최소거리, dist_target :기준이 되는 거리, High : 목표구역 최대거리 값을 시리얼 모니터에 표시

  }
}
