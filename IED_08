#define PIN_ECHO 13
#define PIN_TRIG 12
#define PIN_LED 9

#define SND_VEL 346.0
#define INTERVAL 100
#define _DIST_MIN 100
#define _DIST_MAX 300

float timeout;
//시간제한
float dist_min, dist_max, dist_raw;
//vvv added in 2nd practice.
unsigned long last_sampling_time;
float scale;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0;

  dist_raw = 0.0;
  scale = 0.001 * 0.5 * SND_VEL;

  Serial.begin(57600);
//vvv added in 2nd practice.
  last_sampling_time = 0;
}

void loop() {
//vvv added in 2nd practice.
  if (millis() < last_sampling_time + INTERVAL) return;

  if (USS_measure(PIN_TRIG, PIN_ECHO) != 0) { 
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  }

  Serial.print("MIN: 0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.println("MAX:400");

  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else if(dist_raw > dist_min && dist_raw < 200){
    analogWrite(PIN_LED, 255*(200-dist_raw)/100);
  }
  else {
    analogWrite(PIN_LED, 255/100*(dist_raw-200));
  }
      delay(25);
      //delay(INTERVAL); deleted in 2nd practice.
      last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale;
  if (reading < dist_min || reading > dist_max) reading = 0;
  
  
 
  return reading;
}
