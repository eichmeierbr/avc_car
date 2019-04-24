#include <Servo.h>
const byte N = 2;
//Servo esc;
//Servo servo;
Servo servo[N];
//int escPos = 90;
//int servoPos = 90;
int pos[N];
static const byte ESC_PIN = 7;
static const byte SERVO_PIN = 8;
static const byte RPM_FEEDBACK_PIN = 0;  //interrpt 0, pin 2
static const byte SERVO_FEEDBACK_PIN = A0;

//const float MUL = 0.7058823529; //180/255
unsigned long lastTime_servoFeedback = 0;
static const byte MOTOR_ID = 0;    //ID for diferentiating data recieved and sent over serial connections
static const byte SERVO_ID = 1;

//added for motor data timeout safety feature
static const unsigned long MOTOR_DATA_TIMEOUT = 200;  //4 x 50 ms (50 ms time period expected)
static unsigned long lastTimeMotorData = 0;
static const byte NEUTRAL = 90;

unsigned long last_rpm_pulse_update_ms = 0; //used for detecting a stopped car, and rejecting old data when writing to the serial port
unsigned long last_rpm_pulse_time_us = 0;//keeps track of rpms by comparing to system timer
static const long REV_PERIOD_MAX_US = 100000;  //in us
unsigned long rev_period = REV_PERIOD_MAX_US;  //100 ms is considered too long to be in motion
boolean forward = true;
/*Scratch that, I want these parameters set in ROS:
static const float wheel_radius = 0.05 // meters
static const float revs_to_mps_MUL = //assuming 2.85 gear ratio for brushless motor differential: https://forums.traxxas.com/showthread.php?9080733-Diff-gear-ratios
*/
//boolean rpm_period_updated = false;  //rpms must be updated every 100 ms, otherwise the car has stopped, and velocity data should show 0 m/s

void rpm_feedback()
{
  //Serial.println("in rpm_feedback");
  last_rpm_pulse_update_ms = millis();  //notice the 'ms' here we want to use millisecond for checking whether or not data is vallid. millis() can count up to 50 days while micros() only counts up to 70 minutes, thus millis() is used here.
  unsigned long time_now = micros();    //use time now for accurate time calculations
  unsigned long rev_period_temp = time_now - last_rpm_pulse_time_us; //get spur-gear revoltion period
  if(rev_period_temp > 0) rev_period = rev_period_temp;  //revs are within 
  else rev_period = REV_PERIOD_MAX_US;
  
  last_rpm_pulse_time_us = time_now; //using 'time_now' ensures that the time taken to get to this point in code does not interfere with rev_period accuracy - - - micros();  //reset time
  if(pos[MOTOR_ID] < 90)  //determine the direction that the vehicle is traveling in
  {
    forward = false;
  }else forward = true;
  
  //rpm_period_updated = true;  not needed, only last_rpm_pulse_time_ms is needed for checking
}

void setup() {
  // put your setup code here, to run once:
  
  pinMode(RPM_FEEDBACK_PIN, INPUT_PULLUP);
  attachInterrupt(RPM_FEEDBACK_PIN, rpm_feedback,FALLING);  //arduino reference recomends using digitalPinToInterrupt(RPM_FEEDBACK_PIN) but the command is not recognized here
  
  analogReference(EXTERNAL);    //Using external reference for servo position
  for(int i = 0; i < N; i++)    //initialize
  {
    pos[i] = 90;
    servo[i].attach(ESC_PIN + i);
  }
  Serial.begin(115200);
  while(!Serial);  //wait for serial port to connect
  while(!Serial.available())
  {
    Serial.print("READY");  //let the computer know the serial port is open
    delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() >= 1)
  {
    if(Serial.read() == 'G')
    {
      unsigned long t = millis();
      while((Serial.available() < 3) && ((millis() - t) < 10));  //wait for the rest of the package, or timeout
      if(Serial.available() >= 3)
      {
        char buf[3];
        Serial.readBytes(buf, 3);
        if((buf[0] == 'O') && (buf[1] >= 0) && (buf[1] < 2))
        {
          pos[buf[1]] = byte(buf[2]);
          if(buf[1] == MOTOR_ID) lastTimeMotorData = millis();    //time stamp of last motor data retreival
          //Serial.print("buf[2]: ");
          //Serial.println(byte(buf[2]), DEC);
          //Serial.print("pos: ");
          //Serial.println(pos[buf[1]]);
        }
      }
    }
  }
// This code may be used to return odometry to the ROS platform. Commented out by Braden Eichmeier
// 4/23/19 to debug the motor controller
  
  if((millis() - lastTimeMotorData) > MOTOR_DATA_TIMEOUT) pos[MOTOR_ID] = NEUTRAL;  //stop the motor if data is not being received
  
  for(int i = 0; i < N; i++)
  {
    servo[i].write(pos[i]);
  }
  
//  if(((millis() - lastTime_servoFeedback) >= 50) && Serial) // 20Hz     20) //50Hz matches current ROS driver settings
//  {
//    lastTime_servoFeedback = millis();
//    int servo_feedback = analogRead(SERVO_FEEDBACK_PIN);
//    Serial.write('G');    //PID
//    Serial.write('O');
//    Serial.write(SERVO_ID);
//    //Serial.print(servo_feedback);
//    Serial.write(lowByte(servo_feedback));
//    Serial.write(highByte(servo_feedback));
//    
//    //Serial.println(servo_feedback);
//    
//    float rev_frequency;
//    if((last_rpm_pulse_update_ms + 100) < millis()) rev_frequency = 0;  //use millis() since it can count up to 50 days, and will not have a chance of a hiccup after 70 minutes of using micros()
//    //instead, correct period when slowing down, also stop when the maximum threshold is reached
//    //if((micros() - last_rpm_pulse_time_us) >= REV_PERIOD_MAX_US) rev_frequency = 0;  //car is stopped in this case. I decided not to try correcting the period as mentioned above
//    else rev_frequency = (float) 1/rev_period*1000000;
//    byte *rev_freq_bytes_to_transmit = (byte *) &rev_frequency;
//    if(forward == false) rev_frequency = -rev_frequency;  //a negative frequency is used for reverse
//    Serial.write('G');    //PID
//    Serial.write('O');
//    Serial.write(MOTOR_ID);  //used for addressing
//    Serial.write(rev_freq_bytes_to_transmit, 4);
//    
//  }
}
