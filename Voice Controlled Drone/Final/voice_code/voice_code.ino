//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR 
//OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE 
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.

#include <Wire.h>
#include <ESP32Servo.h> // Change to the standard Servo library for ESP32
#include <WebSocketsServer.h>

const char* ssid = "ESP32-Drone";
const char* password = "";
WebSocketsServer webSocket = WebSocketsServer(80);

//////////////////////////////////////////////////////////////////**Voice mode**//////////////////////////////////////////////////////////////////////

bool isStarted = false;
bool isDescending = false;
char* webSocketCommand = "switch off";

//throttle
int voice_throttle = 1200;
int hoverThrottle = 1500;
int throttleIncrement = 1;

//pitch
int voice_pitch = 1500;
int pitchIncrement = 10;

//roll
int voice_roll =1500;
int rollIncrement = 10;

//yaw
int voice_yaw=1500;
int yawIncrement=20;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6]; // Increase the array size to 6 for Channel 1 to Channel 6
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

// float voltage;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch;
// float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
// float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
// float Kalman1DOutput[]={0,0};

float PRateRoll = 1.2; //For outdoor flights, keep this gain to 0.75 and for indoor flights keep the gain to be 0.6
float IRateRoll = 2;
float DRateRoll = 0.135;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 7.5;
float IRateYaw = 5;
float DRateYaw = 0.015;

uint32_t LoopTimer;
float t=0.0125;      //time cycle

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void channelInterruptHandler()
{
  current_time = micros();
  // Channel 1
  if (digitalRead(channel_1_pin))
  {
    if (last_channel_1 == 0)
    {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (last_channel_1 == 1)
  {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;
  }

  // Channel 2
  if (digitalRead(channel_2_pin))
  {
    if (last_channel_2 == 0)
    {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (last_channel_2 == 1)
  {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }

  // Channel 3
  if (digitalRead(channel_3_pin))
  {
    if (last_channel_3 == 0)
    {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (last_channel_3 == 1)
  {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }

  // Channel 4
  if (digitalRead(channel_4_pin))
  {
    if (last_channel_4 == 0)
    {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (last_channel_4 == 1)
  {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }

  // Channel 5
  if (digitalRead(channel_5_pin))
  {
    if (last_channel_5 == 0)
    {
      last_channel_5 = 1;
      timer_5 = current_time;
    }
  }
  else if (last_channel_5 == 1)
  {
    last_channel_5 = 0;
    ReceiverValue[4] = current_time - timer_5;
  }

  // Channel 6
  if (digitalRead(channel_6_pin))
  {
    if (last_channel_6 == 0)
    {
      last_channel_6 = 1;
      timer_6 = current_time;
    }
  }
  else if (last_channel_6 == 1)
  {
    last_channel_6 = 0;
    ReceiverValue[5] = current_time - timer_6;
  }
}


void neutralPositionAdjustment()
{
  int min = 1490;
  int max = 1510;
  if (ReceiverValue[0] < max && ReceiverValue[0] > min)
  {
    ReceiverValue[0]= 1500;
  } 
  if (ReceiverValue[1] < max && ReceiverValue[1] > min)
  {
    ReceiverValue[1]= 1500;
  } 
  if (ReceiverValue[3] < max && ReceiverValue[3] > min)
  {
    ReceiverValue[3]= 1500;
  } 
  if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] )
  {
    ReceiverValue[0]= 1500;
    ReceiverValue[1]= 1500;
    ReceiverValue[3]= 1500;
  }



}
void gyro_signals(void)
{
Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AccX=AccX-0.02;
  AccY=AccY+0.01;
  AccZ=AccZ-0.04; // calibration offset
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void)
{
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}




void setup(void) {
  
Serial.begin(115200);
Serial.println();

 // Set up WiFi as an access point
  WiFi.softAP(ssid, password);

  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

int led_time=100;
 pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);


  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);
  pinMode(channel_6_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

 
  mot1.attach(mot1_pin,1000,2000);
  mot2.attach(mot2_pin,1000,2000);
  mot3.attach(mot3_pin,1000,2000);
  mot4.attach(mot4_pin,1000,2000);
//to stop esc from beeping
  mot1.write(0);
  mot2.write(0);
  mot3.write(0);
  mot4.write(0); 
  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(2000);
  digitalWrite(15, LOW);
  delay(2000);


  for (RateCalibrationNumber = 0; RateCalibrationNumber < 4000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 4000;
  RateCalibrationPitch /= 4000;
  RateCalibrationYaw /= 4000;
//Gyro Calibrated Values
  // Serial.print("Gyro Calib: ");
  // Serial.print(RateCalibrationRoll);
  // Serial.print("  ");
  // Serial.print(RateCalibrationPitch);
  // Serial.print("  ");
  // Serial.print(RateCalibrationYaw);
  // Serial.print(" -- ");


  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  delay(1000);
  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  delay(1000);

  pinMode(2,OUTPUT);
  // WebSocket event handler
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);


  

LoopTimer = micros();


}

void loop(void) {

  webSocket.loop();
  digitalWrite(2,HIGH);
  //enter your loop code here


  gyro_signals();
   RateRoll -= RateCalibrationRoll;
   RatePitch -= RateCalibrationPitch;
   RateYaw -= RateCalibrationYaw;


  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  

if (ReceiverValue[4] < 1500) {
    // Voice command logic
    if (webSocketCommand == "turn on") { // Start
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = 1500;
        ReceiverValue[2] = 1200; // Initial throttle for start
        ReceiverValue[3] = 1500;
        isStarted = true; // Set flag for started state
    } 
    else if (webSocketCommand == "switch off") { // Stop
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = 1500;
        ReceiverValue[2] = 1000; // Set throttle to minimum for stop
        ReceiverValue[3] = 1500;
        isStarted = false; // Clear started state
    } 
    else if (webSocketCommand == "high") { // Gradually increase throttle (Up)
        if (isStarted && voice_throttle < 1550) {  // Ensure drone is started and throttle is below max
            voice_throttle += throttleIncrement;   // Gradual increment
            ReceiverValue[2] = voice_throttle;
        }
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = 1500;
        ReceiverValue[2] = voice_throttle;
        ReceiverValue[3] = 1500;
    } 
    else if (webSocketCommand == "low") { // Gradually decrease throttle (Down)
        if (isStarted && voice_throttle > 1300) {  // Ensure throttle is above minimum
            voice_throttle -= throttleIncrement;   // Gradual decrement
            ReceiverValue[2] = voice_throttle;
        }
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = 1500;
        ReceiverValue[2] = voice_throttle;
        ReceiverValue[3] = 1500;
    }
    else if (webSocketCommand == "forward") { // Gradually pitch forward (move forward)
        if (isStarted && voice_pitch < 1670) {  // Assuming 1600 is max pitch forward
            voice_pitch += pitchIncrement;      // Gradual increment for smooth forward motion
            ReceiverValue[1] = voice_pitch;     // Control pitch channel
        }
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = voice_pitch;
        ReceiverValue[2] = voice_throttle;      // Keep throttle unchanged
        ReceiverValue[3] = 1500;
    }
    else if (webSocketCommand == "backward") { // Gradually pitch backward (move backward)
        if (isStarted && voice_pitch > 1330) {  // Assuming 1400 is minimum pitch backward
            voice_pitch -= pitchIncrement;      // Gradual decrement for smooth backward motion
            ReceiverValue[1] = voice_pitch;     // Control pitch channel
        }
        ReceiverValue[0] = 1500;
        ReceiverValue[1] = voice_pitch;
        ReceiverValue[2] = voice_throttle;      // Keep throttle unchanged
        ReceiverValue[3] = 1500;
    }
    else if (webSocketCommand == "hover") { // Hover mode
        if (isStarted) {
            ReceiverValue[2] = hoverThrottle; // Maintain hover throttle
            ReceiverValue[0] = 1500;
            ReceiverValue[1] = 1500;
            ReceiverValue[3] = 1500;
        }
    } 

    else if (webSocketCommand == "right") { // Gradually roll right
    if (isStarted && voice_roll < 1625) {  // Assuming 1625 is max roll right
        voice_roll += rollIncrement;       // Gradual increment for smooth right roll
        ReceiverValue[0] = voice_roll;     // Control roll channel
    }
    ReceiverValue[0] = voice_roll;         // Update roll value
    ReceiverValue[1] = 1500;               // Keep pitch unchanged
    ReceiverValue[2] = voice_throttle;     // Keep throttle unchanged
    ReceiverValue[3] = 1500;               // Keep yaw unchanged
    }

    else if (webSocketCommand == "left") { // Gradually roll left
    if (isStarted && voice_roll > 1375) {  // Assuming 1375 is min roll left
        voice_roll -= rollIncrement;       // Gradual decrement for smooth left roll
        ReceiverValue[0] = voice_roll;     // Control roll channel
    }
    ReceiverValue[0] = voice_roll;         // Update roll value
    ReceiverValue[1] = 1500;               // Keep pitch unchanged
    ReceiverValue[2] = voice_throttle;     // Keep throttle unchanged
    ReceiverValue[3] = 1500;               // Keep yaw unchanged
    }

    else if (webSocketCommand == "clock") { // Gradually yaw right (rotate clockwise)
    if (isStarted && voice_yaw > 1380) {  // Assuming 1625 is max yaw right
        voice_yaw -= yawIncrement;        // Gradual increment for smooth rotation
        ReceiverValue[3] = voice_yaw;     // Control yaw channel
    }
    ReceiverValue[0] = 1500;              // Keep roll unchanged
    ReceiverValue[1] = 1500;              // Keep pitch unchanged
    ReceiverValue[2] = voice_throttle;    // Keep throttle unchanged
    ReceiverValue[3] = voice_yaw;         // Update yaw value
    }

    else if (webSocketCommand == "counter") { // Gradually yaw left (rotate counterclockwise)
    if (isStarted && voice_yaw < 1620) {  // Assuming 1375 is min yaw left
        voice_yaw+= yawIncrement;        // Gradual decrement for smooth rotation
        ReceiverValue[3] = voice_yaw;     // Control yaw channel
    }
    ReceiverValue[0] = 1500;              // Keep roll unchanged
    ReceiverValue[1] = 1500;              // Keep pitch unchanged
    ReceiverValue[2] = voice_throttle;    // Keep throttle unchanged
    ReceiverValue[3] = voice_yaw;         // Update yaw value
    }



    else {
        // Handle other voice commands if necessary
    }

} else {
    // Transmitter mode logic (voice mode is off)
    Serial.println(" Transmitter Mode ");
    channelInterruptHandler();
    neutralPositionAdjustment(); // Adjust positions based on transmitter input
}




  DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
  DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
  InputThrottle=ReceiverValue[2];
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);

  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);     
  DesiredRateRoll=PIDReturn[0]; 
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];

  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise


  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


  int ThrottleIdle = 1150;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

  int ThrottleCutOff = 1000;
  if (ReceiverValue[2] < 1050)
  {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  mot1.write(map(MotorInput1, 1000, 2000, 0, 180));
  mot2.write(map(MotorInput2, 1000, 2000, 0, 180));
  mot3.write(map(MotorInput3, 1000, 2000, 0, 180));
  mot4.write(map(MotorInput4, 1000, 2000, 0, 180));

// voltage= (analogRead(36)/4096)*12.46*(35.9/36);
// if(voltage<11.1)
// {

// }

//Reciever signals
  
  Serial.print(ReceiverValue[0]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[1]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[2]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[3]);
  Serial.print(" - ");
  Serial.println(ReceiverValue[4]);
  // Serial.println(" - ");

//   // Serial.print(ReceiverValue[5]);
//   // Serial.print(" - ");

//Motor PWMs in us
  // Serial.print("MotVals-");
  // Serial.print(MotorInput1);
  // Serial.print("  ");
  // Serial.print(MotorInput2);
  // Serial.print("  ");
  // Serial.print(MotorInput3);
  // Serial.print("  ");
  // Serial.print(MotorInput4);
  // Serial.print(" -- ");

// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");

  // Serial.print("AccX[g]:");
  // Serial.print(AccX);
  // Serial.print(" ");
  // Serial.print("AccY[g]:");
  // Serial.print(AccY);
  // Serial.print(" ");
  // Serial.print("AccZ[g]:");
  // Serial.println(AccZ);
  

// //Gyro Rates
  // Serial.print(" Gyro rates:");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print(" -- ");



//PID outputs
// Serial.print("PID O/P ");
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");

//Angles from MPU
  // Serial.print("KalmanAngleRoll:");
  // Serial.println(KalmanAngleRoll);
  // Serial.print(",");
  // Serial.print("KalmanAnglePitch:");
  // Serial.println(KalmanAnglePitch);

  
  //  Serial.println(webSocketCommand);


 
  while (micros() - LoopTimer < (t*1000000));
  {
     LoopTimer = micros();

  }

}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  String receivedText;  // Declare the variable outside the switch statement

  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("Client %u disconnected\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("Client %u connected\n", num);
      break;
    case WStype_TEXT:
      receivedText = String((char*)payload);

      // Check for specific commands and process them
      if (receivedText == "turn on") {
          Serial.println("Received 'start' command");
          webSocketCommand = "turn on";  // Update the global command variable
        } 
        
      else if (receivedText == "switch off") {
        Serial.println("Received 'stop' command");
        webSocketCommand = "switch off";  // Update the global command variable
      }

      else if (receivedText == "high") {
        Serial.println("Received 'up' command");
        webSocketCommand = "high";  // Update the global command variable
      }

      else if (receivedText == "low") {
        Serial.println("Received 'down' command");
        webSocketCommand = "low";  // Update the global command variable
      }

      else if (receivedText == "forward") {
        Serial.println("Received 'forward' command");
        webSocketCommand = "forward";  // Update the global command variable
      }

      else if (receivedText == "backward") {
        Serial.println("Received 'backward' command");
        webSocketCommand = "backward";  // Update the global command variable
      }

      else if (receivedText == "hover") {
        Serial.println("Received 'hover' command");
        webSocketCommand = "hover";  // Update the global command variable
      }

      else if (receivedText == "right") {
        Serial.println("Received 'right' command");
        webSocketCommand = "right";  // Update the global command variable
      }

      else if (receivedText == "left") {
        Serial.println("Received 'left' command");
        webSocketCommand = "left";  // Update the global command variable
      }

      else if (receivedText == "clock") {
        Serial.println("Received 'clockwise' command");
        webSocketCommand = "clock";  // Update the global command variable
      }

      else if (receivedText == "counter") {
        Serial.println("Received 'counterclockwise' command");
        webSocketCommand = "counter";  // Update the global command variable
      }

      else {
        webSocketCommand = webSocketCommand;
        Serial.println(webSocketCommand); 
      }
      // Do not send any responses back to the JavaScript client
      break;
      case WStype_BIN:
      Serial.printf("Received Binary Message of %u bytes\n", length);
      // Optionally handle binary messages here
      break;
      case WStype_PONG:
      Serial.printf("Pong received\n");
      break;
      case WStype_ERROR:
      Serial.printf("Error occurred\n");
      break;
  }
}
