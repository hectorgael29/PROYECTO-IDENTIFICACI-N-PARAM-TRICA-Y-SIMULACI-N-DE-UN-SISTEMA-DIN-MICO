//****************TIME VARIABLES**************************************
unsigned long t_now;
unsigned long t_prev = 0;

//****************INTERRUPTS PINS*************************************

// If you use interrupts, you have to use pins 2 and 3 in UNO, NANO, MINI..
// Using the interrupts, we will calculate the rotation direction- 
//if needed, count the encoder pulses and caluclate the angular speed (rpm)
const byte interruptPinA = 2; //Encoder A
const byte interruptPinB = 3; //Encoder B

//****************COUNTER VARIABLE************************************

//This is the variable allocated for counting the pulses from the encoder
//every time the motor rotates
volatile long EncoderCount = 0;


//*************UNO OUTPUTS --> L298 BRIDGE INPUTS*********************
const byte DirPin1 = 4;
const byte DirPin2 = 5;//DirPin 1 and 2 will decide the motor rotation direction
const byte PWMPin = 6; // This pin will control the Motor speed via PWMval, Habilita motor con PWM
int PWMval=0; // Initial PWMval is set to zero, this value is the PWM duty cycle
              // from 0 to 255

//****************ENCODER PULSES************************************
int PPR=834.0; //Pulse Per Rotation-Measured manually-see youtube video

volatile unsigned long count = 0;
unsigned long count_prev = 0;

//***************************RPM***********************************
// Theta is the angular position
//Theta_now and Theta_prev are both needed to calculate the RPM (rounds per minute)
//RPM can be calcuated as the difference in position with respect to time
float Theta_now; 
float Theta_prev = 0;

//RPM_input is the user input RPM (set value)
//RPM_output is the motor output RPM as measured by the encoder
float RPM_output;
int dt;                      // Period of time used to calcuate RPM
float RPM_max = 200;         // Setting up a safe maximum RPM for the Motor

//******************************MISC VARIABLES*****************
//Maximum motor voltage in clockwise rotation
float Vmax = 12;      // Check Motor Datasheet
//Minimum motor voltage
float Vmin = 5;
float V = 12;         // Motor voltage input. Set voltage to step amplitud 

//**********ISR FUNCTIONS*****************************************
//***********ENCODER A********************************************
//This is the pinA function,Interrupt Service Routine for pin A
// below is the logic for pinA
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);
// detecting if motor rotating clockwise
// if A is high while B is low, then direction of rotation is clockwise
  if (PinB == LOW) {
    if (PinA == HIGH) {
     EncoderCount++;
    }
   }

  else {
     if (PinA == LOW){
      EncoderCount++;
    }
  }
}

//**********ISR FUNCTIONS*************************************
//***********ENCODER B****************************************

void ISR_EncoderB() {
   EncoderCount++;
}
//*************************************************************
//***MOTOR DRIVER FUNCTION*************************************
//The below code takes the calcuated voltage V and maps it to an output PWM range from 0 to 255 (8 bit PWM resolution in UNO)
void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }

  //setting motor direction
  if(V>=0){
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  }
  else{
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  } 
  analogWrite(PWMPin, PWMval);
}
//*************INTERRUPT SERVICE ROUTINE*************************
/*ISR(TIMER1_COMPA_vect) {
  count++;
}*/
void setup() {
  //General setup

  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, RISING);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

//***********INTERRUPT SETUP************************************
  /*cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();*/
}
//**************************************************************
//**************************************************************
//****************MAIN LOOP*************************************
unsigned long previousMillis = 0;
long interval = 100;
void loop() {
 /* if (count > count_prev) {
    t_now = millis();
    Theta_now = EncoderCount / PPR;
    dt = (t_now - t_prev);*/
    
    unsigned long currentMillis = millis();
    if((currentMillis - previousMillis) >= interval)
    {
      previousMillis = currentMillis;
      RPM_output = 10*EncoderCount*(60.0/374.0); // RPM del eje principal
      EncoderCount = 0;
    }
    
    //*********CONDITION OF OPERATION**************************
    //*********SWITCH MOTOR OFF AFTER A PERIOD OF NO INPUT*****
    /*if (t_now / 1000.0 > 100) {
      V= 0;
    }*/

    //***********RPM CALCULATIONS*******************
    /*RPM_output = (Theta_now - Theta_prev) / (dt / 1000.0) * 60;

    if (V > Vmax) {
      V = Vmax;
    }
    if (V < Vmin) {
      V = Vmin;
    }*/
//*********************************************************
//*********************************************************
    //*******WRITING VOLTAGE TO L298***********************
     WriteDriverVoltage(V, Vmax);
     Serial.print(V);
     Serial.print(", ");
     //Serial.print("RPM =");
     //Serial.print(", ");
     Serial.println(RPM_output);
     delay(100);
     /*
     Theta_prev = Theta_now;
     count_prev = count;
     t_prev = t_now;*/
  }