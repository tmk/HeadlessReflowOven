/*********************************************************************
  Headless ReflowController - just with button, buzzer and LED

  based on ReflowOven
    https://github.com/ohararp/ReflowOven

  - To start/abort reflow process push button.
  - Buzzer beeps when cooling stage starts and ends. You may open door oven to cool quickly.
  - LED and buzzer are optional but buzzer is recommended.
  - Using with KOIZUMI Wide Oven Toaster 1200W KOS-1202
**********************************************************************/
// Includes
#include <Adafruit_MAX31855.h>  // Thermocouple Library - https://github.com/rocketscream/MAX31855
#include <PID_v1.h>             // PID Library - http://playground.arduino.cc/Code/PIDLibrary
                                //             - https://github.com/br3ttb/Arduino-PID-Library/

typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR,
  REFLOW_STATE_ABORT
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;



/*  Reflow temperature profile

    Tamura leaded SOLDER PASTE(Sn62.8 Pb36.8 Ag0.4) Reflow Profile
        |
    220-|                                                  x  x
        |                                                x       x
    200-|                                               x           x
        |                                              x|           |
        |                                            x  |           | x
    170-|                              x x x  x x  x    |           |   x
        |                  x  x  x  x           |       |<- 20-60s->|     x
    150-|               x                       |                   |
        |             x |                       |                   |
        |           x   |                       |                   |
        |         x     |                       |                   |
        |       x       |                       |                   |
        |     x         |                       |                   |
        |   x   <3C/s   |        20C/60s        |         20/20s    |
    30 -| x             |                       |                   |
        |     <60s      |<-        60s        ->|                   |
        |     Preheat   |        Soaking        |       Reflow      |   Cool
     0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _

    Almit leadfree TM-HP LFM-48W(Sn-3.0Ag-05Cu) Reflow Profile
    http://www.somersetsolders.com/userfiles/file/solder_paste/almit_lead_free_reflow.pdf
        |
    235-|                                                  x  x
        |                                                x       x
    220-|                                               x           x
        |                                              x|           |
        |                                            x  |           | x
    190-|                              x x x  x x  x    |           |   x
        |                  x  x  x  x           |       |<- 20-50s->|     x
    160-|               x                       |                   |
        |             x |                       |                   |
        |           x   |                       |                   |
        |         x     |                       |                   |
        |       x       |                       |                   |
        |     x         |                       |                   |
        |   x   1.5C/s  |        0.5C/s         |       1.5C/s      |   1.5C/s
    30 -| x             |                       |                   |
        |      90-130s  |<-     30-60s        ->|                   |
        |     Preheat   |        Soaking        |       Reflow      |   Cool
     0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _
*/
float TEMPERATURE_COOL       = 100;
float TEMPERATURE_SOAK_MIN   = 160;
float TEMPERATURE_SOAK_MAX   = 190;
float TEMPERATURE_REFLOW_MAX = 245;

// Preheat: 1.5C/s
#define PREHEAT_TEMPERATURE_STEP    15
#define PREHEAT_MS_STEP             10000

// Soak: 30C/60s
#define SOAK_TEMPERATURE_STEP       3
#define SOAK_MS_STEP                6000

// Reflow: 30C/20s
#define REFLOW_TEMPERATURE_STEP     6
#define REFLOW_MS_STEP              4000


// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
float PID_KP_PREHEAT         = 300;
float PID_KI_PREHEAT         = 0.05;
float PID_KD_PREHEAT         = 350;
// ***** SOAKING STAGE *****
float PID_KP_SOAK            = 300;
float PID_KI_SOAK            = 0.05;
float PID_KD_SOAK            = 350;
// ***** REFLOW STAGE *****
float PID_KP_REFLOW          = 300;
float PID_KI_REFLOW          = 0.05;
float PID_KD_REFLOW          = 350;

#define PID_SAMPLE_TIME 1000
#define SENSOR_SAMPLING_TIME   1000


// Reflow status strings
const char* ssdMessagesReflowStatus[] = {
  "  Ready  ",
  " Preheat ",
  "   Soak  ",
  "  Reflow ",
  "   Cool  ",
  " Complete",
  "  !HOT!  ",
  "  Error  "
};


//
// Pin Definitions
//
int HeaterPin = 5;  // Heater Element(active-high)
int BuzPin = 6;     // Buzzer Pin
int Led = 7;        // Heartbeat Led
int But1 = 4;       // Start Button(pulluped, active-low)
int thermocoupleCLKPin = A1;
int thermocoupleSOPin  = A3;
int thermocoupleCSPin  = A2;


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double inputOld; //Store old Temperature
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
boolean StartTest;

// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;

//Button Variables
byte ButtCount  = 0;

// tc Error Counter
int tcErrorCtr = 0;

// Seconds timer
int timerSeconds;

// Library Setup
// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
Adafruit_MAX31855 thermocouple(thermocoupleCLKPin, thermocoupleCSPin, thermocoupleSOPin);

// Setup
void setup()   {
  // Setup Serial Baudrate
  Serial.begin(9600);

  // initialize the Led pin as an output.
  pinMode(Led, OUTPUT);

 // Init Buttons
  pinMode(But1, INPUT_PULLUP);

  // Init Elements
  pinMode(HeaterPin, OUTPUT);
  pinMode(BuzPin, OUTPUT);

  digitalWrite(HeaterPin, LOW);
  digitalWrite(BuzPin, LOW);

  // Set window size
  windowSize = 5000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Set Relow Status = OFF
  reflowStatus = REFLOW_STATUS_OFF;
  StartTest = false;

  Serial.print("For lead free\r\n");
}

// Begin Main Loop
void loop()
{
   // Current time
  unsigned long now;

  // Test for Start Button Input
   if (digitalRead(But1) == LOW)
  {

    for (int i = 0; i<5;i++){
        if (digitalRead(But1) == LOW)
        {
          ButtCount++;
          delay(10);
        }
      }

      if (ButtCount>=5)
      {
        tone(BuzPin,4100,500); //Buzz the Buzzer
        while (digitalRead(But1) != HIGH) ;
        reflowState = REFLOW_STATE_IDLE;
        if (reflowStatus == REFLOW_STATUS_ON) {
          reflowState = REFLOW_STATE_ABORT;
          StartTest = false;
        } else {
          StartTest = true;
        }
      }
      else
      {
        ButtCount = 0;
        StartTest = false;
      }
  }
  else
  {
    ButtCount = 0;
    StartTest = false;
  }

//********************************************************************************************************
  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    inputOld = input; //Store Old Temperature
    input = thermocouple.readCelsius();
    if(isnan(input))
    {
      input=inputOld;
      tcErrorCtr++;

      if (tcErrorCtr >= 5)
      {
        // Illegal operation
        reflowState  = REFLOW_STATE_ERROR;
        reflowStatus = REFLOW_STATUS_OFF;
      }
    }
    else
    {
      tcErrorCtr = 0;
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle LED as system heart beat
      digitalWrite(Led, !(digitalRead(Led)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(",");
      //Serial.print(thermocouple.readInternal());
      //Serial.print(",");
      Serial.print(setpoint);
      Serial.print(",");
      Serial.print(input);
      Serial.print(",");
      Serial.print(output);
      Serial.print(",");
      Serial.println(ssdMessagesReflowStatus[reflowState]);
    }
    else
    {
      Serial.print(input);
      Serial.print(" ");
      Serial.println(thermocouple.readInternal());
      // Turn off LED
      digitalWrite(Led, HIGH);
    }

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // No thermocouple wire connected
      Serial.print("TC Error!\n");
    }
  }


  // Reflow oven controller state machine
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
    // If oven temperature is still above room temperature
    if (input >= TEMPERATURE_COOL)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
        Serial.print(" - HOT - ");
        Serial.println(input);
      }
      else
      {
        if (StartTest == true)
        {
          Serial.println(" - Begin  - ");
          //Serial.println("Time Setpoint Input Output");
          timerSeconds = 0;

          // Initialize PID control window starting time
          windowStartTime = millis();

          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          reflowOvenPID.SetMode(AUTOMATIC);

          // Preheat profile
          setpoint = input + PREHEAT_TEMPERATURE_STEP;
          timerSoak = millis() + PREHEAT_MS_STEP;

          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
          StartTest = false;
          }
      }
  break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    if (millis() > timerSoak)
    {
        setpoint = input + PREHEAT_TEMPERATURE_STEP;
        if (setpoint > TEMPERATURE_SOAK_MIN) setpoint = TEMPERATURE_SOAK_MIN;
        timerSoak = millis() + PREHEAT_MS_STEP;

        // If minimum soak temperature is achieve
        if (input >= TEMPERATURE_SOAK_MIN-2.5)
        {
          reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
          setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
          timerSoak = millis() + SOAK_MS_STEP;

          // Proceed to soaking state
          reflowState = REFLOW_STATE_SOAK;
        }
    }
  break;

  case REFLOW_STATE_SOAK:
    // If micro soak temperature is achieved
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MS_STEP;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW;
      }
    }
  break;

  case REFLOW_STATE_REFLOW:
    // We need to avoid hovering at peak temperature for too long
    // Crude method that works like a charm and safe for the components
    if (input >= (TEMPERATURE_REFLOW_MAX-5.0))
    {
      // Set PID parameters for cooling ramp
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL;
      // Proceed to cooling state
      reflowState = REFLOW_STATE_COOL;

      // Turn Elements Off
      digitalWrite(HeaterPin, LOW);

      // Signal to Open Door
      for (int i = 0; i<10;i++){
        tone(BuzPin,4100,100); //Buzz the Buzzer
        delay(250);
      }

    }
  break;

  case REFLOW_STATE_COOL:
    // If minimum cool temperature is achieve
    if (input <= TEMPERATURE_COOL)
    {
      // Retrieve current time for buzzer usage
      buzzerPeriod = millis() + 1000;

      for (int i = 0; i<10;i++){
        tone(BuzPin,4100,500); //Buzz the Buzzer
        delay(250);
      }

      reflowStatus = REFLOW_STATUS_OFF;
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_COMPLETE;
    }
  break;

  case REFLOW_STATE_COMPLETE:
    if (millis() > buzzerPeriod)
    {
      reflowState = REFLOW_STATE_IDLE;
    }
  break;

  case REFLOW_STATE_TOO_HOT:
  // If oven temperature drops below room temperature
    if (input < TEMPERATURE_COOL)
    {
    // Ready to reflow
    reflowState = REFLOW_STATE_IDLE;
    }
  break;

  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
    if(isnan(input))
    {
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR;
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

    case REFLOW_STATE_ABORT:
      Serial.println("Abort!");
      StartTest = false;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
    break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime))
    {
      digitalWrite(HeaterPin, HIGH);
    }
    else
    {
      digitalWrite(HeaterPin, LOW);
    }
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(HeaterPin, LOW);
  }

}
