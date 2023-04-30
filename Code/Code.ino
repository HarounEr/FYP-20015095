
/* Motor Control Subsystem *********************************************************/

// Motor Control Commands

// the following commands have been defined 
// - command stop 
// - command forward, distance
// - command backward, distance
// - command turn to face, degrees
// - command wait, time

#define CMD_STOP             0
#define CMD_FORWARD          1
#define CMD_BACKWARD         2
#define CMD_TURN_TO_FACE     3
#define CMD_WAIT             4
#define CMD_DISTANCE         5
#define SLAM_READING         6
#define SLAM_OVER            7
#define CALIBRATION_START    8
#define CALIBRATION_COMPLETE 9

// Command queue values

#define FALSE 0
#define TRUE -1
#define SIZE_OF_COMMAND_Q   64

// Supervisor activity

#define SUPERVISOR_NULL        0
#define SUPERVISOR_IDLE        1
#define SUPERVISOR_CALIBRATE   2
#define SUPERVISOR_SLAM        3
#define SUPERVISOR_CLEANING    4
#define SUPERVISOR_FINISHED    5


// Compass reading

#include <Wire.h>
#include <Adafruit_Sensor.h> // using manufacturer code to display
#include <Adafruit_HMC5883_U.h> // importing the manufacturer library for HMC5883
// assigning a unique ID to the compass 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//int compassCorrection = 0; // set to 0 (not used)

// Motor control states

#define STATE_IDLE        0
#define STATE_FORWARD     1
#define STATE_BACKWARD    2
#define STATE_TURN        3
#define STATE_WAIT        4
#define STATE_READING     5
#define STATE_CALIBRATION 6

// command queue 

struct Command
{
  int command;
  int param1;
  int param2;
  int param3;
};


struct CommandQueue
{
  int size;
  int head;
  int tail;
  struct Command queue[SIZE_OF_COMMAND_Q];
};

// motor direction values

#define MOTOR_STOP      0
#define MOTOR_FORWARD   1
#define MOTOR_BACKWARD  2

// Motors pinouts 

const int leftForward = 5;
const int leftBackward = 4;
const int rightForward = 3;
const int rightBackward = 2;

// ultrasound pinouts

#define trigPin 8 // pin 8
#define echoPin 7 // pin 7

// compass reading

int compass_direction;
float magnetic_x;
float magnetic_y;
float average_magnetic_x;
float average_magnetic_y;
float calibrate_x;
float calibrate_y;

// ultrasound reading

int ultrasound_reading;
long duration;
long distance;


// SLAM array

#define SLAM_MAX 80

#define SLAM_UNKNOWN 0
#define SLAM_EMPTY  1
#define SLAM_SOLID  2

int slam_array[SLAM_MAX][SLAM_MAX];
int current_x;
int current_y;


// response queue

CommandQueue response_queue;


// motor control queue 

CommandQueue motor_control_queue;

// mouvement control state

int mouvement_control_state;


// motor direction variables

int left_motor_direction;
int right_motor_direction;

// start of motor control queue functions 

void init_q(struct CommandQueue *q)
{
  q->head = 0;
  q->tail = 0;
  q->size = 0;
}

int read_q(struct CommandQueue *q, struct Command *e)
{
  int rc;
  int h;
  int s;

  rc = FALSE;

  s = q->size;

  if (s > 0)
  {
    h = q->head;
    e->command  = q->queue[h].command;
    e->param1 = q->queue[h].param1;
    e->param2 = q->queue[h].param2;
    e->param3 = q->queue[h].param3;
    h = (h + 1) % SIZE_OF_COMMAND_Q;
    s--;
    q->head = h;
    q->size = s;
    rc = TRUE;
  }

  return rc;
}

int write_q(struct CommandQueue *q, struct Command e)
{
  int rc;
  int t;
  int s;

  rc = FALSE;

  s = q->size;

  if (s < SIZE_OF_COMMAND_Q)
  {
    t = q->tail;
    q->queue[t].command  = e.command;
    q->queue[t].param1 = e.param1;
    q->queue[t].param2 = e.param2;
    q->queue[t].param3 = e.param3;
    t = (t + 1) % SIZE_OF_COMMAND_Q;
    s++;
    q->tail = t;
    q->size = s;
    rc = TRUE;
  }

  return rc;
}

// end of motor control queue functions 

// motor control functions

// left motor function
void left_motor (int direction)
{
  switch (direction)
  {
    case MOTOR_FORWARD:
      digitalWrite(leftForward, HIGH);
      digitalWrite(leftBackward, LOW);
      break;

    case MOTOR_BACKWARD:
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, HIGH);
      break;

    default:
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      break;
  }
}


// right motor function
void right_motor (int direction)
{
  switch (direction)
  {
    case MOTOR_FORWARD:
      digitalWrite(rightForward, HIGH);
      digitalWrite(rightBackward, LOW);
      break;

    case MOTOR_BACKWARD:
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, HIGH);
      break;

    default:
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
      break;
  }
}

// compass reading

float read_compass()
{
  // compass code goes here

  sensors_event_t event; 
  mag.getEvent(&event);
  magnetic_x = event.magnetic.x;
  magnetic_y = event.magnetic.y;
  
  float heading = atan2(magnetic_y - calibrate_y, magnetic_x - calibrate_x);
// search declination
  float declinationAngle = 0;
  heading += declinationAngle;
  
// Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}

// Ultrasound sensor

int read_ultrasound()
{
  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance:
  distance = duration * 0.034 / 2;
  Serial.print("Distance = ");
  Serial.println(distance);
  return distance;  
}

//////////////////////////////////////////////////////// SUPERVISOR ////////////////////////////////////////////
int supervisor_state;
Command cmd;
Command cmd2;
Command response;

/*** IDLE_STATE ***/  
void enter_IDLE ()
{
  Serial.print("enter idle??\n");
  enter_state(SUPERVISOR_CALIBRATE);
  


}

void body_IDLE ()
{
  Serial.print("body idle WHY\n");
  
}

void exit_IDLE ()
{ 
  Serial.print("running exit idle\n");

}


/*** CALIBRATE_STATE ***/  
void enter_CALIBRATE ()
{
  Serial.print("enter calibrate\n");
  cmd.command = CALIBRATION_START;
  write_q(&motor_control_queue, cmd);

  
}

void body_CALIBRATE ()
{
  Serial.print("BODY CALIBRATE\n");
  switch (response.command)
  {
    
    case CALIBRATION_COMPLETE:
      //GO TO SLAM
      Serial.print("Entered calibration complete");
      enter_state(SUPERVISOR_SLAM);
      break;

    default:
      break;
  
  }
}

void exit_CALIBRATE ()
{
  
  Serial.print("running exit calibrate\n ");
}


/*** SLAM_STATE ***/  
void enter_SLAM ()
{

  // entered SLAM
  int i;
  int j;

  for (i=0; i<SLAM_MAX; i++)
  {
    for (j=0; j<SLAM_MAX; j++)
    {
      slam_array[i][j] = SLAM_UNKNOWN;
    }
  }  
  current_x = SLAM_MAX / 2;
  current_y = SLAM_MAX / 2;

  for (i=0; i<8; i++)
  {
    // facing the direction 
    cmd.command = CMD_TURN_TO_FACE;
    cmd.param1 = i*45;
    write_q(&motor_control_queue, cmd);

    // take distance reading
    
    cmd.command = CMD_DISTANCE;
    write_q(&motor_control_queue, cmd);
    //delay(500);
  }
  // SLAM OVER
  cmd.command = SLAM_OVER;
  write_q(&motor_control_queue, cmd);
}

void body_SLAM ()
{
  switch(response.command)
  {
    case SLAM_READING:
    // trigonometry calculation
      //enter_state(SUPERVISOR_SLAM);
      break;

    case SLAM_OVER:
    
      enter_state(SUPERVISOR_CLEANING);
      break;

    default:
      break;

  }
}

void exit_SLAM ()
{
  Serial.print("running exit code slam");
  
}


/*** CLEANING_STATE ***/
void enter_CLEANING ()
{
  Serial.print("Entered cleaning\n ");
}

void body_CLEANING ()
{

}

void exit_CLEANING ()
{
  
}
// cleaning complete

/*** FINISH_STATE ***/  
void enter_FINISH ()
{

}

void body_FINISH ()
{
// return to starting area (to charge)
}

void exit_FINISH ()
{
  
}

/*** SUPERVISOR ***/


// entry code and exit code
void enter_state(int new_state)
{
  switch (supervisor_state)
  {

  case SUPERVISOR_IDLE:
    exit_IDLE();
    break;
  case SUPERVISOR_CALIBRATE:
    exit_CALIBRATE();
    break;
  case SUPERVISOR_SLAM:
    exit_SLAM();
    break;
  case SUPERVISOR_CLEANING:
    exit_CLEANING();
    break;
  case SUPERVISOR_FINISHED:
    exit_FINISH();
    break;

  default:
    break;
  }

  supervisor_state = new_state;

  switch (supervisor_state)
  {

  case SUPERVISOR_IDLE:
    enter_IDLE();
    break;
  case SUPERVISOR_CALIBRATE:
    enter_CALIBRATE();
    break;
  case SUPERVISOR_SLAM:
    enter_SLAM();
    break;
  case SUPERVISOR_CLEANING:
    enter_CLEANING();
    break;
  case SUPERVISOR_FINISHED:
    enter_FINISH();
    break;

  default:
    break;
  }
}

// Supervisor function
void supervisor()
{

  if (read_q(&response_queue, &response)== TRUE )
    {
      switch (supervisor_state)
      {

        case SUPERVISOR_IDLE:
          body_IDLE();
          break;

        case SUPERVISOR_CALIBRATE:
          body_CALIBRATE();
          break;


        case SUPERVISOR_SLAM:
          body_SLAM();
          break;

        case SUPERVISOR_CLEANING:
          body_CLEANING();
          break;

        case SUPERVISOR_FINISHED:
          body_FINISH();
          break;

        default:
          break;

      }
    }
}

/////////////////////////////////////////////////////// Movemennt Control //////////////////////////////////////
#define NUMBER_OF_CALIBRATION_READINGS 40 
int calibration_steps;


void mouvement_control()
{
  int i;
  
  switch (mouvement_control_state)
  {
    case STATE_IDLE:
    
      if (read_q(&motor_control_queue, &cmd) == TRUE )
      {
          switch (cmd.command)
          {
            case CMD_STOP:
              left_motor_direction = MOTOR_STOP;
              right_motor_direction = MOTOR_STOP;

              break;

            case CMD_FORWARD:

              left_motor_direction = MOTOR_FORWARD;
              right_motor_direction = MOTOR_FORWARD;              
              mouvement_control_state = STATE_FORWARD;
              break;
                       
            case CMD_BACKWARD:
              left_motor_direction = MOTOR_BACKWARD;
              right_motor_direction = MOTOR_BACKWARD;
              mouvement_control_state = STATE_BACKWARD;
              break;
                   
            case CMD_TURN_TO_FACE:

              left_motor_direction = MOTOR_FORWARD;
              right_motor_direction = MOTOR_STOP;            
              mouvement_control_state = STATE_TURN;
              break;
                   
            case CMD_WAIT:
              mouvement_control_state = STATE_WAIT;
              break;

            case CMD_DISTANCE:
             
              //// sensors
              compass_direction = read_compass();
              ultrasound_reading = read_ultrasound();
              // write distance and compass reading to RESPONSE_Q
              response.command = SLAM_READING;
              response.param1 = compass_direction;
              response.param2 = ultrasound_reading;
              write_q(&response_queue, response);

              break;        

            case SLAM_OVER:
              // slam over to RESPONSE_Q

              response.command = SLAM_OVER;
              write_q(&response_queue, response);
              break;

            case CALIBRATION_START: // calibration state

              calibration_steps = 0;
              average_magnetic_x = 0.0;
              average_magnetic_y = 0.0;
              mouvement_control_state = STATE_CALIBRATION;
              break;            
              
            default:
              break;
          }
      }
      break;

    case STATE_CALIBRATION: 
      if (calibration_steps < NUMBER_OF_CALIBRATION_READINGS)
      {      
        left_motor_direction = MOTOR_FORWARD;
        right_motor_direction = MOTOR_BACKWARD; 
        delay(100);
        read_compass();
        average_magnetic_x = average_magnetic_x + magnetic_x;
        average_magnetic_y = average_magnetic_y + magnetic_y;
        
        calibration_steps++; 
      }
      else
      {

        calibrate_x = average_magnetic_x / (float)NUMBER_OF_CALIBRATION_READINGS;
        calibrate_y = average_magnetic_y / (float)NUMBER_OF_CALIBRATION_READINGS;
        Serial.println(calibrate_x);
        Serial.println(calibrate_y);
        Serial.print("1\n ");
        // supervisor calibration state ends
        response.command = CALIBRATION_COMPLETE;
        write_q(&response_queue, response);
        mouvement_control_state = STATE_IDLE;
      }
      break;      

   // forward mouvement
    case STATE_FORWARD:
    
      delay(10*cmd.param1);
      left_motor_direction = MOTOR_STOP;
      right_motor_direction = MOTOR_STOP;
      break;
      
    // backward mouvement
    
    case STATE_BACKWARD:
      Serial.print("backward: ");
      delay(100*cmd.param1);
      break;
      
    // turining
    
    case STATE_TURN:
      
      float current_direction;
      float moving_direction;
      float m_d_minus;
      float m_d_plus;
      // add compass code that checks the compass direction to destination.
      current_direction = read_compass(); // read compass
      moving_direction = cmd.param1;

      if (moving_direction == 0.00 )
        {
          m_d_minus = 0;
          m_d_plus = 10;
        }
      else if (moving_direction == 360.00 )
        {
          m_d_minus = 350;
          m_d_plus = 360;
        }
      else 
      {
        m_d_minus = moving_direction-5;
        m_d_plus = moving_direction+5;
      }
      // if heading is the same as the desired heading stop motors
      if ( m_d_minus <= current_direction && current_direction <= m_d_plus)  
      {
        
        left_motor_direction = MOTOR_STOP;
        right_motor_direction = MOTOR_STOP;
        Serial.print("Exit Turning ");
        delay(50000);
        mouvement_control_state = STATE_IDLE; // If pointing to the right direction change state
        left_motor_direction = MOTOR_STOP;
        right_motor_direction = MOTOR_STOP;
      }  
      break; 


    case STATE_WAIT:

      left_motor_direction = MOTOR_STOP;
      right_motor_direction = MOTOR_STOP;
      break;

    default:
      break;
  }
}



/************************************************************************************/


void setup() 
{
  // intialising command
  Command cmd;
  Command cmd2;
  Command response;
  Serial.begin(9600);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* Problem detecting compass */
    Serial.println("Ooops, no compass detected");
    while(1);
  }
  
  // set motor output pins
  
  pinMode(leftForward , OUTPUT);
  pinMode(leftBackward , OUTPUT);
  pinMode(rightForward , OUTPUT);
  pinMode(rightBackward , OUTPUT);

  // intialise motor direction variables

  left_motor_direction = MOTOR_STOP;
  right_motor_direction = MOTOR_STOP;

  // intialise motor control queue 
  init_q(&motor_control_queue);


  // intialise reponse queue
  init_q(&response_queue);

  // initialise mouvement control
  mouvement_control_state = STATE_IDLE;
  
  // initialise supervisor
  supervisor_state = SUPERVISOR_NULL;
  enter_state(SUPERVISOR_IDLE);
 
  // Ultrasound sensor

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

}

void loop()
{
  supervisor();
  mouvement_control();
  left_motor(left_motor_direction);
  right_motor(right_motor_direction);
}
