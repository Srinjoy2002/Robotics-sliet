#include <QTRSensors.h>
#define PWM1 5 //LEFT MOTOR
#define AIN1 8
#define AIN2 7
#define PWM2 6 //RIGHT MOTOR
#define BIN1 4
#define BIN2 9
#define pushButton 2
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int dir_motor = 1;
int turnSpeed = 180;  //110
char path[100] = "";
unsigned char path_length = 0; // the length of the path
int range = 150;

void setup(){
                // configure the sensorValues
                qtr.setTypeAnalog();
                qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
                delay(500);
                pinMode(2, OUTPUT);
                pinMode(BIN1,OUTPUT);
                pinMode(AIN1,OUTPUT);
                pinMode(AIN2,OUTPUT);
                pinMode(BIN2,OUTPUT);
                pinMode(PWM1,OUTPUT);
                pinMode(PWM2,OUTPUT);
                pinMode(pushButton, INPUT);
                Serial.begin(9600);
                pinMode(12,OUTPUT);
                digitalWrite(12, HIGH);
                pinMode(3,OUTPUT);
                digitalWrite(3, HIGH);
                delay(3000);
                calibrationMode();
                delay(2000);
                /*int buttonState = digitalRead(pushButton);
              Serial.println(buttonState);
                if (buttonState == 1) {
                      delay(2000);
                      calibrationMode();   // turn the LED on (HIGH is the voltage level)
                  }*/
}


void loop(){
   //maze_solve();
   follow_segment();
  }



void set_Motors( int x, int y){
    if(x>=0){
              //FORWARD//LEFT MOTOR
              digitalWrite(AIN1,HIGH);
              digitalWrite(AIN2,LOW);
              //Speed control of Motor LEFT
              analogWrite(PWM1,x);
            }
    else if(x<0){
                  //FORWARD//LEFT MOTOR
                  digitalWrite(AIN1,LOW);
                  digitalWrite(AIN2,HIGH);
                  //Speed control of Motor LEFT
                  analogWrite(PWM1,-x);
                }
     if(y>=0){
                 //FORWARD//RIGHT MOTOR
                  digitalWrite(BIN1,HIGH);
                  digitalWrite(BIN2,LOW);
                //Speed control of Motor LEFT
                  analogWrite(PWM2,y);
             }
     else if(y<0){
                //FORWARD//LEFT MOTOR
                  digitalWrite(BIN1,LOW);
                  digitalWrite(BIN2,HIGH);
                //Speed control of Motor LEFT
                  analogWrite(PWM2,-y);
                  }
}





void follow_segment()
{
        int last_proportional = 0;
        long integral=0;
        float Kp = 0.4;       
        float Ki = 0.00001;    
        float Kd = 6;     
        while(1)
                {
                  
                   for (uint8_t i = 0; i < SensorCount; i++){
                    
                   }
                          
                        
                  
                  unsigned int position = qtr.readLineBlack(sensorValues);
                  Serial.println(position);
                  
                  int proportional = ((int)position)-3500;
                  Serial.println(proportional);
                  
                  int derivative = proportional - last_proportional;
                  integral += proportional;
                  
                  last_proportional = proportional;
                   
                  int power_difference = proportional*Kp + integral*Ki + derivative*Kd;
                  
                  const int max = 200; // the maximum speed//150
                  if(power_difference > max)
                           power_difference = max;
                  if(power_difference < -max)
                            power_difference = -max;
                  Serial.println(power_difference);
                  if(power_difference < 0)
                             set_Motors(max+power_difference,max);
                  else
                              set_Motors(max,max-power_difference);
                 
                  qtr.read(sensorValues);

                   if(sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5]<200 && sensorValues[6] < 200 && sensorValues[7] < 200 && sensorValues[0] < 200 && sensorValues[1]<200)
                  {
                          
                          
                         return;
                  }
                 
                      if((sensorValues[0] > 300 && sensorValues[7] > 300 && sensorValues[4] > 300 && sensorValues[3] > 300)||(sensorValues[0] > 300 && sensorValues[1] > 300 && 
                      sensorValues[3] > 300 && sensorValues[4] > 300)|| (sensorValues[7] > 300 && sensorValues[6] > 300 && sensorValues[3] > 300 && sensorValues[4] > 300) )
                  {   // Found an intersection.
                    Serial.println("INTERSECTION");
                    return;
                    
                  }
                  
                }
}



void calibrationMode(){
                        digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
                        analogWrite (PWM1, 200);
                        analogWrite (PWM2, 200);
                        // analogRead() takes about 0.1 ms on an AVR.
                        // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensorValues
                        // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
                        // Call calibrate() 100 times to make calibration take about 3.2 seconds.
                        for (uint16_t i = 0; i < 100; i++)
                            {
                                qtr.calibrate();
                                    if (i % 20 == 0){
                                                      //dir_motor = -dir_motor;
                                                      //change_direction(dir_motor);
                                                    }
                             }
                        analogWrite (PWM1, 0);
                        analogWrite (PWM2, 0);
                        digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration
}


void change_direction(int dir){
      if (dir == 1) {
                      digitalWrite (AIN1, HIGH);
                      digitalWrite (AIN2, LOW);
                      analogWrite (PWM1, 200);
                      digitalWrite (BIN1, LOW);
                      digitalWrite (BIN2, HIGH);
                      analogWrite (PWM2, 200);
                    }
      else {
                      digitalWrite (AIN1, LOW);
                      digitalWrite (AIN2, HIGH);
                      analogWrite (PWM1, 200);
                      digitalWrite (BIN1, HIGH);
                      digitalWrite (BIN2, LOW);
                      analogWrite (PWM2, 200);
            }
}




// Turns according to the parameter dir, which should be 'L', 'R', 'S'
// (straight), or 'B' (back).
void Turn(char dir)
{
  switch(dir)
  {
  case 'R':
    // Turn left.
    set_Motors(-180,200);
    delay(300);
    break;
  case 'L':
    // Turn right.
    set_Motors(200,-180);
    delay(300); //255//260
    break;
  case 'B':
    // Turn around.
    set_Motors(200,-180); //80
    delay(700); //810//850//860//865
    break;
  case 'S':
    // Don't do anything!
    break;
  }
}


char select_turn(unsigned char found_right, unsigned char found_straight,unsigned char found_left)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if(found_right)
    return 'R';
   else if(found_straight)
    return 'S';
   //else if(found_back)
    //return 'B';
  else if(found_left)
    return 'L';
  else
    return 'B';
}


void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
    switch(path[path_length-i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch(total_angle)
  {
  case 0:
    path[path_length - 3] = 'S';
    break;
  case 90:
    path[path_length - 3] = 'R';
    break;
  case 180:
    path[path_length - 3] = 'B';
    break;
  case 270:
    path[path_length - 3] = 'L';
    break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}


// This function is called once, from main.c.
void maze_solve()
{
  // Loop until we have solved the maze.
  while(1)
  {
    // FIRST MAIN LOOP BODY  
   ; follow_segment();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    // Note that we are slowing down - this prevents the robot
    // from tipping forward too much.
      set_Motors(100,100);
      delay(200); //40//20

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left=0;
    unsigned char found_straight=0;
    unsigned char found_right=0;
    unsigned char found_back=0;

    // Now read the sensorValues and check the intersection type.
    const uint8_t SensorCount = 8;
    uint16_t sensorValues[SensorCount];
    qtr.read(sensorValues);

    // Check for left and right exits.
    if(sensorValues[0] > 300 && sensorValues[1] > 300 && sensorValues[2] > 300 && (sensorValues[3] > 300 || sensorValues[4] > 300))
      found_right = 1;
    if(sensorValues[7] > 300 && sensorValues[6]> 300 && sensorValues[5] > 300 && (sensorValues[4]> 300 || sensorValues[3]> 300 ))
     found_left = 1; 
    
        set_Motors(100,100);  //-30
       delay(150); //50

/*
    set_Motors(45,45);
    delay(200);
*/
    
     qtr.read(sensorValues);
    if(sensorValues[2] > 300 || sensorValues[3] > 300 || sensorValues[4] > 300 || sensorValues[5] > 300)   
    
      found_straight = 1;
       set_Motors(100,100);
       delay(200);
    if(sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && 
        sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
        found_back = 1;

      qtr.read(sensorValues);

    // Check for the ending spot.
    // If all three middle sensorValues are on dark black, we have
    // solved the maze.
    if(sensorValues[2] > 300 && sensorValues[3] > 300 && sensorValues[4] > 300 && sensorValues[5] > 300  )
      break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_right, found_straight, found_left);

    // Make the turn indicated by the path.
    Turn(dir);

    // Store the intersection in the path variable.
    //path[path_length] = dir;
    //path_length ++;

    // You should check to make sure that the path_length does not
    // exceed the bounds of the array.  We'll ignore that in this
    // example.

    // Simplify the learned path.
    //simplify_path();
  }
}
