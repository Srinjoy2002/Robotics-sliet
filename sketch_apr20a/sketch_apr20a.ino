void follow_segment()
{
        int last_proportional = 0;
        long integral=0;
        float Kp = 0.8;       //0.1 // for maxspeed= 180, Kp=0.40, Ki=0.00001, Kd = 8.55
        float Ki = 0.00001;    
        float Kd = 3;     //1.2//20 //for max speed =140, kp = 0.40, kd= 6
        while(1)
                {
                  
                   for (uint8_t i = 0; i < SensorCount; i++){
                    
                   }
                          
                        
                                                                      // Get the position of the line.
                  unsigned int position = qtr.readLineBlack(sensorValues);
                                                                    //Serial.println(position);
                                                                   // The "proportional" term should be 0 when we are on the line.
                  int proportional = ((int)position)-3500;
                                                                 //Serial.println(proportional);
                                                                // Compute the derivative (change) and integral (sum) of the
                                                               // position.
                  int derivative = proportional - last_proportional;
                  integral += proportional;
                                                              // Remember the last position.
                  last_proportional = proportional;
                           // Compute the difference between the two motor power settings,
                          // m1 - m2. If this is a positive number the robot will turn
                          // to the left. If it is a negative number, the robot will
                          // turn to the right, and the magnitude of the number determines
                          // the sharpness of the turn.
                  int power_difference = P*Kp + I*Ki + D*Kd;
                          // Compute the actual motor settings. We never set either motor
                          // to a negative value.
                  const int max = 110; // the maximum speed//150
                  if(power_difference > max)
                           power_difference = max;
                  if(power_difference < -max)
                            power_difference = -max;
                          //Serial.println(power_difference);
                  if(power_difference < 0)
                             set_Motors(max+power_difference,max);
                  else
                              set_Motors(max,max-power_difference);
                         // We use the inner three sensorValues (1, 2, and 3) for
                         // determining whether there is a line straight ahead, and the
                         // sensorValues 0 and 4 for detecting lines going to the left and
                         // right.
