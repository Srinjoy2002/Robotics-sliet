#define left_IR 1
#define mid_IR 3
#define right_IR 5
#define IN1_m1 6
#define IN2_m1 7
#define IN1_m2 8
#define IN2_m2 9







void setup() {
  Serial.begin(9600);
  pinMode(left_IR,INPUT);
  pinMode(mid_IR,INPUT);
  pinMode(right_IR,INPUT);
  pinMode(IN1_m1,OUTPUT);
  pinMode(IN2_m1,OUTPUT);
  pinMode(IN1_m2,OUTPUT);
  pinMode(IN2_m2,OUTPUT);
 
}

void loop() {
  int l=digitalRead(left_IR);
  int m=digitalRead(mid_IR);
  int r=digitalRead(right_IR);

  if(l==0&&m==0&&r==0){
    digitalWrite(IN1_m1,1);
    digitalWrite(IN2_m1,0);  //u-turn
    digitalWrite(IN1_m2,0);
    digitalWrite(IN2_m2,1);
  }
  else if (l==1&&m==1&&r==0){
    digitalWrite(IN1_m1,0);
    digitalWrite(IN2_m1,0);   //left
    digitalWrite(IN1_m2,1);
    digitalWrite(IN2_m2,0);
  }
  else if (l==0&&m==1&&r==1){
    digitalWrite(IN1_m1,1);
    digitalWrite(IN2_m1,0);
    digitalWrite(IN1_m2,0);  //right
    digitalWrite(IN2_m2,0);
  }
  else if (l==1&&m==1&&r==0){
    digitalWrite(IN1_m1,1);
    digitalWrite(IN2_m1,0);
    digitalWrite(IN1_m2,0);   //hard left
    digitalWrite(IN2_m2,1);
  }
  else if (l==0&&m==1&&r==1){
    digitalWrite(IN1_m1,0);
    digitalWrite(IN2_m1,1);
    digitalWrite(IN1_m2,1);   //hard right
    digitalWrite(IN2_m2,0);
  }
  else if (l==1&&m==1&&r==0){
    digitalWrite(IN1_m1,0);
    digitalWrite(IN2_m1,1);
    digitalWrite(IN1_m2,1);   //t-joint
    digitalWrite(IN2_m2,0);
  }
  else if (l==0&&m==1&&r==0){
    digitalWrite(IN1_m1,1);
    digitalWrite(IN2_m1,0);
    digitalWrite(IN1_m2,0);   //straight
    digitalWrite(IN2_m2,0);
  }
  else{
    digitalWrite(IN1_m1,1);
    digitalWrite(IN2_m1,0);
    digitalWrite(IN1_m2,1);   
    digitalWrite(IN2_m2,0);
  }
}
