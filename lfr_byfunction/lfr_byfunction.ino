#define left 6
#define center 7
#define right 8
#define IN1 2   //motor-1
#define IN2 3   //motor-1
#define IN3 4   //motor-2
#define IN4 5   //motor-2



void setup() {
  Serial.begin(9600);
  pinMode(left, INPUT);
  pinMode(center, INPUT);
  pinMode(right, INPUT);

  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
}

void loop() {
  bool l = digitalRead(left);
  bool m = digitalRead(center);
  bool r = digitalRead(right);

  

  if (l == 1 && m == 0 && r == 1) {
    botforward();
  } else if (l == 0 && m == 0 && r == 0) {
    botStop();
  } else if (l == 1 && m== 1 && r == 1) {
    botUturn();
  } else if (l == 0 &&  m== 0 && r == 1) {
    botleft();
  } else if (l == 1 &&  m== 0 && r == 0) {
    botright();
  } else if (l == 0 &&  m== 1 && r == 1) {
    botleft();
  } else if (l == 1 &&  m== 1 && r == 0) {
    botright();
  }
}

void botforward() {
  
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
}
void botleft() {
  
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
}
void botright() {
  
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
void botStop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
void botUturn(){
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
  
