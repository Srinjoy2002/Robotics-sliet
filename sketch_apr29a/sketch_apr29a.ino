#define left 10
#define center 11
#define right 12

//motor one
#define ENA 5
#define IN1 8
#define IN2 7

//motor two
#define ENB 6
#define IN3 9
#define IN4 4

int Speed = 120; // speed of this robot

void setup() {
  Serial.begin(9600);
  pinMode(left, INPUT);
  pinMode(center, INPUT);
  pinMode(right, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  bool leftV = digitalRead(left);
  bool centerV = digitalRead(center);
  bool rightV = digitalRead(right);

  Serial.println(rightV);

  if (leftV == 1 && centerV == 0 && rightV == 1) {
    carforward();
    Serial.println("forward");
  } else if (leftV == 0 && centerV == 0 && rightV == 0) {
    carturnleft();
  } else if (leftV == 1 && centerV == 1 && rightV == 1) {
    carturnright();
  } else if (leftV == 0 && centerV == 1 && rightV == 1) {
    carturnright();
  } else if (leftV == 1 && centerV == 1 && rightV == 0) {
    carturnleft();
  
//  } else if (leftV == 0 && centerV == 1 && rightV == 1) {
//    carturnleft();
//  } else if (leftV == 1 && centerV == 1 && rightV == 0) {
//    carturnright();
  }
}

void carforward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void carturnleft() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void carturnright() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void carStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
