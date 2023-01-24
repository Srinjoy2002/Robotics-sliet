#define S1 0
#define S2 1
#define S3 2
#define S4 3
#define S5 4
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(S1,INPUT);
  pinMode(S2,INPUT);
  pinMode(S3,INPUT);
  pinMode(S4,INPUT);
  pinMode(S5,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int a=digitalRead(S1);
  int b=digitalRead(S2);
  int c=digitalRead(S3);
  int d=digitalRead(S4);
  int e=digitalRead(S5);

  Serial.print(a);
  Serial.print(b);
  Serial.print(c);
  Serial.print(d);
  Serial.println(e);
  
  
}
