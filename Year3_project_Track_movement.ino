int in1 = 3, in2 = 4, enA = 5, in3 = 6, in4 = 7, enB = 9;

void setup() {
  pinMode(in1, OUTPUT), pinMode(in2, OUTPUT), pinMode(enA, OUTPUT); //Motor One
  pinMode(in3, OUTPUT), pinMode(in4, OUTPUT), pinMode(enB, OUTPUT); //Motor Two
}

void loop() {
  Serial.begin(9600);
  if(Serial.available() > 0){
    int Direction = Serial.read();
    switch(Direction){
      case 'W':
      Forward();
      break;

      case 'S':
      Reverse();
      break;

      case 'A':
      Left();
      break;

      case 'D':
      Right();
      break;

      default:
      Stop();
      break;
    }
  }
}

void Forward(){
  digitalWrite(in1, HIGH), digitalWrite(in2, LOW), analogWrite(enA, 255);
  digitalWrite(in3, HIGH), digitalWrite(in4, LOW), analogWrite(enB, 255);
}

void Reverse(){
  digitalWrite(in1, LOW), digitalWrite(in2, HIGH), analogWrite(enA, 255);
  digitalWrite(in3, LOW), digitalWrite(in4, HIGH), analogWrite(enB, 255);
}

void Left(){
  digitalWrite(in1, LOW), digitalWrite(in2, HIGH), analogWrite(enA, 255);
  digitalWrite(in3, HIGH), digitalWrite(in4, LOW), analogWrite(enB, 255);
}

void Right(){
  digitalWrite(in1, HIGH), digitalWrite(in2, LOW), analogWrite(enA, 255);
  digitalWrite(in3, LOW), digitalWrite(in4, HIGH), analogWrite(enB, 255);
}

void Stop(){
  digitalWrite(in1, LOW), digitalWrite(in2, LOW), analogWrite(enA, 255);
  digitalWrite(in3, LOW), digitalWrite(in4, LOW), analogWrite(enB, 255);
}
