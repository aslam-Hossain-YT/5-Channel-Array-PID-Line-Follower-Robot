//motor pin defines
#define IN1 6
#define IN2 5
#define IN3 4
#define IN4 2
#define enA 9
#define enB 3

//declaration of necessary variable to line follow
int s[6], total, sensor_position;
int threshold = 531; //set threshold as like i've shown in video
float avg;
int position[6] = { 1, 2, 3, 4, 5 };
int set_point = 3;

//push button and led
int button_pin = 12;
int led = 13;
bool button_pin_state;

void setup() {
  //motor driver pins as output
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //button pin as input
  pinMode(button_pin, INPUT_PULLUP);
  //led pin as output
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  button_pin_state = digitalRead(button_pin);
  display_value();

  //when button is pressed then the robot will start to follow lines.
  while (button_pin_state != 1) {
    //blink led
    for (int i = 0; i < 3; ++i) {
      digitalWrite(led, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      delay(100);
    }
    PID_LINE_FOLLOW();  //line follow using PID_Value
  }
}

void Sensor_reading() {
  sensor_position = 0;
  total = 0;
  for (byte i = 0; i < 5; i++) {  //snesor data read from A0, A1, A2, A6, A7
    if (i > 2) {
      s[i] = analogRead(i + 3);
    } else {
      s[i] = analogRead(i);
    }
    if (s[i] > threshold) s[i] = 1;  //analog value to digital conversion
    else s[i] = 0;
    sensor_position += s[i] * position[i];
    total += s[i];
  }
  if (total) avg = sensor_position / total;  //average value
}

void PID_LINE_FOLLOW() {

  int kp = 50, kd = 500, PID_Value, P, D;
  float error, previous_error;
  int base_speed = 200, left_motor_speed, right_motor_speed, turn_speed = 100;
  char t;

  while (1) {
    Sensor_reading();
    error = set_point - avg;
    D = kd * (error - previous_error);
    P = error * kp;
    PID_Value = P + D;
    previous_error = error;

    //adjusting motor speed to keep the robot in line
    right_motor_speed = base_speed - PID_Value;  //right motor speed
    left_motor_speed = base_speed + PID_Value;  //left motor speed
    motor(left_motor_speed, right_motor_speed);               //when robot in straight position

    Sensor_reading();
    if (total == 0) { 
      digitalWrite(led, HIGH);
      if (t != 's') {
        if (t == 'r') motor(turn_speed, -turn_speed); 
        else motor(-turn_speed, turn_speed);          
        while (!s[2]) Sensor_reading();
        digitalWrite(led, LOW);
      }
    }

    if (s[4] == 1 && s[0] == 0) t = 'l';
    if (s[4] == 0 && s[0] == 1) t = 'r';

    else if (total == 5) { 
      Sensor_reading();
      if (total == 5) {
        motor(0, 0);
        while (total == 5) Sensor_reading();
      } else if (total == 0) t = 's';
    }
  }
}

void display_value() {  //display the analog value of sensor in serial monitor
  for (byte i = 0; i < 5; i++) {
    if (i > 2) {
      s[i] = analogRead(i + 3);
    } else {
      s[i] = analogRead(i);
    }
    Serial.print(String(s[i]) + " ");
  }
  Serial.println();
  delay(50);
}

//motor run function
void motor(int a, int b) {
  if (a > 0) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
  } else {
    a = -(a);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
  }
  if (b > 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
  } else {
    b = -(b);
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
  }

  if (a > 200) a = 200;
  if (b > 200) b = 200;

  analogWrite(enB, a);
  analogWrite(enA, b);
}
