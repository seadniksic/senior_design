
// BR = Back Right
#define BR_IN1_PIN 30
#define BR_IN2_PIN 31
#define BR_EN_PIN 2

// BL = Back Left
#define BL_IN3_PIN 26
#define BL_IN4_PIN 27
#define BL_EN_PIN 3

// FR = Front Right
#define FR_IN3_PIN 34
#define FR_IN4_PIN 35
#define FR_EN_PIN 29

// FL = Front Left
#define FL_IN1_PIN 36
#define FL_IN2_PIN 37
#define FL_EN_PIN 28

#define FRFOR axis_forward(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN)
#define FLFOR axis_forward(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN)
#define BRFOR axis_forward(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN)
#define BLFOR axis_forward(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN)
#define FRBACK axis_backward(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN)
#define FLBACK axis_backward(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN)
#define BRBACK axis_backward(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN)
#define BLBACK axis_backward(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN)
#define BLON axis_on(BL_EN_PIN);
#define BRON axis_on(BR_EN_PIN);


void init_axis(uint8_t in1, uint8_t in2, uint8_t en);
void axis_forward(uint8_t in1, uint8_t in2, uint8_t en);
void axis_backward(uint8_t in1, uint8_t in2, uint8_t en);
void axis_off(uint8_t in1, uint8_t in2, uint8_t en);
void axis_on(uint8_t en);

void drive_left()
{
  axis_backward(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  axis_forward(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  axis_forward(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  axis_backward(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);
}

void drive_diag_FL()
{
  FRFOR;
  BLFOR;
}

void drive_diag_FR()
{
  FLFOR;
  BRFOR;
}

void lateral_arc_CW()
{
  FLFOR;
  FRBACK;
}

void lateral_arc_CCW()
{
  FRFOR;
  FLBACK;
}

void drive_forward()
{
  FLFOR;
  FRFOR;
  BLFOR;
  BRFOR;
}






void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);

  init_axis(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  init_axis(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  init_axis(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  init_axis(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);

  pinMode(FL_EN_PIN, OUTPUT);
  digitalWrite(FL_EN_PIN, LOW);


}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }

  drive_forward();
  delay(1000);
  axis_off(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  axis_off(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  axis_off(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  axis_off(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);

  while (true)
  {
    digitalWrite(13, HIGH);
  }

}

void init_axis(uint8_t in1, uint8_t in2, uint8_t en)
{
  pinMode(en, OUTPUT);
  digitalWrite(en, LOW);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

}

void axis_forward(uint8_t in1, uint8_t in2, uint8_t en)
{
  digitalWrite(en, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void axis_backward(uint8_t in1, uint8_t in2, uint8_t en)
{
  digitalWrite(en, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void axis_off(uint8_t in1, uint8_t in2, uint8_t en)
{
  digitalWrite(en, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void axis_on(uint8_t en)
{
  digitalWrite(en, HIGH);
}
