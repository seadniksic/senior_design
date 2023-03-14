#include "locomotion.h"

void drive_left()
{
  BRBACK;
  BLFOR;
  FRFOR;
  FLBACK;
}

void drive_right()
{
  BRFOR;
  BLBACK;
  FRBACK;
  FLFOR;
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

void drive_diag_BL()
{
  FLBACK;
  BRBACK;
  
}

void drive_diag_BR()
{
  FRBACK;
  BLBACK; 
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

void rotate_CW()
{
  FLFOR;
  BLFOR;
  FRBACK;
  BRBACK;
}
void rotate_CCW()
{
  FRFOR;
  BRFOR;
  FLBACK;
  BLBACK;
}

void drive_forward()
{
  FLFOR;
  FRFOR;
  BLFOR;
  BRFOR;
}

void drive_backward()
{
  FLBACK;
  FRBACK;
  BLBACK;
  BRBACK;
}

void all_axis_off()
{
  AXIS_OFF(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  AXIS_OFF(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  AXIS_OFF(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  AXIS_OFF(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);
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

void locomotion_init()
{
  pinMode(13, OUTPUT);

  init_axis(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  init_axis(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  init_axis(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  init_axis(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);

  analogWriteFrequency(FL_EN_PIN, FREQ);
  analogWriteFrequency(FR_EN_PIN, FREQ);
  analogWriteFrequency(BR_EN_PIN, FREQ);
  analogWriteFrequency(BL_EN_PIN, FREQ);

  pinMode(FL_EN_PIN, OUTPUT);
  digitalWrite(FL_EN_PIN, LOW);
}

void locomotion_run()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }

  drive_forward();
  delay(2000);
  all_axis_off();

  while (true)
  {
    digitalWrite(13, HIGH);
  }
}

