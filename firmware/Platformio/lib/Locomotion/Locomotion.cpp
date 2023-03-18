#include "Locomotion.h"

void Locomotion_Drive_Left()
{
  BRBACK(ANALOG_WRITE_VAL);
  BLFOR(ANALOG_WRITE_VAL);
  FRFOR(ANALOG_WRITE_VAL);
  FLBACK(ANALOG_WRITE_VAL);
}

void Locomotion_Drive_Left(const uint8_t &val)
{
  BRBACK(val);
  BLFOR(val);
  FRFOR(val);
  FLBACK(val);
}

void Locomotion_Drive_Right()
{
  BRFOR(ANALOG_WRITE_VAL);
  BLBACK(ANALOG_WRITE_VAL);
  FRBACK(ANALOG_WRITE_VAL);
  FLFOR(ANALOG_WRITE_VAL);
}

void Locomotion_Drive_Right(const uint8_t &val)
{
  BRFOR(val);
  BLBACK(val);
  FRBACK(val);
  FLFOR(val);
}


void Locomotion_Drive_Forward()
{
  FLFOR(ANALOG_WRITE_VAL);
  FRFOR(ANALOG_WRITE_VAL);
  BLFOR(ANALOG_WRITE_VAL);
  BRFOR(ANALOG_WRITE_VAL);
}

void Locomotion_Drive_Forward(const uint8_t &val)
{
  FLFOR(val);
  FRFOR(val);
  BLFOR(val);
  BRFOR(val);
}

void Locomotion_Differential_Drive_Forward(const uint8_t &pwm_left, const uint8_t &pwm_right)
{
  FLFOR(pwm_left);
  BLFOR(pwm_left);

  FRFOR(pwm_right);
  BRFOR(pwm_right);
}

void Locomotion_Drive_Backward()
{
  FLBACK(ANALOG_WRITE_VAL);
  FRBACK(ANALOG_WRITE_VAL);
  BLBACK(ANALOG_WRITE_VAL);
  BRBACK(ANALOG_WRITE_VAL);
}

void Locomotion_Drive_Backward(const uint8_t &val)
{
  FLBACK(val);
  FRBACK(val);
  BLBACK(val);
  BRBACK(val);
}

void Locomotion_Differential_Drive_Backward(const uint8_t &pwm_left, const uint8_t &pwm_right)
{
  FLBACK(pwm_left);
  BLBACK(pwm_left);

  FRBACK(pwm_right);
  BRBACK(pwm_right);
}

void Locomotion_Drive_Diag_FL()
{
  FRFOR(ANALOG_WRITE_VAL);
  BLFOR(ANALOG_WRITE_VAL);
}

void Locomotion_Drive_Diag_FL(const uint8_t &val)
{
  FRFOR(val);
  BLFOR(val);
}

void Locomotion_Drive_Diag_FR()
{
  FLFOR(ANALOG_WRITE_VAL);
  BRFOR(ANALOG_WRITE_VAL);
}

void Locomotion_Drive_Diag_FR(const uint8_t &val)
{
  FLFOR(val);
  BRFOR(val);
}

void Locomotion_Drive_Diag_BL()
{
  FLBACK(ANALOG_WRITE_VAL);
  BRBACK(ANALOG_WRITE_VAL); 
}

void Locomotion_Drive_Diag_BL(const uint8_t &val)
{
  FLBACK(val);
  BRBACK(val); 
}

void Locomotion_Drive_Diag_BR()
{
  FRBACK(ANALOG_WRITE_VAL);
  BLBACK(ANALOG_WRITE_VAL); 
}

void Locomotion_Drive_Diag_BR(const uint8_t &val)
{
  FRBACK(val);
  BLBACK(val); 
}

void Locomotion_Lateral_Arc_CW()
{
  FLFOR(ANALOG_WRITE_VAL);
  FRBACK(ANALOG_WRITE_VAL);
}

void Locomotion_Lateral_Arc_CCW()
{
  FRFOR(ANALOG_WRITE_VAL);
  FLBACK(ANALOG_WRITE_VAL);
}

void Locomotion_Rotate_CW()
{
  FLFOR(ANALOG_WRITE_VAL);
  BLFOR(ANALOG_WRITE_VAL);
  FRBACK(ANALOG_WRITE_VAL);
  BRBACK(ANALOG_WRITE_VAL);
}

void Locomotion_Rotate_CW(const uint8_t &val)
{
  FLFOR(val);
  BLFOR(val);
  FRBACK(val);
  BRBACK(val);
}

void Locomotion_Rotate_CCW()
{
  FRFOR(ANALOG_WRITE_VAL);
  BRFOR(ANALOG_WRITE_VAL);
  FLBACK(ANALOG_WRITE_VAL);
  BLBACK(ANALOG_WRITE_VAL);
}

void Locomotion_Rotate_CCW(const uint8_t &val)
{
  FRFOR(val);
  BRFOR(val);
  FLBACK(val);
  BLBACK(val);
}

void Locomotion_All_Axis_Off()
{
  AXIS_OFF(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  AXIS_OFF(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  AXIS_OFF(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  AXIS_OFF(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);
}

void Locomotion_Init_Axis(uint8_t in1, uint8_t in2, uint8_t en)
{
  pinMode(en, OUTPUT);
  digitalWrite(en, LOW);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

}

void Locomotion_Init()
{
  pinMode(13, OUTPUT);

  Locomotion_Init_Axis(BR_IN1_PIN, BR_IN2_PIN, BR_EN_PIN);
  Locomotion_Init_Axis(BL_IN3_PIN, BL_IN4_PIN, BL_EN_PIN);
  Locomotion_Init_Axis(FR_IN4_PIN, FR_IN3_PIN, FR_EN_PIN);
  Locomotion_Init_Axis(FL_IN2_PIN, FL_IN1_PIN, FL_EN_PIN);

  analogWriteFrequency(FL_EN_PIN, FREQ);
  analogWriteFrequency(FR_EN_PIN, FREQ);
  analogWriteFrequency(BR_EN_PIN, FREQ);
  analogWriteFrequency(BL_EN_PIN, FREQ);

}

void Locomotion_Run()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }

  Locomotion_Drive_Forward();
  delay(2000);
  Locomotion_All_Axis_Off();

  while (true)
  {
    digitalWrite(13, HIGH);
  }
}


