/*
  Arduino Uno R3 — Quadcopter flight controller (stabilized mode)
  - MPU6050 (I²C) + Mahony filter (quaternions) -> Euler angles
  - PID on angular rates (deg/s)
  - RC input via Pin Change Interrupts (D8..D11)
  - ESC PWM output via direct PORTD (D4..D7)
  - Fixed loop period ~5200 us (~200 Hz)
*/

#include <Wire.h>

// ---------- Channels ----------
#define CANAL1 0  // roll
#define CANAL2 1  // pitch
#define CANAL3 2  // throttle
#define CANAL4 3  // yaw

// ---------- Axes / indexes ----------
#define X 0
#define Y 1
#define Z 2

#define ROLL  0
#define PITCH 1
#define YAW   2
#define GAZ   3

// ---------- ESC indexes ----------
#define ESC1 0
#define ESC2 1
#define ESC3 2
#define ESC4 3

// ---------- States ----------
#define ARRET  0
#define ARME   1
#define MARCHE 2

// ---------- Pins ----------
#define LED 12

// ---------- MPU ----------
#define MPU_ADDR 0x68

// ---------- Units / constants ----------
#define MY_RAD_TO_DEG 57.2957795f
#define MY_DEG_TO_RAD 0.01745329f
#define INV_MICROS_TO_SEC 0.000001f

#define ACCEL_SCALE_INV (1.0f / 4096.0f)  // accel fullscale configured in init
#define GYRO_SCALE_INV  (1.0f / 65.5f)    // gyro fullscale configured in init

// ---------- Control tuning ----------
#define DEADBAND_ANGLE 0.9f

#define INTEGRAL_DECAY_GENERAL 0.98f
#define INTEGRAL_DECAY_RELAX   0.95f
#define ANGLE_RELAX_THRESHOLD  2.0f
#define INTEGRAL_THRESHOLD     2.0f

const float kpRoll  = 0.475f;
const float kiRoll  = 0.017f;
const float kdRoll  = 1.95f;

const float kpPitch = 0.475f;
const float kiPitch = 0.017f;
const float kdPitch = 1.95f;

const float kpYaw   = 2.75f;
const float kiYaw   = 0.02f;
const float kdYaw   = 0.00f;

const int max_degre_par_sec = 95;
const int min_degre_par_sec = -95;

const float coef_stabilisation = 3.0f;

float yaw_target = 0.0f;
const float yaw_deadband  = 2.0f;
const float yaw_hold_gain = 2.0f;

const float limite_pid_roll  = 400.0f;
const float limite_pid_pitch = 400.0f;
const float limite_pid_yaw   = 400.0f;

// ---------- Globals ----------
unsigned long dt_us = 0;
unsigned long memo_temps = 0;

volatile unsigned long instant_courant = 0;
volatile unsigned long debut_impulsion[4] = {0, 0, 0, 0};
volatile unsigned int  duree_impulsion[4] = {1500, 1500, 1000, 1500};

unsigned int   duree_impulsion_ESC[4] = {1000, 1000, 1000, 1000};
unsigned long  fin_impulsion_ESC[4]   = {0, 0, 0, 0};
unsigned long  debut_impulsion_ESC     = 0;
unsigned long  instant_actuel          = 0;

float acceleration_raw[3]    = {0, 0, 0};
float gyroscope_raw[3]       = {0, 0, 0};

float acceleration_offset[3] = {0, 0, 0};
float gyroscope_offset[3]    = {0, 0, 0};

float acceleration_lisse[3]  = {0, 0, 0};
float gyroscope_lisse[3]     = {0, 0, 0};

float accel[3]               = {0, 0, 0};
float gyro[3]                = {0, 0, 0};
float angle[3]               = {0, 0, 0};

bool init_angle_gyro = false;

float consigne[4]    = {0, 0, 0, 0};
float correct_P[3]   = {0, 0, 0};
float correct_I[3]   = {0, 0, 0};
float correct_D[3]   = {0, 0, 0};
float correct_PID[3] = {0, 0, 0};

float limit_PID[3]   = {limite_pid_roll, limite_pid_pitch, limite_pid_yaw};

float erreur[3]          = {0, 0, 0};
float erreur_integral[3] = {0, 0, 0};
float erreur_derivee[3]  = {0, 0, 0};
float memo_erreur[3]     = {0, 0, 0};

float ajustement_roll  = 0;
float ajustement_pitch = 0;

unsigned long debut_loop = 0;
byte etat = ARRET;

// ---------- Quaternion ----------
struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};

// Mahony
float integralFBx = 0, integralFBy = 0, integralFBz = 0;
const float Mahony_Kp = 2.0f;
const float Mahony_Ki = 0.05f;

// ---------- Helpers ----------
inline __attribute__((always_inline)) float borner(float v, float mn, float mx) {
  return (v < mn) ? mn : (v > mx ? mx : v);
}

inline __attribute__((always_inline)) float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline __attribute__((always_inline)) float fastInvSqrt(float x) {
  union { float f; long i; } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
  return conv.f;
}

// ---------- IO setup ----------
void configurer_sorties() {
  DDRD |= B11110000;  // D4..D7 outputs (ESC signals)
  DDRB |= B00010000;  // D12 output (LED)
}

void configuration() {
  PCICR  |= (1 << PCIE0);   // Enable pin change interrupts for PORTB
  PCMSK0 |= (1 << PCINT0);  // D8
  PCMSK0 |= (1 << PCINT1);  // D9
  PCMSK0 |= (1 << PCINT2);  // D10
  PCMSK0 |= (1 << PCINT3);  // D11
}

// ---------- MPU ----------
void initialisation_MPU() {
  Wire.begin();
  TWBR = 2;  // ~800 kHz I²C (ATmega328P)

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x80);  // reset
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);  // wake up
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);  // gyro config
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);  // accel config
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);  // DLPF config
  Wire.endTransmission();

  delay(250);
}

void lecture_MPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() >= 14) {
    acceleration_raw[X]  = (Wire.read() << 8) | Wire.read();
    acceleration_raw[Y]  = (Wire.read() << 8) | Wire.read();
    acceleration_raw[Z]  = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temperature
    gyroscope_raw[ROLL]  = (Wire.read() << 8) | Wire.read();
    gyroscope_raw[PITCH] = (Wire.read() << 8) | Wire.read();
    gyroscope_raw[YAW]   = (Wire.read() << 8) | Wire.read();
  }

  // Convention axes (project-specific)
  acceleration_raw[X]  *= -1;
  gyroscope_raw[PITCH] *= -1;
  gyroscope_raw[YAW]   *= -1;
}

void calibrer_MPU() {
  const int nb_mesures = 2000;

  for (int i = 0; i < nb_mesures; ++i) {
    if ((i % 20) == 0) digitalWrite(LED, !digitalRead(LED));

    lecture_MPU();

    acceleration_offset[X] += acceleration_raw[X] / nb_mesures;
    acceleration_offset[Y] += acceleration_raw[Y] / nb_mesures;
    acceleration_offset[Z] += acceleration_raw[Z] / nb_mesures;

    gyroscope_offset[ROLL]  += gyroscope_raw[ROLL]  / nb_mesures;
    gyroscope_offset[PITCH] += gyroscope_raw[PITCH] / nb_mesures;
    gyroscope_offset[YAW]   += gyroscope_raw[YAW]   / nb_mesures;

    // Keep ESCs alive during calibration
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;

    delay(3);
  }

  acceleration_offset[Z] -= 4096; // remove 1g
  digitalWrite(LED, LOW);
}

// ---------- Quaternion math ----------
Quaternion quat_multiply(Quaternion a, Quaternion b) {
  Quaternion r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}

Quaternion quat_normalize(Quaternion in) {
  float inv = fastInvSqrt(in.w*in.w + in.x*in.x + in.y*in.y + in.z*in.z);
  in.w *= inv; in.x *= inv; in.y *= inv; in.z *= inv;
  return in;
}

void get_euler_angles_from_quaternion() {
  angle[ROLL] = atan2(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * MY_RAD_TO_DEG;

  float s = 2.0f*(q.w*q.y - q.z*q.x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  angle[PITCH] = asin(s) * MY_RAD_TO_DEG;

  angle[YAW] = atan2(2.0f*(q.w*q.z + q.x*q.y), 1.0f - 2.0f*(q.y*q.y + q.z*q.z)) * MY_RAD_TO_DEG;
}

// Mahony update (gyro in deg/s, accel normalized)
inline __attribute__((always_inline)) void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, unsigned long dt_us_local) {
  const float dt_s = dt_us_local * INV_MICROS_TO_SEC;

  float inv = fastInvSqrt(ax*ax + ay*ay + az*az);
  ax *= inv; ay *= inv; az *= inv;

  // Estimated gravity direction
  const float vx = 2.0f*(q.x*q.z - q.w*q.y);
  const float vy = 2.0f*(q.w*q.x + q.y*q.z);
  const float vz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

  // Error (cross product)
  const float ex = (ay * vz - az * vy);
  const float ey = (az * vx - ax * vz);
  const float ez = (ax * vy - ay * vx);

  // Integral feedback
  const float Ki_dt = Mahony_Ki * dt_s;
  integralFBx += Ki_dt * ex;
  integralFBy += Ki_dt * ey;
  integralFBz += Ki_dt * ez;

  // Apply feedback to gyro
  gx += Mahony_Kp * ex + integralFBx;
  gy += Mahony_Kp * ey + integralFBy;
  gz += Mahony_Kp * ez + integralFBz;

  // Quaternion derivative
  Quaternion omega = {0.0f, gx * MY_DEG_TO_RAD, gy * MY_DEG_TO_RAD, gz * MY_DEG_TO_RAD};
  Quaternion qDot = quat_multiply(q, omega);

  qDot.w *= 0.5f; qDot.x *= 0.5f; qDot.y *= 0.5f; qDot.z *= 0.5f;

  q.w += qDot.w * dt_s;
  q.x += qDot.x * dt_s;
  q.y += qDot.y * dt_s;
  q.z += qDot.z * dt_s;

  q = quat_normalize(q);
}

// ---------- Fusion IMU ----------
void calcul_angle_fusion() {
  unsigned long t = micros();
  dt_us = t - memo_temps;
  if (dt_us == 0) dt_us = 5200;
  memo_temps = t;

  // Remove offsets
  acceleration_raw[X]  -= acceleration_offset[X];
  acceleration_raw[Y]  -= acceleration_offset[Y];
  acceleration_raw[Z]  -= acceleration_offset[Z];

  gyroscope_raw[ROLL]  -= gyroscope_offset[ROLL];
  gyroscope_raw[PITCH] -= gyroscope_offset[PITCH];
  gyroscope_raw[YAW]   -= gyroscope_offset[YAW];

  // Low-pass smoothing
  acceleration_lisse[X] = acceleration_lisse[X]*0.8f + acceleration_raw[X]*0.2f;
  acceleration_lisse[Y] = acceleration_lisse[Y]*0.8f + acceleration_raw[Y]*0.2f;
  acceleration_lisse[Z] = acceleration_lisse[Z]*0.8f + acceleration_raw[Z]*0.2f;

  gyroscope_lisse[ROLL]  = gyroscope_lisse[ROLL]*0.8f  + gyroscope_raw[ROLL]*0.2f;
  gyroscope_lisse[PITCH] = gyroscope_lisse[PITCH]*0.8f + gyroscope_raw[PITCH]*0.2f;
  gyroscope_lisse[YAW]   = gyroscope_lisse[YAW]*0.8f   + gyroscope_raw[YAW]*0.2f;

  // Scale to g and deg/s
  accel[X] = acceleration_lisse[X] * ACCEL_SCALE_INV;
  accel[Y] = acceleration_lisse[Y] * ACCEL_SCALE_INV;
  accel[Z] = acceleration_lisse[Z] * ACCEL_SCALE_INV;

  gyro[ROLL]  = gyroscope_lisse[ROLL]  * GYRO_SCALE_INV;
  gyro[PITCH] = gyroscope_lisse[PITCH] * GYRO_SCALE_INV;
  gyro[YAW]   = gyroscope_lisse[YAW]   * GYRO_SCALE_INV;

  mahony_update(gyro[ROLL], gyro[PITCH], gyro[YAW], accel[X], accel[Y], accel[Z], dt_us);
  get_euler_angles_from_quaternion();

  ajustement_roll  = angle[ROLL]  * coef_stabilisation;
  ajustement_pitch = angle[PITCH] * coef_stabilisation;

  if (!init_angle_gyro) {
    init_angle_gyro = true;
    angle[ROLL]  = atan2(accel[Y], accel[Z]) * MY_RAD_TO_DEG;
    angle[PITCH] = atan2(-accel[X], sqrt(accel[Y]*accel[Y] + accel[Z]*accel[Z])) * MY_RAD_TO_DEG;
    angle[YAW]   = 0.0f;
  }
}

// ---------- PID reset / safety ----------
void raz_controleurPID() {
  for (int i = 0; i < 3; ++i) {
    erreur[i] = 0;
    erreur_integral[i] = 0;
    erreur_derivee[i] = 0;
    memo_erreur[i] = 0;

    correct_P[i] = 0;
    correct_I[i] = 0;
    correct_D[i] = 0;
    correct_PID[i] = 0;
  }
}

void stopper_moteur() {
  duree_impulsion_ESC[ESC1] = 1000;
  duree_impulsion_ESC[ESC2] = 1000;
  duree_impulsion_ESC[ESC3] = 1000;
  duree_impulsion_ESC[ESC4] = 1000;
}

// ---------- RC mapping ----------
void calcul_consigne() {
  // Roll
  if (duree_impulsion[CANAL1] > 1508) {
    consigne[ROLL] = map_float(duree_impulsion[CANAL1], 1508, 2000, 0, max_degre_par_sec);
  } else if (duree_impulsion[CANAL1] < 1492) {
    consigne[ROLL] = map_float(duree_impulsion[CANAL1], 1000, 1492, min_degre_par_sec, 0);
  } else {
    consigne[ROLL] = 0;
  }

  // Pitch
  if (duree_impulsion[CANAL2] > 1508) {
    consigne[PITCH] = map_float(duree_impulsion[CANAL2], 1508, 2000, 0, min_degre_par_sec);
  } else if (duree_impulsion[CANAL2] < 1492) {
    consigne[PITCH] = map_float(duree_impulsion[CANAL2], 1000, 1492, max_degre_par_sec, 0);
  } else {
    consigne[PITCH] = 0;
  }

  // Yaw: rate command + simple hold when stick centered
  float cmd_yaw = 0.0f;
  if (duree_impulsion[CANAL3] > 1050) {
    if (duree_impulsion[CANAL4] > 1508) {
      cmd_yaw = map_float(duree_impulsion[CANAL4], 1508, 2000, 0, max_degre_par_sec);
    } else if (duree_impulsion[CANAL4] < 1492) {
      cmd_yaw = map_float(duree_impulsion[CANAL4], 1000, 1492, min_degre_par_sec, 0);
    } else {
      cmd_yaw = 0;
    }
  }

  if (abs(cmd_yaw) < yaw_deadband) {
    float yaw_error = yaw_target - angle[YAW];
    if (yaw_error > 180) yaw_error -= 360;
    if (yaw_error < -180) yaw_error += 360;
    consigne[YAW] = yaw_error * yaw_hold_gain;
  } else {
    consigne[YAW] = cmd_yaw;
    yaw_target = angle[YAW];
  }

  // Throttle
  consigne[GAZ] = duree_impulsion[CANAL3];
  if (consigne[GAZ] > 1700) consigne[GAZ] = 1700;

  // Stabilized mode correction
  consigne[ROLL]  -= ajustement_roll;
  consigne[PITCH] -= ajustement_pitch;
}

// ---------- PID compute ----------
void calcul_commande_pid() {
  erreur[ROLL]  = consigne[ROLL]  - gyro[ROLL];
  erreur[PITCH] = consigne[PITCH] - gyro[PITCH];
  erreur[YAW]   = consigne[YAW]   - gyro[YAW];

  for (int axis = 0; axis < 3; axis++) {
    if (abs(erreur[axis]) < DEADBAND_ANGLE) {
      erreur[axis] = 0;
      erreur_integral[axis] = 0;
    }
  }

  if (etat != MARCHE || consigne[GAZ] < 1050) {
    erreur_integral[ROLL]  = 0;
    erreur_integral[PITCH] = 0;
    erreur_integral[YAW]   = 0;
  } else {
    if (abs(angle[ROLL]) < ANGLE_RELAX_THRESHOLD && abs(angle[PITCH]) < ANGLE_RELAX_THRESHOLD) {
      erreur_integral[ROLL]  *= INTEGRAL_DECAY_RELAX;
      erreur_integral[PITCH] *= INTEGRAL_DECAY_RELAX;
    }
    for (int axis = 0; axis < 3; axis++) {
      if (abs(erreur[axis]) > INTEGRAL_THRESHOLD) {
        erreur_integral[axis] *= INTEGRAL_DECAY_GENERAL;
      }
    }
  }

  correct_P[ROLL]  = kpRoll  * erreur[ROLL];
  correct_P[PITCH] = kpPitch * erreur[PITCH];
  correct_P[YAW]   = kpYaw   * erreur[YAW];

  erreur_integral[ROLL]  += erreur[ROLL];
  erreur_integral[PITCH] += erreur[PITCH];
  erreur_integral[YAW]   += erreur[YAW];

  correct_I[ROLL]  = kiRoll  * erreur_integral[ROLL];
  correct_I[PITCH] = kiPitch * erreur_integral[PITCH];
  correct_I[YAW]   = kiYaw   * erreur_integral[YAW];

  correct_I[ROLL]  = borner(correct_I[ROLL],  -limit_PID[ROLL],  limit_PID[ROLL]);
  correct_I[PITCH] = borner(correct_I[PITCH], -limit_PID[PITCH], limit_PID[PITCH]);
  correct_I[YAW]   = borner(correct_I[YAW],   -limit_PID[YAW],   limit_PID[YAW]);

  erreur_derivee[ROLL]  = erreur[ROLL]  - memo_erreur[ROLL];
  erreur_derivee[PITCH] = erreur[PITCH] - memo_erreur[PITCH];
  erreur_derivee[YAW]   = erreur[YAW]   - memo_erreur[YAW];

  memo_erreur[ROLL]  = erreur[ROLL];
  memo_erreur[PITCH] = erreur[PITCH];
  memo_erreur[YAW]   = erreur[YAW];

  correct_D[ROLL]  = kdRoll  * erreur_derivee[ROLL];
  correct_D[PITCH] = kdPitch * erreur_derivee[PITCH];
  correct_D[YAW]   = kdYaw   * erreur_derivee[YAW];

  correct_PID[ROLL]  = correct_P[ROLL]  + correct_I[ROLL]  + correct_D[ROLL];
  correct_PID[PITCH] = correct_P[PITCH] + correct_I[PITCH] + correct_D[PITCH];
  correct_PID[YAW]   = correct_P[YAW]   + correct_I[YAW]   + correct_D[YAW];

  correct_PID[ROLL]  = borner(correct_PID[ROLL],  -limit_PID[ROLL],  limit_PID[ROLL]);
  correct_PID[PITCH] = borner(correct_PID[PITCH], -limit_PID[PITCH], limit_PID[PITCH]);
  correct_PID[YAW]   = borner(correct_PID[YAW],   -limit_PID[YAW],   limit_PID[YAW]);
}

// ---------- Motor mixing + PWM ----------
void calculer_impulsions_ESC() {
  duree_impulsion_ESC[ESC1] = consigne[GAZ] + correct_PID[PITCH] - correct_PID[ROLL] + correct_PID[YAW];
  duree_impulsion_ESC[ESC2] = consigne[GAZ] - correct_PID[PITCH] - correct_PID[ROLL] - correct_PID[YAW];
  duree_impulsion_ESC[ESC3] = consigne[GAZ] - correct_PID[PITCH] + correct_PID[ROLL] + correct_PID[YAW];
  duree_impulsion_ESC[ESC4] = consigne[GAZ] + correct_PID[PITCH] + correct_PID[ROLL] - correct_PID[YAW];

  duree_impulsion_ESC[ESC1] = borner(duree_impulsion_ESC[ESC1], 1100, 2000);
  duree_impulsion_ESC[ESC2] = borner(duree_impulsion_ESC[ESC2], 1100, 2000);
  duree_impulsion_ESC[ESC3] = borner(duree_impulsion_ESC[ESC3], 1100, 2000);
  duree_impulsion_ESC[ESC4] = borner(duree_impulsion_ESC[ESC4], 1100, 2000);
}

void generer_impulsion_ESC() {
  debut_impulsion_ESC = micros();

  PORTD |= B11110000; // D4..D7 HIGH

  fin_impulsion_ESC[ESC1] = debut_impulsion_ESC + duree_impulsion_ESC[ESC1];
  fin_impulsion_ESC[ESC2] = debut_impulsion_ESC + duree_impulsion_ESC[ESC2];
  fin_impulsion_ESC[ESC3] = debut_impulsion_ESC + duree_impulsion_ESC[ESC3];
  fin_impulsion_ESC[ESC4] = debut_impulsion_ESC + duree_impulsion_ESC[ESC4];

  while (PORTD >= 16) {
    instant_actuel = micros();
    if (fin_impulsion_ESC[ESC1] <= instant_actuel) { PORTD &= B11101111; }
    if (fin_impulsion_ESC[ESC2] <= instant_actuel) { PORTD &= B11011111; }
    if (fin_impulsion_ESC[ESC3] <= instant_actuel) { PORTD &= B10111111; }
    if (fin_impulsion_ESC[ESC4] <= instant_actuel) { PORTD &= B01111111; }
  }
}

// ---------- Arming ----------
void attendre_gaz_mini() {
  int cpt = 0;
  while (duree_impulsion[CANAL3] < 990  || duree_impulsion[CANAL3] > 1020 ||
         duree_impulsion[CANAL4] < 1450 || duree_impulsion[CANAL4] > 1550) {

    cpt++;

    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;

    delay(3);

    if (cpt == 125) {
      digitalWrite(LED, !digitalRead(LED));
      cpt = 0;
    }
  }
  digitalWrite(LED, LOW);
}

// ---------- Arduino entry points ----------
void setup() {
  configurer_sorties();
  delay(3000);

  initialisation_MPU();
  calibrer_MPU();

  configuration();  // RC interrupts

  attendre_gaz_mini();

  etat = ARRET;
  memo_temps = micros() - 5200;

  lecture_MPU();
}

void loop() {
  debut_loop = micros();

  lecture_MPU();
  calcul_angle_fusion();

  // Fail-safe
  for (int i = 0; i < 4; ++i) {
    if (duree_impulsion[i] < 900 || duree_impulsion[i] > 2100) {
      etat = ARRET;
      stopper_moteur();
      digitalWrite(LED, HIGH);
      return;
    }
  }

  // State machine (arming)
  if (etat == ARRET && duree_impulsion[CANAL3] < 1050 && duree_impulsion[CANAL4] < 1050) {
    etat = ARME;
  }
  if (etat == ARME && duree_impulsion[CANAL3] < 1050 && duree_impulsion[CANAL4] > 1450) {
    etat = MARCHE;
    raz_controleurPID();
  }
  if (etat == MARCHE && duree_impulsion[CANAL3] < 1050 && duree_impulsion[CANAL4] > 1950) {
    etat = ARRET;
  }

  calcul_consigne();
  calcul_commande_pid();

  if (etat == MARCHE) {
    calculer_impulsions_ESC();
  } else {
    stopper_moteur();
  }

  generer_impulsion_ESC();

  while (micros() - debut_loop < 5200) { /* fixed loop period */ }
}

// ---------- RC input ISR (D8..D11) ----------
ISR(PCINT0_vect) {
  static byte etat_prec = 0;
  byte etat_actuel = PINB;
  byte changement  = etat_prec ^ etat_actuel;

  instant_courant = micros();

  if (changement & (1 << PB0)) { // D8
    if (etat_actuel & (1 << PB0)) debut_impulsion[CANAL1] = instant_courant;
    else duree_impulsion[CANAL1] = instant_courant - debut_impulsion[CANAL1];
  }
  if (changement & (1 << PB1)) { // D9
    if (etat_actuel & (1 << PB1)) debut_impulsion[CANAL2] = instant_courant;
    else duree_impulsion[CANAL2] = instant_courant - debut_impulsion[CANAL2];
  }
  if (changement & (1 << PB2)) { // D10
    if (etat_actuel & (1 << PB2)) debut_impulsion[CANAL3] = instant_courant;
    else duree_impulsion[CANAL3] = instant_courant - debut_impulsion[CANAL3];
  }
  if (changement & (1 << PB3)) { // D11
    if (etat_actuel & (1 << PB3)) debut_impulsion[CANAL4] = instant_courant;
    else duree_impulsion[CANAL4] = instant_courant - debut_impulsion[CANAL4];
  }

  etat_prec = etat_actuel;
}
