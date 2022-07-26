
#define X_AXIS_STEP_PIN 2
#define Y_AXIS_STEP_PIN 3
#define Z_AXIS_STEP_PIN 4
#define A_AXIS_STEP_PIN 12

#define X_AXIS_DIR_PIN 5
#define Y_AXIS_DIR_PIN 6
#define Z_AXIS_DIR_PIN 7
#define A_AXIS_DIR_PIN 13

#define STEPPER_ENABLE_PIN 8

// Delta robot parameters
const float L = 100;
const float l = 400;
const float wb = 56;
const float up = 24.14;
const float sp = 41.78;
const float wp = 12.06;


float th12 = 0, th22 = 0, th32 = 0;

bool flag = true;

int microstep = 32;
int step_per_rev = 200 * 32;
float steps_per_degree = step_per_rev / 360;

//initial motor angles
int cur_x = 0;
int cur_y = 0;
int cur_z = 0;

int spd = 100;
int acc = 100;

#include <math.h>


// ================================================================================
// Import the third-party AccelStepper library.
#include <AccelStepper.h>

// Declare three AccelStepper objects to manage the output timing.
AccelStepper xaxis(AccelStepper::DRIVER, X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AccelStepper yaxis(AccelStepper::DRIVER, Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);
AccelStepper zaxis(AccelStepper::DRIVER, Z_AXIS_STEP_PIN, Z_AXIS_DIR_PIN);
AccelStepper aaxis(AccelStepper::DRIVER, A_AXIS_STEP_PIN, A_AXIS_DIR_PIN);

// ================================================================================
/// Configure the hardware once after booting up.  This runs once after powering
/// up the board, pressing reset, or connecting to the console serial port.
void setup(void)
{
  // set up the CNC Shield I/O
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // initialize drivers in disabled state
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  xaxis.setMaxSpeed(spd);
  xaxis.setAcceleration(acc);

  yaxis.setMaxSpeed(spd);
  yaxis.setAcceleration(acc);

  zaxis.setMaxSpeed(spd);
  zaxis.setAcceleration(acc);

  aaxis.setMaxSpeed(spd);
  aaxis.setAcceleration(acc);

  // set up the serial port for debugging output
  Serial.begin(115200);

  // enable the drivers, the motors will remain constantly energized
  digitalWrite(STEPPER_ENABLE_PIN, LOW);


  move(500, 500, 500, 500);
  delay(2000);

}
/**********************/
/// Call the ``run()`` function for each stepper driver object, which will
/// recalculate speeds and generate step pulses as needed.
void poll_steppers(void)

{
  xaxis.run();
  yaxis.run();
  zaxis.run();
  aaxis.run();
}
/// Return true if any one of the drivers are still moving.
bool is_moving(void)
{
  return (xaxis.isRunning() || yaxis.isRunning() || zaxis.isRunning() || aaxis.isRunning());
}

/// Move a relative displacement at the current speed, blocking until the move is done.
void move(long x, long y, long z, long a)
{
  Serial.print("Moving to ");
  Serial.print(x); Serial.print(", "); Serial.print(y); Serial.print(", "); Serial.print(z); Serial.print(", ");
  Serial.println(a);

  xaxis.move(x);
  yaxis.move(y);
  zaxis.move(z);
  aaxis.move(a);

  do {
    poll_steppers();
  } while (is_moving());
}

/**********************/

void fk(float x, float y, float z)
{
  // The constraint equation of the legs are fo the form
  // E_i*cos(th_i) + F_i*sin(th_i) + G_i = 0
  // Defining the constants E_i, F_i and G_i

  float a = wb - up;
  float b = (sp - sqrt(3) * wb) / 2;
  float c = wp - wb / 2;

  float E1 = 2 * L * (y + a);
  float F1 = 2 * z * L;
  float G1 = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(a, 2) + pow(L, 2) + 2 * y * a - pow(l, 2);

  float E2 = -L * ((sqrt(3) * (x + b)) + y + c);
  float F2 = 2 * z * L;
  float G2 = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(b, 2) + pow(c, 2) + pow(L, 2) + 2 * (x * b + y * c) - pow(l, 2);

  float E3 = L * ((sqrt(3) * (x - b)) - y - c);
  float F3 = 2 * z * L;
  float G3 = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(b, 2) + pow(c, 2) + pow(L, 2) + 2 * (-x * b + y * c) - pow(l, 2);

  // after half angle substitutuion, the equation can be solved
  // in its quadratic form

  //t11 = -F1 + sqrt(E1^2 + F1^2 - G1^2)/(G1 - E1);
  float t12 = (-F1 - pow((pow(E1, 2) + pow(F1, 2) - pow(G1, 2)), 0.5)) / (G1 - E1);

  //t21 = -F2 + sqrt(E2^2 + F2^2 - G2^2)/(G2 - E2);
  float t22 = (-F2 - pow((pow(E2, 2) + pow(F2, 2) - pow(G2, 2)), 0.5)) / (G2 - E2);

  //t31 = -F3 + sqrt(E3^2 + F3^2 - G3^2)/(G3 - E3);
  float t32 = (-F3 - pow((pow(E3, 2) + pow(F3, 2) - pow(G3, 2)), 0.5)) / (G3 - E3);

  // int th11 = 2*round(atan(t11)*180/3.14159265);
  th12 = 2 * round(atan(t12) * 180 / 3.14159265);

  //int th21 = 2*round(atan(t21)*180/3.14159265);
  th22 = 2 * round(atan(t22) * 180 / 3.14159265);

  //int th31 = 2*round(atan(t31)*180/3.14159265);
  th32 = 2 * round(atan(t32) * 180 / 3.14159265);

}

void go(int px, int py, int pz)
{
  fk(px, py, pz);

  spd = 300;
  acc = 500;

  xaxis.setMaxSpeed(spd);
  xaxis.setAcceleration(acc);
  yaxis.setMaxSpeed(spd);
  yaxis.setAcceleration(acc);
  zaxis.setMaxSpeed(spd);
  zaxis.setAcceleration(acc);

  move((th12 + cur_x)*steps_per_degree, (th22 + cur_y)*steps_per_degree, (th32 + cur_z)*steps_per_degree, 0);
  cur_x = th12;
  cur_y = th22;
  cur_z = th32;

}



void loop()
{

  if (flag == true)
  {

    // z not less than 470 (negative)
    go(0, 100, -420);
    flag = false;
  }

  go(100, 0, -400);
  go(0, 100, -400);
  go(-100, 0, -400);
  go(0, -100, -400);
  go(100, 0, -400);


}
