#include <SimpleFOC.h>

// Magnetic encoder AS5600
//Encoder encoder = Encoder(2,3,2048);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// channel A and B callbacks
//void doA(){encoder.handleA();}
//void doB(){encoder.handleB();}
// initialize encoder hardware
//encoder.init();
// hardware interrupt enable
//encoder.enableInterrupts(doA, doB);

// Gimbal BLDC motor 2208 90kv
BLDCMotor motor1 = BLDCMotor(11); // define motor
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 10, 11, 8); // define driver

// angle set point variable
float target_angle = 0;
// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }

void setup() {
  sensor.init(); // initialize encoder sensor

  // driver config
  // power supply voltage
  driver1.voltage_power_supply = 12; // 12 V default
  driver1.init();

  // specify angle control loop
  motor1.controller = MotionControlType::angle;

  // velocity PI controller parameters
  // default P=0.5 I=10
  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 20;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec ~0.3V per millisecond
  motor1.PID_velocity.output_ramp = 1000;

  // default voltage_power_supply
  motor1.voltage_limit = 6;

  // configure low pass filter time constant Tf
  // velocity low pass filtering
  // default 5ms - try different values to see what is best
  // the lower the less filtered
  motor1.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor1.P_angle.P = 20;
  // maximal velocity of the position control
  // default 20
  motor1.velocity_limit = 4;

  // link the motor to the encoder
  motor1.linkSensor(&sensor);
  // link the motor to the driver
  motor1.linkDriver(&driver1);

  motor1.init(); // initialize motor
  motor1.initFOC(); // align encoder and start FOC

  // add target command T
  command.add('T', onTarget, "target angle");
  // monitoring port
  Serial.begin(115200);
  Serial.println("Motor ready.");
  Serial.println("Set the target angle using serial terminal:");
  _delay(1000);
}

void loop() {
  motor1.loopFOC(); // iterative FOC function
  motor1.move(target_angle); // iterative function to set & calc angle/position loop
  // can be run at much lower frequency than LoopFOC function

  // user communication
  command.run();
}
