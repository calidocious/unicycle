// BLDC driver standalone example
#include <SimpleFOC.h>

// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

void setup() {

  // driver init
  if (driver.init()) Serial.println("success!");
  else{
    Serial.println("failed!");
    return;
  }
  
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 32000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  // Max DC voltage allowed - default voltage_power_supply
  // driver.voltage_limit = 12;

  // enable driver
  driver.enable();

  Serial.begin(115200);

  _delay(1000);
}

void loop() {
  // setting pwm
  // phase A: 3V, phase B: 6V, phase C: 5V
  driver.setPwm(3,6,5);
}