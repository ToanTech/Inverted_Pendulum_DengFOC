#include <SimpleFOC.h>

// BLDC motor init
BLDCMotor motor = BLDCMotor(7);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
//Motor encoder init
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C pendulum = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

float adjust_ang= 60;
float adjust_p_vel= 3;
float adjust_motor_vel= 0.18;
float adjust_total_ang= 0;   //微调角度
// commander communication instance
Commander command = Commander(Serial);
void doTarget_motor_vel(char* cmd) { Serial.print("Change LQR motor_v");command.scalar(&adjust_motor_vel, cmd); }
void doTarget_p_vel(char* cmd) { Serial.print("Change LQR P_v");command.scalar(&adjust_p_vel, cmd); }
void doTarget_ang_vel(char* cmd) { Serial.print("Change LQR ang");command.scalar(&adjust_ang, cmd); }
void doTarget_total_ang(char* cmd) { Serial.print("Change total ang");command.scalar(&adjust_total_ang, cmd); }

void setup() {

  // initialise motor encoder hardware
  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor0.init(&I2Cone);
  pendulum.init(&I2Ctwo);
  
  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // link the motor to the encoder
  motor.linkSensor(&sensor0);
  
  // driver
  driver.voltage_power_supply =16.8;
  driver.voltage_limit = 16.8; 
  driver.init();
  // link the driver and the motor
  motor.linkDriver(&driver);


  Serial.begin(115200);
  motor.useMonitoring(Serial);
  // initialize motor
  
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
  command.add('M', doTarget_motor_vel, "adjust_motor_vel");
  command.add('P', doTarget_p_vel, "adjust_p_vel");
  command.add('A', doTarget_ang_vel, "adjust_ang");
  command.add('T',doTarget_total_ang, "adjust_total_ang");
  
}

// loop downsampling counter
long loop_count = 0;
float target_voltage;

void loop() {
  // ~1ms 
  motor.loopFOC();
  command.run();
  //motor.monitor();
  // control loop each ~25ms
  if(loop_count++ > 25){
    // updating the pendulum angle sensor
    // NECESSARY for library versions > v2.2 
    pendulum.update();
    sensor0.update();
    // calculate the pendulum angle 
    float pendulum_angle = constrainAngle(pendulum.getAngle()-3.83-adjust_total_ang + M_PI);
    Serial.println(pendulum.getAngle());
    //Serial.println(sensor0.getAngle());
    if( abs(pendulum_angle) < 0.5){ // if angle small enough stabilize
      target_voltage = controllerLQR(pendulum_angle, pendulum.getVelocity(), motor.shaftVelocity());}
    else{ // else do swing-up
      //Serial.println("swing-up");
      // sets 40% of the maximal voltage to the motor in order to swing up
      //target_voltage = -_sign(pendulum.getVelocity())*motor.voltage_limit*1;
      target_voltage = -_sign(pendulum.getVelocity())*16.8;
      //Serial.println(target_voltage);
      //target_voltage=0;
    }
    // set the target voltage to the motor
    motor.move(target_voltage);
    // restart the counter
    loop_count=0;
  }
   

}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel){
  // if angle controllable
  // calculate the control law 
  // LQR controller u = k*x
  //  - k = [40, 7, 0.3]
  //  - x = [pendulum angle, pendulum velocity, motor velocity]' 
  float u =  adjust_ang*p_angle + adjust_p_vel*p_vel + adjust_motor_vel*m_vel;
  // limit the voltage set to the motor
  //if(abs(u) > motor.voltage_limit*1) u = _sign(u)*motor.voltage_limit*1;
  if(abs(u) > driver.voltage_power_supply*0.7) u = _sign(u)*driver.voltage_power_supply*0.7;
  return u;
}
