#include <SimpleFOC.h>

// 无刷直流电机初始化
BLDCMotor motor = BLDCMotor(7); //2204电机
// 定义无刷直流驱动器
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
// 电机编码器初始化
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C pendulum = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

float adjust_ang= 60;
float adjust_p_vel= 3;
float adjust_motor_vel= 0.18;
float adjust_total_ang= 0;   //微调角度
// 通讯命令预设
Commander command = Commander(Serial);
void doTarget_motor_vel(char* cmd) { Serial.print("Change LQR motor_v");command.scalar(&adjust_motor_vel, cmd); }
void doTarget_p_vel(char* cmd) { Serial.print("Change LQR P_v");command.scalar(&adjust_p_vel, cmd); }
void doTarget_ang_vel(char* cmd) { Serial.print("Change LQR ang");command.scalar(&adjust_ang, cmd); }
void doTarget_total_ang(char* cmd) { Serial.print("Change total ang");command.scalar(&adjust_total_ang, cmd); }

void setup() {

  // 初始化电机编码器硬件
  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor0.init(&I2Cone);
  pendulum.init(&I2Ctwo);
  
  // 设置要使用的控制回路类型
  motor.controller = MotionControlType::torque;

  // 将电机连接到编码器
  motor.linkSensor(&sensor0);
  
  // driver
  driver.voltage_power_supply =16.8;
  driver.voltage_limit = 16.8; 
  driver.init();
  // 把电机连接到驱动器上
  motor.linkDriver(&driver);
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  // 初始化运动
  motor.init();
  // 校准编码器并启动FOC
  motor.initFOC();
  command.add('M', doTarget_motor_vel, "adjust_motor_vel");
  command.add('P', doTarget_p_vel, "adjust_p_vel");
  command.add('A', doTarget_ang_vel, "adjust_ang");
  command.add('T',doTarget_total_ang, "adjust_total_ang");
  
}

// 循环下采样计数器
long loop_count = 0;
float target_voltage;

void loop() {
  // ~1ms 
  motor.loopFOC();
  command.run();
   // 控制回路每次~25ms
  if(loop_count++ > 25){
    // 倒立摆传感器读取
    pendulum.update();
    sensor0.update();
    // 计算摆角
    float pendulum_angle = constrainAngle(pendulum.getAngle()-3.83-adjust_total_ang + M_PI);
    Serial.println(pendulum.getAngle());
    //Serial.println(sensor0.getAngle());
    if( abs(pendulum_angle) < 0.5){ // 如果角度足够小稳定
      target_voltage = controllerLQR(pendulum_angle, pendulum.getVelocity(), motor.shaftVelocity());}
    else{ // 倒立摆
      //Serial.println("swing-up");
       // 设置100%的最大电压到电机，以便摆动
      //target_voltage = -_sign(pendulum.getVelocity())*motor.voltage_limit*1;
      target_voltage = -_sign(pendulum.getVelocity())*16.8;
      //Serial.println(target_voltage);
      //target_voltage=0;
    }
     // 将目标电压设置到电机上
    motor.move(target_voltage);
    // 重置计数器
    loop_count=0;
  }
   

}

// 函数限制-和之间的夹角，以-180度和180度表示
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR稳定控制器功能
// 计算需要设置电机的电压，以稳定摆
float controllerLQR(float p_angle, float p_vel, float m_vel){
  // 如果角度可控
  // 计算控制律 
  // LQR controller u = k*x
  //  - k = [40, 7, 0.3]
  //  - x = [摆角，摆速度，电机速度]' 
  float u =  adjust_ang*p_angle + adjust_p_vel*p_vel + adjust_motor_vel*m_vel;
  // 限制设定给电机的电压
  //if(abs(u) > motor.voltage_limit*1) u = _sign(u)*motor.voltage_limit*1;
  if(abs(u) > driver.voltage_power_supply*0.7) u = _sign(u)*driver.voltage_power_supply*0.7;
  return u;
}
