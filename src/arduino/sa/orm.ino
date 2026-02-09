#include <EEPROM.h>
#include <PID_v1.h>


#include "orm.h"
/*
AccelStepper j0(1,X_STEP_PIN,X_DIR_PIN);
AccelStepper j1(1,Y_STEP_PIN,Y_DIR_PIN);
AccelStepper j2(1,Z_STEP_PIN,Z_DIR_PIN);
AccelStepper j3(1,E_STEP_PIN,E_DIR_PIN);
AccelStepper j4(1,Q_STEP_PIN,Q_DIR_PIN);
AccelStepper j5(1,W_STEP_PIN,W_DIR_PIN);
*/
/*
Servo js0;  // Joint Servo 0 
Servo js1;  // Joint Servo 1
Servo js2;  // Joint Servo 2
Servo js3;  // Joint Servo 3
Servo js4;  // Joint Servo 4
Servo js5;  // Joint Servo 5
*/


#define MIN_PWM 140 // Good 110, last 78
#define MAX_PWM 255

double input_a_zero = 0;  // Current Position of Actuator 0
double output_a_zero = 0; // Desired Effort to Apply -255 .. 255
double setpoint_a_zero =0; // Desired Position of Actuator 0

double input_a_zero_speed = 0;  // Current Position of Actuator 0
double output_a_zero_speed = 0; // Desired Effort to Apply -255 .. 255
double setpoint_a_zero_speed =0; // Desired Position of Actuator 0

double input_a_zero_accel = 0;  // Current Position of Actuator 0
double output_a_zero_accel = 0; // Desired Effort to Apply -255 .. 255
double setpoint_a_zero_accel=0; // Desired Position of Actuator 05

PID PID_A_ZERO(&input_a_zero, &output_a_zero, &setpoint_a_zero, 0.05,  0.0045, 0.0045, DIRECT);

// 0.1, 0.50 , 0.001
// 0.10,  0.85, 0.0015 - good for 0.5 kg load

PID PID_A_ZERO_SPEED(&input_a_zero_speed, &output_a_zero_speed, &setpoint_a_zero_speed, 0.10,  0.85, 0.0015/*0.01*/, DIRECT);
PID PID_A_ZERO_ACCEL(&input_a_zero_accel, &output_a_zero_accel, &setpoint_a_zero_accel, 0.055,  0.0040, 0.00025/*0.01*/, DIRECT);

const double DEFAULT_PID_KP = 0.05;
const double DEFAULT_PID_KI = 0.0045;
const double DEFAULT_PID_KD = 0.0045;

//Servo gripperServo;

const int ADC_MAX = 1023;

const char osp_command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};

//const int ORM_J_ENCODER_INPUT[JOINTS_COUNT] = {X_ENCODER_IN,Y_ENCODER_IN, Z_ENCODER_IN, E_ENCODER_IN, Q_ENCODER_IN, W_ENCODER_IN} ;/*A1,A2,A3,A4,A5};*/
//const int ORM_J_ENABLE_PIN[JOINTS_COUNT] =    {X_ENABLE_PIN,Y_ENABLE_PIN, Z_ENABLE_PIN, E_ENABLE_PIN, Q_ENABLE_PIN, W_ENABLE_PIN} ;/*7,10,13,16,19};*/
/*
const short orm_j_angle_min = 0;
const short orm_j_angle_max = 1023;
*/

const int ORM_J_ADC_PINS[ORA_JOINTS_COUNT] = {A0};

const int EEPROM_ANGLE_CORRECTION_OFFSET = 0;
const int EEPROM_ANGLE_WIDTH_OFFSET = EEPROM_ANGLE_CORRECTION_OFFSET + ORA_JOINTS_COUNT * sizeof(short);
const int EEPROM_DEFAULT_ANGLE_OFFSET = EEPROM_ANGLE_WIDTH_OFFSET + ORA_JOINTS_COUNT * sizeof(short);
const int EEPROM_SERVO_ZERO_ANGLE_ADC = EEPROM_DEFAULT_ANGLE_OFFSET + ORA_JOINTS_COUNT * sizeof(short);
const int EEPROM_SERVO_MAX_ANGLE_ADC = EEPROM_SERVO_ZERO_ANGLE_ADC + ORA_JOINTS_COUNT * sizeof(short);
const unsigned short PID_EEPROM_MAGIC = 0xA5A5;
const int EEPROM_PID_MAGIC_OFFSET = EEPROM_SERVO_MAX_ANGLE_ADC + ORA_JOINTS_COUNT * sizeof(short);
const int EEPROM_PID_KP_OFFSET = EEPROM_PID_MAGIC_OFFSET + sizeof(unsigned short);
const int EEPROM_PID_KI_OFFSET = EEPROM_PID_KP_OFFSET + sizeof(double);
const int EEPROM_PID_KD_OFFSET = EEPROM_PID_KI_OFFSET + sizeof(double);
const unsigned short CALIB_EEPROM_MAGIC = 0xC3C3;
const int EEPROM_CALIB_MAGIC_OFFSET = EEPROM_PID_KD_OFFSET + sizeof(double);

const int PID_MODE_SPEED = 1;
const int PID_MODE_ANGLE = 2;

int PID_MODE = PID_MODE_SPEED;
const int CONTROL_MODE_PID = 0;
const int CONTROL_MODE_FORCE_PWM = 1;

void ORM::ospHandleGenericCommand(){
  // TODO
}

void ORM::cmdSetAngle(){
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  char force = osp_input_buffer[OSP_ORM_ANGLE_FORCE_INDEX];
  j_angle_desired[ORA_INDEX] = angle;
  j_angle_force[ORA_INDEX] = force;
  control_mode = CONTROL_MODE_PID;
}

void ORM::cmdSetCorrAngle(){
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  j_angle_correction[ORA_INDEX] = angle;
  saveCalibrationToEeprom(ORA_INDEX);

}

void ORM::cmdSetAngleWidth(){
  int actuator_no = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  j_angle_width[ORA_INDEX] = angle;
  saveCalibrationToEeprom(ORA_INDEX);
}

void ORM::cmdSetForcePwm(){
  int pwm = (short)(((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX]);
  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }
  control_pwm = pwm;
  control_mode = CONTROL_MODE_FORCE_PWM;
}

void ORM::cmdSetPidProportional(){
  int newKp = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  double Kp = (double)newKp / 10000;
  setPidProportional(Kp);
}

void ORM::cmdSetPidIntegral(){
  int newKi = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  double Ki = (double)newKi / 10000;
  setPidIntegral(Ki);
}

void ORM::cmdSetPidDifferential(){
  int newKd = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  double Kd = (double)newKd / 10000;
  setPidDifferential(Kd);
}

void ORM::setPidDifferential(double value) {
  Kd = value;
  _updateMotorDCTunings();
  savePidToEeprom();
}

double ORM::getPidDifferential() {
  return Kd;
}

void ORM::setPidIntegral(double value) {
  Ki = value;
  _updateMotorDCTunings();
  savePidToEeprom();
}

double ORM::getPidIntegral() {
  return Ki;
}

void ORM::setPidProportional(double value) {
  Kp = value;
  _updateMotorDCTunings();
  savePidToEeprom();
}

double ORM::getPidProportional() {
  return Kp;
}

void ORM::_updateMotorDCTunings() {
  PID_A_ZERO.SetTunings(Kp, Ki, Kd);
  PID_A_ZERO_SPEED.SetTunings(Kp, Ki, Kd);
}

void ORM::savePidToEeprom() {
  EEPROM.put(EEPROM_PID_MAGIC_OFFSET, (unsigned short)PID_EEPROM_MAGIC);
  EEPROM.put(EEPROM_PID_KP_OFFSET, Kp);
  EEPROM.put(EEPROM_PID_KI_OFFSET, Ki);
  EEPROM.put(EEPROM_PID_KD_OFFSET, Kd);
}

void ORM::loadPidFromEeprom() {
  unsigned short magic = 0;
  EEPROM.get(EEPROM_PID_MAGIC_OFFSET, magic);
  if (magic != PID_EEPROM_MAGIC) {
    return;
  }
  EEPROM.get(EEPROM_PID_KP_OFFSET, Kp);
  EEPROM.get(EEPROM_PID_KI_OFFSET, Ki);
  EEPROM.get(EEPROM_PID_KD_OFFSET, Kd);
  _updateMotorDCTunings();
}

void ORM::saveCalibrationToEeprom(int jointNo) {
  EEPROM.put(EEPROM_ANGLE_CORRECTION_OFFSET + jointNo * sizeof(short), j_angle_correction[jointNo]);
  EEPROM.put(EEPROM_ANGLE_WIDTH_OFFSET + jointNo * sizeof(short), j_angle_width[jointNo]);
  EEPROM.put(EEPROM_DEFAULT_ANGLE_OFFSET + jointNo * sizeof(short), j_angle_desired[jointNo]);
  EEPROM.put(EEPROM_SERVO_ZERO_ANGLE_ADC + jointNo * sizeof(short), servo_zero_angle[jointNo]);
  EEPROM.put(EEPROM_SERVO_MAX_ANGLE_ADC + jointNo * sizeof(short), servo_max_angle[jointNo]);
  EEPROM.put(EEPROM_CALIB_MAGIC_OFFSET, (unsigned short)CALIB_EEPROM_MAGIC);
}

void ORM::ospHandleORACommand(){
  int address = osp_input_buffer[OSP_ORA_ADDRESS_INDEX];

  if (address == current_address){
    int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  

    if (cmd == OSP_ORA_CMD_SET_ANGLE) {
      cmdSetAngle();
    }
    if (cmd == OSP_ORA_CMD_SET_CORR_ANGLE) {
      cmdSetCorrAngle();
    }
    if (cmd == OSP_ORA_CMD_SET_ANGLE_WIDTH) {
      cmdSetAngleWidth();
    }
    if (cmd == OSP_ORA_CMD_SET_PID_PROPORTIONAL){
      cmdSetPidProportional();
    }
    if (cmd == OSP_ORA_CMD_SET_PID_INTEGRAL){
      cmdSetPidIntegral();
    }
    if (cmd == OSP_ORA_CMD_SET_PID_DIFFERENTIAL){
      cmdSetPidDifferential();
    }
    if (cmd == OSP_ORA_CMD_SET_FORCE_PWM){
      cmdSetForcePwm();
    }
  } else {
    // Re-transmit command if not addressed to this device
    Serial.write(osp_input_buffer, OSP_COMMAND_LENGTH);
  }
}

void ORM::ospHandleCommand(){
  int dev = osp_input_buffer[OSP_MSG_DEV_INDEX];
  
  if (dev == OSP_DEV_GENERIC) {
    ospHandleGenericCommand();
  } else if (dev == OSP_DEV_CURRENT) {
    if (dev == OSP_DEV_ORA) {
      ospHandleORACommand();
    }
  }
}

void ORM::ospPrepareOutputBuffer(){
  for(int i=0;i<OSP_BUFFER_SIZE;i++) {
    osp_output_buffer[i] = osp_command_template[i];
  }
}

void ORM::oraInfoCurrentAngle(){
  ospPrepareOutputBuffer();
    
  unsigned int angle = j_angle_read[ORA_INDEX]; 

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_ANGLE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = current_address;
  osp_output_buffer[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xFF;
  osp_output_buffer[OSP_ORM_ANGLE_MSB_INDEX] = angle >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::oraInfoCurrentSpeed(){
  ospPrepareOutputBuffer();  
  short int speed = j_speed_current[ORA_INDEX];// j_speed_current[actuatorNo];

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_SPEED;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = current_address;
  osp_output_buffer[OSP_ORM_SPEED_LSB_INDEX] = speed & 0xFF;
  osp_output_buffer[OSP_ORM_SPEED_MSB_INDEX] = speed >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::sendUpdateInfo(){
  unsigned long current_millis = millis();
  if (current_millis - last_millis > UPDATE_INTERVAL){
    // Stepper Joints Update
    oraInfoCurrentAngle();
    oraInfoCurrentSpeed();
    last_millis = current_millis;
  }
}


short ORM::readAngle(int actuatorNo){
  /*
  // Gamme Angle Obtained from ADC. Casted to Int Angle Range
  long gamma = ((long)analogRead(ORM_J_ENCODER_INPUT[actuatorNo])) * (long)orm_max_int_angle / (long)orm_j_encoder_adc_range;
  // Applying Correction
  gamma += j_angle_correction[actuatorNo];
  // Keep the value in the range of 0 to MAX INT ANGLE
  gamma %= orm_max_int_angle;

  // Keeping track of rotation across of ZERO
  long gamma_plus = gamma + orm_max_int_angle;
  long gamma_minus = gamma - orm_max_int_angle;

  long diff_gamma = abs(gamma - j_angle_read[actuatorNo]);
  long diff_gamma_plus = abs(gamma_plus - j_angle_read[actuatorNo]);
  long diff_gamma_minus = abs(gamma_minus- j_angle_read[actuatorNo]);

  long gamma_res=0; 
  if (diff_gamma<=diff_gamma_plus){
    if(diff_gamma<=diff_gamma_minus){
      gamma_res = gamma;      
    } else {
      gamma_res = gamma_minus;
    }
  } else {
    if(diff_gamma_plus <= diff_gamma_minus){
      gamma_res = gamma_plus;
    } else {
      gamma_res = gamma_minus;
    }
  }
  // Applying statistical filtering
  // 1. Save the current value

  j_angle_sample[actuatorNo * STAT_SAMPLE_SIZE +read_samples_ptr[actuatorNo]] = gamma_res;
  read_samples_ptr[actuatorNo] ++;
  read_samples_ptr[actuatorNo] %= STAT_SAMPLE_SIZE;
  if (read_samples_size[actuatorNo]<STAT_SAMPLE_SIZE){
    read_samples_size[actuatorNo] ++;
  }

  // 2. Take an average of the available measurements
  long int gamma_sum = 0;
  for (int i=0; i<read_samples_size[actuatorNo];i++){
    gamma_sum += j_angle_sample[actuatorNo * STAT_SAMPLE_SIZE + i];
  }
  gamma_res = gamma_sum / read_samples_size[actuatorNo];
  
  return gamma_res;
  */
  return 0;
}

int isqrt(int x) {
    if (x <= 0)
        return 0;

    int r = x;
    int y = (r + x / r) / 2;

    while (y < r) {
        r = y;
        y = (r + x / r) / 2;
    }

    return r;
}

void ORM::updateActuatorsPosition(){  
  // UPDATE THE SPEED FIRST
  unsigned long current_millis = millis();
  
  if(current_millis-speed_millis>ORM_SPEED_UPDATE_INTERVAL_MS){
    speed_millis = current_millis;

    // Speed Control Routine
    for(int i=0;i<ORA_JOINTS_COUNT;i++){
      // Recalculate the read speed
      if(j_angle_read_prev[i] == 0) {
        // Assuming the previous angle value was not initialised
        j_angle_read_prev[i] = j_angle_read[i];
      }
      long speed_angle_diff = j_angle_read[i] - j_angle_read_prev[i];
      j_angle_read_prev[i] = j_angle_read[i];

      j_speed_read[i] = speed_angle_diff;

      long accel_speed_diff = j_speed_read[i] - j_speed_read_prev[i];
      j_accel_read[i] = accel_speed_diff;

      if (motor_power == 0){
        // If no motor power - just apply the same angle that is currenrly read 
        j_angle_current[i] = j_angle_read[i];
        j_angle_desired[i] = j_angle_read[i];
        j_speed_current[i] = 0;
        continue;
      }

      if(j_angle_force[i]){
        // If the angle is forced - apply it immediately, no acceleration logic
        j_angle_current[i] = j_angle_desired[i];
        j_speed_current[i] = 0;
        continue;        
      }
      // Calculate the desired speed, considering the acceleration, deacceleration and the current and desired angle positions 
      long angle_diff = j_angle_desired[i] - j_angle_read[i];
      long direction = sgn(angle_diff);

      long accelerate_speed =(long)j_speed_current[i] + direction*(long)orm_j_acceleration[i]*(long)ORM_SPEED_UPDATE_INTERVAL_MS / (long)ORM_MS_IN_SECOND;

      long abs_angle_diff = angle_diff * direction;

      if (control_mode == CONTROL_MODE_FORCE_PWM) {
        if (control_pwm > 0) {
          analogWrite(A_ZERO_FWD_PIN, control_pwm);
          analogWrite(A_ZERO_BCK_PIN, 0);
        } else {
          analogWrite(A_ZERO_FWD_PIN, 0);
          analogWrite(A_ZERO_BCK_PIN, -control_pwm);
        }
        continue;
      }

      // Apply PID controller to control the absolute position
      // Update for PID-controlled joints
      input_a_zero = (double)j_angle_read[0];
      setpoint_a_zero = (double)j_angle_desired[0];

      PID_A_ZERO.Compute();

      int a_zero_effort = (int)output_a_zero;

      a_zero_effort = int(sgn(a_zero_effort)*((float)MIN_PWM + (float)abs(a_zero_effort)*((float)MAX_PWM - (float)MIN_PWM)/(float)MAX_PWM));
/*
      abs_angle_diff-=300;
      if(abs_angle_diff<0){
        abs_angle_diff = 0;
      }
*/
      long sqrt_angle_diff = isqrt(abs_angle_diff);
      long accel_sqrt = (long)isqrt(2*orm_j_acceleration[i]);

      float deaccelerate_speed = (long)direction*(long)sqrt_angle_diff * (long)accel_sqrt * (long)ORM_SPEED_UPDATE_INTERVAL_MS / (long)ORM_MS_IN_SECOND; 
      
      // correction of deacceleration speed 

      // deaccelerate_speed -= ORM_SPEED_UPDATE_INTERVAL_MS * orm_j_acceleration[i] / ORM_MS_IN_SECOND;

      long max_speed = direction * orm_j_speed_max[i];

      long result_speed = min(min(direction*max_speed,direction*accelerate_speed),direction*deaccelerate_speed);

      j_speed_current[i] = direction*result_speed;

      // Applying PID-controller to contorl the speed
      input_a_zero_speed = j_speed_read[i];
      setpoint_a_zero_speed = j_speed_current[i];


      long accel_desired = j_speed_current[i] - j_speed_read[i];

      input_a_zero_accel = j_accel_read[i];
      setpoint_a_zero_accel = accel_desired;

      PID_A_ZERO_ACCEL.Compute();

      PID_A_ZERO_SPEED.Compute();

      int a_zero_speed_effort = output_a_zero_speed;
      a_zero_speed_effort = sgn(a_zero_speed_effort)*(MIN_PWM + abs(a_zero_speed_effort)*(MAX_PWM - MIN_PWM)/MAX_PWM);

      int a_zero_accel_effort = output_a_zero_accel;
      a_zero_accel_effort = sgn(a_zero_accel_effort)*(MIN_PWM + abs(a_zero_accel_effort)*(MAX_PWM - MIN_PWM)/MAX_PWM);



 //}
 /*
      if(abs_angle_diff<0){
        if (a_zero_effort > 0) {
          analogWrite(A_ZERO_PWM_PIN, a_zero_effort);
          digitalWrite(A_ZERO_FWD_PIN, 0);
          digitalWrite(A_ZERO_BCK_PIN, 1);
        } else {
          analogWrite(A_ZERO_PWM_PIN, -a_zero_effort);
          digitalWrite(A_ZERO_FWD_PIN, 1);
          digitalWrite(A_ZERO_BCK_PIN, 0);
        }
      } else {
        if (a_zero_speed_effort > 0) {
          analogWrite(A_ZERO_PWM_PIN, a_zero_speed_effort);
          digitalWrite(A_ZERO_FWD_PIN, 0);
          digitalWrite(A_ZERO_BCK_PIN, 1);
        } else {
          analogWrite(A_ZERO_PWM_PIN, -a_zero_speed_effort);
          digitalWrite(A_ZERO_FWD_PIN, 1);
          digitalWrite(A_ZERO_BCK_PIN, 0);
        }
      }
*/
      // Apply PWM from the Speed PID regulator
      if (a_zero_speed_effort > 0) {
        analogWrite(A_ZERO_FWD_PIN, a_zero_speed_effort);
        analogWrite(A_ZERO_BCK_PIN, 0);
      } else {
        analogWrite(A_ZERO_FWD_PIN, 0);
        analogWrite(A_ZERO_BCK_PIN, -a_zero_speed_effort);
      }
      
      /*
      //if(j_speed_read[i]*accel_desired>=0 ) {
        if (a_zero_accel_effort > 0) {
          analogWrite(A_ZERO_FWD_PIN, a_zero_accel_effort);
          analogWrite(A_ZERO_BCK_PIN, 0);
          //digitalWrite(A_ZERO_FWD_PIN, 0);
          //digitalWrite(A_ZERO_BCK_PIN, 1);
        } else {
          analogWrite(A_ZERO_FWD_PIN, 0);
          analogWrite(A_ZERO_BCK_PIN, -a_zero_accel_effort);
          //analogWrite(A_ZERO_PWM_PIN, -a_zero_accel_effort);
          //digitalWrite(A_ZERO_FWD_PIN, 1);
          //digitalWrite(A_ZERO_BCK_PIN, 0);
        }
      //} else {
        // Apply Break
      //  analogWrite(A_ZERO_FWD_PIN, 0);
      //  analogWrite(A_ZERO_BCK_PIN, 0);
      //}
      */
      // Do not limit to the desired angle only. Allow to pass over the desired angle if deacceleration is not possible
      float new_angle = (long)j_angle_current[i] + (long)j_speed_current[i]*(long)ORM_SPEED_UPDATE_INTERVAL_MS / (long)ORM_MS_IN_SECOND ;
      long new_angle_diff = j_angle_desired[i] - new_angle;

      // Do not allow to set too much difference in the comarison to the read value 
      if (abs(j_angle_read[i]-new_angle)>1500){
        continue;
      }
      //if(new_angle_diff * angle_diff < 0){
        //j_angle_current[i] = j_angle_desired[i];
      //} else {
          // TODO here the condition check must be introduced to make sure that the angle lays within the allowed range
      if ( new_angle < 0 - j_angle_correction[i]) {
        // If the angle is less then minimal allowed angle - set it to minimal value:
        j_angle_current[i] = - j_angle_correction[i];
      }else if (new_angle > j_angle_width[i] - j_angle_correction[i] ){
        // If the angloe is above the maximal allowed angle - set it to maximal value
        j_angle_current[i] = j_angle_width[i] - j_angle_correction[i];

      } else {
        j_angle_current[i] = new_angle;  // Update the angle
      }
     
    }
  }
}


void ORM::updateSensorsMeasurements(){
  for (int i=0;i<ORA_JOINTS_COUNT;i++){
    int adc_read = analogRead(ORM_J_ADC_PINS[i]);
    int angle_int = orm_max_int_angle * adc_read  / ADC_MAX;
    j_angle_read_samples[i][j_angle_samples_ptr] = angle_int;
  }
  j_angle_samples_ptr++;
  j_angle_samples_ptr %= ADC_SAMPLES_N;
  if(j_angle_samples_count<ADC_SAMPLES_N){
    j_angle_samples_count++;
  }
  for(int i=0;i<ORA_JOINTS_COUNT;i++){
    long int sum = 0;
    for (int j=0;j<j_angle_samples_count;j++){
      sum += j_angle_read_samples[i][j];
    }
    int filtered_angle = sum / j_angle_samples_count;
    j_angle_filtered[i] = filtered_angle;
    
    filtered_angle = (float) j_angle_width[i] * (float)(filtered_angle - servo_zero_angle[i]) / (float)(servo_max_angle[i]-servo_zero_angle[i]);

    filtered_angle = filtered_angle-j_angle_correction[i];
    j_angle_read[i] = filtered_angle;
    //j_angle_desired[i] = j_angle_read[i];
  }

}

void ORM::ospSerialLoop(){

  while(Serial.available()){
    char b = Serial.read();
    if(osp_command_template[osp_ptr]==b || osp_command_template[osp_ptr]==0){
      osp_input_buffer[osp_ptr] = b;
      osp_ptr++; 
      if(osp_ptr==OSP_COMMAND_LENGTH){
        ospHandleCommand();
        osp_ptr = 0;
      }
    } else {
      osp_ptr = 0;
    }
    
  }
  updateSensorsMeasurements();
  updateActuatorsPosition();
  sendUpdateInfo();
  /*

  for (int i=0;i<JOINTS_COUNT;i++){
   joints[i]->run();
  }
  */
}

void ORM::setup(){
   // Timer2: fast PWM, prescaler = 1 -> ~31.37 kHz
  //TCCR2B = (TCCR2B & 0b11111000) | 0x05;
  // Timer1: fast PWM, prescaler = 1 -> ~31.37 kHz
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  // prescaler 1024
  Serial.begin(115200);
  Serial.setTimeout(0.01);

  //analogReference(EXTERNAL);

  // Initing the current address
  current_address = 0;
  control_mode = CONTROL_MODE_PID;
  control_pwm = 0;

  // Enable pull-up resistors for address pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  int addr_0 = !digitalRead(2);
  int addr_1 = !digitalRead(3);
  int addr_2 = !digitalRead(4);
  int addr_3 = !digitalRead(5);
  
  current_address = addr_3 << 3 | addr_2 << 2 | addr_1 << 1 | addr_0;

  last_millis = millis();
  speed_millis = millis();

  
  //if (motor_power!=0){
  pinMode(MOTOR_POWER_PIN, OUTPUT);
  digitalWrite(MOTOR_POWER_PIN,LOW);
  //} else {
  //  pinMode(MOTOR_POWER_PIN, OUTPUT);
  //  digitalWrite(MOTOR_POWER_PIN,HIGH);
  //}

// By default - no torque to be applied to PWM
  pinMode(A_ZERO_PWM_PIN, OUTPUT);
  analogWrite(A_ZERO_PWM_PIN, 0);
  pinMode(A_ZERO_FWD_PIN, OUTPUT);
  analogWrite(A_ZERO_FWD_PIN, 0);
  pinMode(A_ZERO_BCK_PIN, OUTPUT);
  analogWrite(A_ZERO_BCK_PIN,0);
  
  /*
  joints[0] = &j0;
  joints[1] = &j1;
  joints[2] = &j2;
  joints[3] = &j3;
  joints[4] = &j4;
  joints[5] = &j5;  
*/
  PID_A_ZERO.SetMode(AUTOMATIC);
  PID_A_ZERO.SetOutputLimits(-255, 255);

  PID_A_ZERO_SPEED.SetMode(AUTOMATIC);
  PID_A_ZERO_SPEED.SetOutputLimits(-255, 255);

  PID_A_ZERO_ACCEL.SetMode(AUTOMATIC);
  PID_A_ZERO_ACCEL.SetOutputLimits(-255, 255);

  Kp = DEFAULT_PID_KP;
  Ki = DEFAULT_PID_KI;
  Kd = DEFAULT_PID_KD;
  loadPidFromEeprom();

  // Read the current value of the angles
  updateSensorsMeasurements();

  unsigned short calib_magic = 0;
  EEPROM.get(EEPROM_CALIB_MAGIC_OFFSET, calib_magic);
  bool calib_valid = (calib_magic == CALIB_EEPROM_MAGIC);

  // Set the desired value of the angle
/*
  joint_servos[0] = &js0;
  joint_servos[1] = &js1;
  joint_servos[2] = &js2;
  joint_servos[3] = &js3;
  joint_servos[4] = &js4;
  joint_servos[5] = &js5;

  gripper_servo = &gripperServo;
  gripper_servo->attach(8);
  gripper_servo->write(0);
*/
  // Initing Joints Servos 
  /*
  for (int i=0;i<JOINTS_COUNT;i++){
    joint_servos[i]->attach(ORM_J_SERVO_PINS[i],500,2500);
    joint_servos[i]->write(0);    
  }
*/
  for (int i=0;i<ORA_JOINTS_COUNT;i++) {
    /*
   // INIT JOINT
   joints[i]->setEnablePin(ORM_J_ENABLE_PIN[i]);
   joints[i]->setPinsInverted(false,false,true);
   //joints[i]->disableOutputs();
   joints[i]->setAcceleration(200000); // 200 steps per second per second
   joints[i]->enableOutputs();
   // Populate desired speed
   j_speed_desired[i] = (long)orm_max_int_angle * (long)500 /(long)orm_j_stepper_full_rot[i];
   // Set speed in steps/second
   joints[i]->setMaxSpeed((long)j_speed_desired[i] * (long)orm_j_stepper_full_rot[i] / (long)orm_max_int_angle);
   //joints[i]->setSpeed(100);*/
   // LOAD CORRECTION ANGLE
   if (calib_valid) {
     EEPROM.get(EEPROM_ANGLE_CORRECTION_OFFSET + i*sizeof(short), j_angle_correction[i]); 
   } else {
     j_angle_correction[i] = 0;
   }
   j_angle_current[i]  = 0;

   if (calib_valid) {
     EEPROM.get(EEPROM_ANGLE_WIDTH_OFFSET+i*sizeof(short),j_angle_width[i]); 
   } else {
     j_angle_width[i] = 16384;
   }
   if(j_angle_width[i]<=0) {
     j_angle_width[i] = 16384;
   }

   if (calib_valid) {
     EEPROM.get(EEPROM_DEFAULT_ANGLE_OFFSET+i*sizeof(short),j_angle_desired[i]); 
   } else {
     j_angle_desired[i] = 0;
   }
   
   unsigned short zero_adc = 0;
   unsigned short max_adc = 0;  

   if (calib_valid) {
     EEPROM.get(EEPROM_SERVO_ZERO_ANGLE_ADC+i*sizeof(short),zero_adc);
     EEPROM.get(EEPROM_SERVO_MAX_ANGLE_ADC+i*sizeof(short),max_adc);
   }

   if (!calib_valid || zero_adc == 0) {
    zero_adc = default_servo_zero_angle[i];
   } 
   servo_zero_angle[i] = zero_adc;
   if (!calib_valid || max_adc ==0 ) {
    max_adc = default_servo_max_angle[i];
   }
   servo_max_angle[i] = max_adc;

 }

  // For now setting the desired angle to the one that was read at the start 
  j_angle_desired[0] = j_angle_read[0];

}

ORM::ORM(){
  
}
