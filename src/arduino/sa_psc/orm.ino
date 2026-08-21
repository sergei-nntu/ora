#include <EEPROM.h>


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


const int MIN_PWM_DEFAULT = 120; // Good 110, last 78
const int MAX_PWM = 255;

//Servo gripperServo;

const int ADC_MAX = 1023;
const unsigned long SPEED_MEASUREMENT_INTERVAL = 50; // Good - 50

const char osp_command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};

//const int ORM_J_ENCODER_INPUT[JOINTS_COUNT] = {X_ENCODER_IN,Y_ENCODER_IN, Z_ENCODER_IN, E_ENCODER_IN, Q_ENCODER_IN, W_ENCODER_IN} ;/*A1,A2,A3,A4,A5};*/
//const int ORM_J_ENABLE_PIN[JOINTS_COUNT] =    {X_ENABLE_PIN,Y_ENABLE_PIN, Z_ENABLE_PIN, E_ENABLE_PIN, Q_ENABLE_PIN, W_ENABLE_PIN} ;/*7,10,13,16,19};*/
/*
const short orm_j_angle_min = 0;
const short orm_j_angle_max = 1023;
*/

const int ORM_J_ADC_PIN = A0;

const int EEPROM_ANGLE_CORRECTION_OFFSET = 0;
const int EEPROM_ANGLE_WIDTH_OFFSET = EEPROM_ANGLE_CORRECTION_OFFSET + sizeof(short);
const int EEPROM_DEFAULT_ANGLE_OFFSET = EEPROM_ANGLE_WIDTH_OFFSET + sizeof(short);
const int EEPROM_SERVO_ZERO_ANGLE_ADC = EEPROM_DEFAULT_ANGLE_OFFSET + sizeof(short);
const int EEPROM_SERVO_MAX_ANGLE_ADC = EEPROM_SERVO_ZERO_ANGLE_ADC + sizeof(short);
const unsigned short PID_EEPROM_MAGIC = 0xA5A5;
const int EEPROM_PID_MAGIC_OFFSET = EEPROM_SERVO_MAX_ANGLE_ADC + sizeof(short);
const int EEPROM_PID_KP_OFFSET = EEPROM_PID_MAGIC_OFFSET + sizeof(unsigned short);
const int EEPROM_PID_KI_OFFSET = EEPROM_PID_KP_OFFSET + sizeof(double);
const int EEPROM_PID_KD_OFFSET = EEPROM_PID_KI_OFFSET + sizeof(double);
const unsigned short CALIB_EEPROM_MAGIC = 0xC3C3;
const int EEPROM_CALIB_MAGIC_OFFSET = EEPROM_PID_KD_OFFSET + sizeof(double);
const int EEPROM_SPEED_MAX_OFFSET = EEPROM_CALIB_MAGIC_OFFSET + sizeof(unsigned short);
const int EEPROM_ACCELERATION_OFFSET = EEPROM_SPEED_MAX_OFFSET + sizeof(short);
const unsigned short MOTION_EEPROM_MAGIC = 0xC4C4;
const int EEPROM_MOTION_MAGIC_OFFSET = EEPROM_ACCELERATION_OFFSET + sizeof(short);
const int EEPROM_MIN_PWM_OFFSET = EEPROM_MOTION_MAGIC_OFFSET + sizeof(unsigned short);
const unsigned short PWM_EEPROM_MAGIC = 0xC5C5;
const int EEPROM_PWM_MAGIC_OFFSET = EEPROM_MIN_PWM_OFFSET + sizeof(short);
const int EEPROM_ADC_SAMPLES_N_OFFSET = EEPROM_PWM_MAGIC_OFFSET + sizeof(unsigned short);
const unsigned short ADC_SAMPLES_EEPROM_MAGIC = 0xC6C6;
const int EEPROM_ADC_SAMPLES_MAGIC_OFFSET = EEPROM_ADC_SAMPLES_N_OFFSET + sizeof(short);

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
  j_angle_desired = angle;
  j_angle_force = force;
  target_angle_stable_iterations = 0;
  control_mode = CONTROL_MODE_PID;
}

void ORM::cmdSetCorrAngle(){
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  j_angle_correction = angle;
  saveCalibrationToEeprom();

}

void ORM::cmdSetAngleWidth(){
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];
  j_angle_width = angle;
  saveCalibrationToEeprom();
}

void ORM::cmdSetMaxSpeed(){
  int speed = (short)(((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX]);
  if (speed < 0) {
    speed = 0;
  } else if (speed > 32767) {
    speed = 32767;
  }
  j_speed_max = speed;
  saveMotionLimitsToEeprom();
}

void ORM::cmdSetAcceleration(){
  int accel = (short)(((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX]);
  if (accel < 0) {
    accel = 0;
  } else if (accel > 32767) {
    accel = 32767;
  }
  j_acceleration = accel;
  saveMotionLimitsToEeprom();
}

void ORM::cmdSetMinPwm(){
  int pwm = (short)(((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX]);
  if (pwm < 0) {
    pwm = 0;
  } else if (pwm > MAX_PWM) {
    pwm = MAX_PWM;
  }
  min_pwm = pwm;
  saveMinPwmToEeprom();
}

void ORM::cmdSetAdcSamplesN(){
  int samples = (short)(((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX]);
  if (samples < 1) {
    samples = 1;
  } else if (samples > ADC_SAMPLES_N_MAX) {
    samples = ADC_SAMPLES_N_MAX;
  }
  adc_samples_n = samples;
  if (j_angle_samples_ptr >= adc_samples_n) {
    j_angle_samples_ptr = 0;
  }
  if (j_angle_samples_count > adc_samples_n) {
    j_angle_samples_count = adc_samples_n;
  }
  saveAdcSamplesToEeprom();
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
  //_updateMotorDCTunings();
  //savePidToEeprom();
}

double ORM::getPidDifferential() {
  return Kd;
}

void ORM::setPidIntegral(double value) {
  Ki = value;
  //_updateMotorDCTunings();
  //savePidToEeprom();
}

double ORM::getPidIntegral() {
  return Ki;
}

void ORM::setPidProportional(double value) {
  Kp = value;
  //_updateMotorDCTunings();
  //savePidToEeprom();
}

double ORM::getPidProportional() {
  return Kp;
}

void ORM::saveCalibrationToEeprom() {
  EEPROM.put(EEPROM_ANGLE_CORRECTION_OFFSET, j_angle_correction);
  EEPROM.put(EEPROM_ANGLE_WIDTH_OFFSET, j_angle_width);
  EEPROM.put(EEPROM_DEFAULT_ANGLE_OFFSET, j_angle_desired);
  EEPROM.put(EEPROM_SERVO_ZERO_ANGLE_ADC, servo_zero_angle);
  EEPROM.put(EEPROM_SERVO_MAX_ANGLE_ADC, servo_max_angle);
  EEPROM.put(EEPROM_CALIB_MAGIC_OFFSET, (unsigned short)CALIB_EEPROM_MAGIC);
}

void ORM::saveMotionLimitsToEeprom() {
  EEPROM.put(EEPROM_SPEED_MAX_OFFSET, j_speed_max);
  EEPROM.put(EEPROM_ACCELERATION_OFFSET, j_acceleration);
  EEPROM.put(EEPROM_MOTION_MAGIC_OFFSET, (unsigned short)MOTION_EEPROM_MAGIC);
}

void ORM::saveMinPwmToEeprom() {
  EEPROM.put(EEPROM_MIN_PWM_OFFSET, (short)min_pwm);
  EEPROM.put(EEPROM_PWM_MAGIC_OFFSET, (unsigned short)PWM_EEPROM_MAGIC);
}

void ORM::saveAdcSamplesToEeprom() {
  EEPROM.put(EEPROM_ADC_SAMPLES_N_OFFSET, (short)adc_samples_n);
  EEPROM.put(EEPROM_ADC_SAMPLES_MAGIC_OFFSET, (unsigned short)ADC_SAMPLES_EEPROM_MAGIC);
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
    if (cmd == OSP_ORA_CMD_SET_MAX_SPEED){
      cmdSetMaxSpeed();
    }
    if (cmd == OSP_ORA_CMD_SET_ACCELERATION){
      cmdSetAcceleration();
    }
    if (cmd == OSP_ORA_CMD_SET_MIN_PWM){
      cmdSetMinPwm();
    }
    if (cmd == OSP_ORA_CMD_SET_ADC_SAMPLES_N){
      cmdSetAdcSamplesN();
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
    
  unsigned int angle = j_angle_read;

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_ANGLE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = current_address;
  osp_output_buffer[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xFF;
  osp_output_buffer[OSP_ORM_ANGLE_MSB_INDEX] = angle >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::oraInfoCurrentSpeed(){
  ospPrepareOutputBuffer();  
  short int speed = j_speed_current;

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_SPEED;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = current_address;
  osp_output_buffer[OSP_ORM_SPEED_LSB_INDEX] = speed & 0xFF;
  osp_output_buffer[OSP_ORM_SPEED_MSB_INDEX] = speed >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::oraInfoReadSpeed(){
  ospPrepareOutputBuffer();
  short int speed = j_speed_read;

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_SPEED;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = current_address+1;
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
    //oraInfoReadSpeed();
    last_millis = current_millis;
  }
}


short ORM::readAngle(){
  return 0;
}

long isqrt(long x) {
    if (x <= 0)
        return 0;

    long r = x;
    long y = (r + x / r) / 2;

    while (y < r) {
        r = y;
        y = (r + x / r) / 2;
    }

    return r;
}

void ORM::updateActuatorsPosition(){  
  unsigned long current_millis = millis();
  
  if(current_millis-speed_millis>ORM_SPEED_UPDATE_INTERVAL_MS){
    speed_millis = current_millis;

    if (control_mode == CONTROL_MODE_FORCE_PWM) {
      target_angle_stable_iterations = 0;
      if (control_pwm > 0) {
        analogWrite(A_ZERO_FWD_PIN, control_pwm);
        analogWrite(A_ZERO_BCK_PIN, 0);
      } else {
        analogWrite(A_ZERO_FWD_PIN, 0);
        analogWrite(A_ZERO_BCK_PIN, -control_pwm);
      }
      return;
    }

    if (motor_power == 0){
      // If no motor power - just apply the same angle that is currenrly read
      j_angle_current = j_angle_read;
      j_angle_desired = j_angle_read;
      j_speed_current = 0;
      speed_control_effort = 0;
      impulse_debt = 0;
      speed_diff_sign_prev = 0;
      target_angle_stable_iterations = 0;
      analogWrite(A_ZERO_FWD_PIN, 0);
      analogWrite(A_ZERO_BCK_PIN, 0);
      return;
    }

    if(j_angle_force){
      // If the angle is forced - apply it immediately, no acceleration logic
      j_angle_current = j_angle_desired;
      j_speed_current = 0;
      speed_control_effort = 0;
      impulse_debt = 0;
      speed_diff_sign_prev = 0;
      target_angle_stable_iterations = 0;
      analogWrite(A_ZERO_FWD_PIN, 0);
      analogWrite(A_ZERO_BCK_PIN, 0);
      return;
    }

    long target_angle_diff = (long)j_angle_desired - (long)j_angle_read;
    bool hold_speed_control_effort = false;
    if (abs(target_angle_diff) <= ORM_TARGET_ANGLE_TOLERANCE) {
      if (target_angle_stable_iterations <= ORM_TARGET_ANGLE_STABLE_ITERATIONS) {
        target_angle_stable_iterations++;
      }
      if (target_angle_stable_iterations > ORM_TARGET_ANGLE_STABLE_ITERATIONS) {
        hold_speed_control_effort = true;
      }
    } else {
      target_angle_stable_iterations = 0;
    }

    long predicted_current_speed = j_speed_read;
    if(history_records_size>0){
      int oldest_history_ptr = 0;
      if(history_records_size>=HISTORY_RECORDS_N){
        oldest_history_ptr = history_records_ptr;
      }

      long speed_sum = 0;
      for(int i=0;i<history_records_size;i++){
        int history_ptr = (oldest_history_ptr + i) % HISTORY_RECORDS_N;
        speed_sum += j_speed_read_history[history_ptr];
      }
      predicted_current_speed = speed_sum / history_records_size;
    }

    long predicted_angle = (long)j_angle_read + predicted_current_speed * (long)ORM_SPEED_PREDICTION_INTERVAL_MS / (long)ORM_MS_IN_SECOND;
    long angle_diff = (long)j_angle_desired - predicted_angle;
    long direction = sgn(angle_diff);
    long predicted_desired_speed = 0;

    if(direction!=0){
      long accelerate_speed = (long)j_speed_current + direction*(long)j_acceleration*(long)ORM_SPEED_PREDICTION_INTERVAL_MS / (long)ORM_MS_IN_SECOND;
      long abs_angle_diff = angle_diff * direction;
      long sqrt_angle_diff = isqrt(abs_angle_diff);
      long accel_sqrt = isqrt(2L*(long)j_acceleration);
      long deaccelerate_speed = direction * sqrt_angle_diff * accel_sqrt;
      long max_speed = direction * (long)j_speed_max;
      long result_speed = min(min(direction*max_speed,direction*accelerate_speed),direction*deaccelerate_speed);

      predicted_desired_speed = direction*result_speed;
    }

    j_speed_current = predicted_desired_speed;

    if (!hold_speed_control_effort) {
      long speed_diff = predicted_desired_speed - predicted_current_speed;
      int speed_diff_sign = 0;
      if(speed_diff>ORM_SPEED_DIFF_EPSILON_DEFAULT){
        speed_diff_sign = 1;
      } else if(speed_diff<-ORM_SPEED_DIFF_EPSILON_DEFAULT){
        speed_diff_sign = -1;
      }
      int impulse_debt_speed_diff_sign = 0;
      if(speed_diff>ORM_SPEED_DIFF_IMPULSE_DEPT_EPSILON){
        impulse_debt_speed_diff_sign = 1;
      } else if(speed_diff<-ORM_SPEED_DIFF_IMPULSE_DEPT_EPSILON){
        impulse_debt_speed_diff_sign = -1;
      }

      if(impulse_debt_speed_diff_sign!=0 && speed_diff_sign_prev!=0 && impulse_debt_speed_diff_sign!=speed_diff_sign_prev){

        if(abs(angle_diff)<750){
          speed_control_effort -= impulse_debt * 6 / 10;
        } else {
          speed_control_effort -= impulse_debt * 0 / 10;
        }
        impulse_debt = 0;
      }

      if(speed_control_effort>MAX_PWM*100){
        speed_control_effort = MAX_PWM*100;
      } else if(speed_control_effort<-MAX_PWM*100){
        speed_control_effort = -MAX_PWM*100;
      }

      int speed_control_effort_before = speed_control_effort;
      if(speed_diff_sign>0){
        if(abs(angle_diff)<750){
          speed_control_effort += 100*max(1,abs(speed_diff)/1000);//ORM_SPEED_UPDATE_INTERVAL_MS*(isqrt(abs(speed_diff)/10))/100;
        } else {
          speed_control_effort += 50*max(1,abs(speed_diff)/1000);
        }
        //speed_control_effort += 5;
      } else if(speed_diff_sign<0){
        if(abs(angle_diff)<750){
          speed_control_effort -= 100*max(1,abs(speed_diff)/1000);//ORM_SPEED_UPDATE_INTERVAL_MS*(isqrt(abs(speed_diff)/10))/100;
        } else {
          speed_control_effort -= 50*max(1,abs(speed_diff)/1000);
        }
      }

      if(speed_control_effort>MAX_PWM*100){
        speed_control_effort = MAX_PWM*100;
      } else if(speed_control_effort<-MAX_PWM*100){
        speed_control_effort = -MAX_PWM*100;
      }
      impulse_debt += speed_control_effort - speed_control_effort_before;
      if(impulse_debt_speed_diff_sign!=0){
        speed_diff_sign_prev = impulse_debt_speed_diff_sign;
      }
    }

    int a_zero_speed_effort = speed_control_effort/100;

    if(a_zero_speed_effort>MAX_PWM){
      a_zero_speed_effort = MAX_PWM;
    } else if(a_zero_speed_effort<-MAX_PWM){
      a_zero_speed_effort = -MAX_PWM;
    }

    if(a_zero_speed_effort!=0){
      a_zero_speed_effort = sgn(a_zero_speed_effort)*(min_pwm + (long)abs(a_zero_speed_effort)*(long)(MAX_PWM - min_pwm)/(long)MAX_PWM);
    }

    if (a_zero_speed_effort > 0) {
      analogWrite(A_ZERO_FWD_PIN, a_zero_speed_effort);
      analogWrite(A_ZERO_BCK_PIN, 0);
    } else {
      analogWrite(A_ZERO_FWD_PIN, 0);
      analogWrite(A_ZERO_BCK_PIN, -a_zero_speed_effort);
    }
  }
}


void ORM::updateSensorsMeasurements(){
  unsigned long current_sensor_millis = millis();
  int adc_read = analogRead(ORM_J_ADC_PIN);
  int angle_int = orm_max_int_angle * adc_read  / ADC_MAX;
  j_angle_read_samples[j_angle_samples_ptr] = angle_int;
  j_angle_samples_ptr++;
  j_angle_samples_ptr %= adc_samples_n;
  if(j_angle_samples_count<adc_samples_n){
    j_angle_samples_count++;
  }
  long int sum = 0;
  for (int j=0;j<j_angle_samples_count;j++){
    sum += j_angle_read_samples[j];
  }
  int filtered_angle = sum / j_angle_samples_count;
  j_angle_filtered = filtered_angle;

  filtered_angle = (float) j_angle_width * (float)(filtered_angle - servo_zero_angle) / (float)(servo_max_angle-servo_zero_angle);

  filtered_angle = filtered_angle-j_angle_correction;
  j_angle_read = filtered_angle;
  unsigned long elapsed_ms = current_sensor_millis - sensor_millis;
  if(history_records_size>0 && elapsed_ms<SPEED_MEASUREMENT_INTERVAL){
    return;
  }
  if(history_records_size==0){
    j_speed_read = 0;
  } else {
    int history_prev_ptr = history_records_ptr - 1;
    if(history_prev_ptr<0){
      history_prev_ptr = HISTORY_RECORDS_N - 1;
    }
    if(elapsed_ms==0){
      j_speed_read = 0;
    } else {
      long angle_diff = (long)j_angle_read - (long)j_angle_read_history[history_prev_ptr];
      long speed_per_second = angle_diff * (long)ORM_MS_IN_SECOND / (long)elapsed_ms;
      if(speed_per_second>32767L){
        speed_per_second = 32767L;
      } else if(speed_per_second<-32768L){
        speed_per_second = -32768L;
      }
      j_speed_read = speed_per_second;
    }
  }
  sensor_millis = current_sensor_millis;
  j_angle_read_history[history_records_ptr] = j_angle_read;
  j_speed_read_history[history_records_ptr] = j_speed_read;
  history_millis[history_records_ptr] = current_sensor_millis;
  history_records_ptr++;
  history_records_ptr %= HISTORY_RECORDS_N;
  if(history_records_size<HISTORY_RECORDS_N){
    history_records_size++;
  }
  //j_angle_desired = j_angle_read;

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
  speed_control_effort = 0;
  impulse_debt = 0;
  speed_diff_sign_prev = 0;

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
  pinMode(A_ZERO_FWD_PIN, OUTPUT);
  analogWrite(A_ZERO_FWD_PIN, 0);
  pinMode(A_ZERO_BCK_PIN, OUTPUT);
  analogWrite(A_ZERO_BCK_PIN,0);

  pinMode(BTS_L_EN, OUTPUT);
  pinMode(BTS_R_EN, OUTPUT);
  digitalWrite(BTS_L_EN, 1);
  digitalWrite(BTS_R_EN, 1);
  
  /*
  joints[0] = &j0;
  joints[1] = &j1;
  joints[2] = &j2;
  joints[3] = &j3;
  joints[4] = &j4;
  joints[5] = &j5;  
*/

  // Read the current value of the angles
  updateSensorsMeasurements();

  unsigned short calib_magic = 0;
  EEPROM.get(EEPROM_CALIB_MAGIC_OFFSET, calib_magic);
  bool calib_valid = (calib_magic == CALIB_EEPROM_MAGIC);
  unsigned short motion_magic = 0;
  EEPROM.get(EEPROM_MOTION_MAGIC_OFFSET, motion_magic);
  bool motion_valid = (motion_magic == MOTION_EEPROM_MAGIC);
  unsigned short pwm_magic = 0;
  EEPROM.get(EEPROM_PWM_MAGIC_OFFSET, pwm_magic);
  bool pwm_valid = (pwm_magic == PWM_EEPROM_MAGIC);
  unsigned short adc_samples_magic = 0;
  EEPROM.get(EEPROM_ADC_SAMPLES_MAGIC_OFFSET, adc_samples_magic);
  bool adc_samples_valid = (adc_samples_magic == ADC_SAMPLES_EEPROM_MAGIC);

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
  /*
   // INIT JOINT
   joints->setEnablePin(ORM_J_ENABLE_PIN);
   joints->setPinsInverted(false,false,true);
   //joints->disableOutputs();
   joints->setAcceleration(200000); // 200 steps per second per second
   joints->enableOutputs();
   // Populate desired speed
   j_speed_desired = (long)orm_max_int_angle * (long)500 /(long)orm_j_stepper_full_rot;
   // Set speed in steps/second
   joints->setMaxSpeed((long)j_speed_desired * (long)orm_j_stepper_full_rot / (long)orm_max_int_angle);
   //joints->setSpeed(100);
  */
  // LOAD CORRECTION ANGLE
  if (calib_valid) {
    EEPROM.get(EEPROM_ANGLE_CORRECTION_OFFSET, j_angle_correction);
  } else {
    j_angle_correction = 0;
  }
  j_angle_current = 0;

  if (calib_valid) {
    EEPROM.get(EEPROM_ANGLE_WIDTH_OFFSET,j_angle_width);
  } else {
    j_angle_width = 16384;
  }
  if(j_angle_width<=0) {
    j_angle_width = 16384;
  }

  if (calib_valid) {
    EEPROM.get(EEPROM_DEFAULT_ANGLE_OFFSET,j_angle_desired);
  } else {
    j_angle_desired = 0;
  }

  unsigned short zero_adc = 0;
  unsigned short max_adc = 0;

  if (calib_valid) {
    EEPROM.get(EEPROM_SERVO_ZERO_ANGLE_ADC,zero_adc);
    EEPROM.get(EEPROM_SERVO_MAX_ANGLE_ADC,max_adc);
  }

  if (!calib_valid || zero_adc == 0) {
   zero_adc = default_servo_zero_angle;
  }
  servo_zero_angle = zero_adc;
  if (!calib_valid || max_adc ==0 ) {
   max_adc = default_servo_max_angle;
  }
  servo_max_angle = max_adc;

  j_speed_max = orm_j_speed_max_default;
  j_acceleration = orm_j_acceleration_default;
  if (motion_valid) {
    EEPROM.get(EEPROM_SPEED_MAX_OFFSET, j_speed_max);
    EEPROM.get(EEPROM_ACCELERATION_OFFSET, j_acceleration);
  }
  if (j_speed_max <= 0) {
    j_speed_max = orm_j_speed_max_default;
  }
  if (j_acceleration <= 0) {
    j_acceleration = orm_j_acceleration_default;
  }

  min_pwm = MIN_PWM_DEFAULT;
  if (pwm_valid) {
    short eeprom_min_pwm = 0;
    EEPROM.get(EEPROM_MIN_PWM_OFFSET, eeprom_min_pwm);
    if (eeprom_min_pwm >= 0 && eeprom_min_pwm <= MAX_PWM) {
      min_pwm = eeprom_min_pwm;
    }
  }

  adc_samples_n = ADC_SAMPLES_N_MAX;
  if (adc_samples_valid) {
    short eeprom_adc_samples = 0;
    EEPROM.get(EEPROM_ADC_SAMPLES_N_OFFSET, eeprom_adc_samples);
    if (eeprom_adc_samples >= 1 && eeprom_adc_samples <= ADC_SAMPLES_N_MAX) {
      adc_samples_n = eeprom_adc_samples;
    }
  }

  // For now setting the desired angle to the one that was read at the start 
  j_angle_desired = j_angle_read;

}

ORM::ORM(){
  
}
