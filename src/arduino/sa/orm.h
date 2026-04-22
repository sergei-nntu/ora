#ifndef ORM_H
#define ORM_H

#include "osp.h"

#define ORA_INDEX   0
#define ORA_JOINTS_COUNT 1

#define STAT_SAMPLE_SIZE          20

#define JS_ANGLE_SCALE_FACTOR_DIVISOR 1024

#define ORM_SPEED_MAX             2000  // 360 * 1000 / 16000 = 22.5 degrees per second 

#define ORM_SPEED_UPDATE_INTERVAL_MS 25   // ms

#define ORM_ACCELERATION_MAX      2000 // 360 * 1000 / 16000 = 22.5 degrees per second^2

#define ORM_DEACCELERATE_ANGLE      1000  

#define ORM_MS_IN_SECOND            1000

// Update Interval in Milliseconds
#define UPDATE_INTERVAL  100 // 10 Hz


//#define ADC_SAMPLES_N_MAX  8
#define ADC_SAMPLES_N_MAX  3

#define MOTOR_POWER_PIN   12

// Define the pins for PID-controlled actuator (Actuator 0)
#define A_ZERO_PWM_PIN 11
#define A_ZERO_FWD_PIN 10
#define A_ZERO_BCK_PIN 9

// Convenience sign function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

const long orm_j_encoder_adc_range = 1024; // Max Value of Encoder ADC output


const long orm_max_int_angle = 32768;      // Max Positive Value of Integer Angle correspnods to 2*Pi
                                                 // ORA-3. ORA-4. ORA-3. ORA-2 ORA-2 ORA-2

const long orm_180_angle_width = orm_max_int_angle/2;

const short orm_j_stepper_full_rot[ORA_JOINTS_COUNT] = {16000};  // Number of steps to reach 2*Pi Angle

const short orm_j_speed_max_default[ORA_JOINTS_COUNT] =     {3000};
const short orm_j_speed_min[ORA_JOINTS_COUNT] =             {400};
const short orm_j_acceleration_default[ORA_JOINTS_COUNT] =  {250};

const  int default_servo_zero_angle[ORA_JOINTS_COUNT] = {2700};
const  int default_servo_max_angle[ORA_JOINTS_COUNT] = {28500};

class ORM {
  private:
    // OSP COMMUNICATION VARIABLES
    char            osp_output_buffer[OSP_BUFFER_SIZE];
    unsigned char   osp_input_buffer[OSP_BUFFER_SIZE];
    int             osp_ptr=0;
    short           current_address;

    unsigned long speed_millis = 0;

    double Ki, Kp, Kd; // Pid Coefs

    short j_speed_desired[ORA_JOINTS_COUNT] =   {500}; // Default Speed 11.25 degrees second. Must be populated in the constructor
    short j_speed_current[ORA_JOINTS_COUNT] =   {0};
    short j_speed_read[ORA_JOINTS_COUNT] =      {0};
    short j_speed_read_prev[ORA_JOINTS_COUNT] = {0};
    short j_speed_max[ORA_JOINTS_COUNT];
    long j_accel_read[ORA_JOINTS_COUNT] = {0};
    long j_accel_prev[ORA_JOINTS_COUNT] = {0};
    short j_acceleration[ORA_JOINTS_COUNT];
    short j_angle_desired[ORA_JOINTS_COUNT] =   {0};
    short j_angle_current[ORA_JOINTS_COUNT] =   {0};
    short j_angle_read[ORA_JOINTS_COUNT] =      {0};
    short j_angle_read_prev[ORA_JOINTS_COUNT] = {0};
    short j_angle_correction[ORA_JOINTS_COUNT] ={0};
    short j_angle_width[ORA_JOINTS_COUNT] =     {0};
    char  j_angle_force[ORA_JOINTS_COUNT] =     {0};
    short j_callibr_left[ORA_JOINTS_COUNT] = {2};
    short js_angle_scale_factor[ORA_JOINTS_COUNT] = {1024}; // Scale factor to be applied prior to sending the angle to servos
    short js_small_angle_threshold[ORA_JOINTS_COUNT] = {1024}; // If the difference between the desired and the current angle does not exceed this value - do not apply the acceleration.

    short servo_zero_angle[ORA_JOINTS_COUNT];
    short servo_max_angle[ORA_JOINTS_COUNT];

    short j_angle_read_samples[ORA_JOINTS_COUNT][ADC_SAMPLES_N_MAX];
    short adc_samples_n = ADC_SAMPLES_N_MAX;
    short j_angle_samples_ptr = 0;
    short j_angle_samples_count = 0;
    short j_angle_filtered[ORA_JOINTS_COUNT];


    char motor_power = 1;

    // GRIPPER CONTROL VARIABLES
    short gripper_angle = 0; // INT ANGLE -16384 .. 16383
    //Servo * gripper_servo;

    // STATISTICAL FILTERING 

    short j_angle_sample[ORA_JOINTS_COUNT * STAT_SAMPLE_SIZE];

    int read_samples_size[ORA_JOINTS_COUNT] = {0};
    int read_samples_ptr[ORA_JOINTS_COUNT] = {0};
    /*
    short j_speed_current[JOINTS_COUNT] = {0,};
    short j_goal_achieved[JOINTS_COUNT] = {16,};  
    */

    // Flow Control Functions
    void            ospHandleCommand();
    void            sendUpdateInfo();   
    void            updateActuatorsPosition(); 
    void            updateSensorsMeasurements();

    // Incoming Command Processing Functions
    void ospHandleORACommand();
    void ospHandleGenericCommand();

    void cmdSetAngle();
    void cmdMakeSteps();
    void cmdSetCorrAngle();
    void cmdSetAngleWidth();
    void cmdSetMotorPower();
    void cmdSetForcePwm();
    void cmdSetMaxSpeed();
    void cmdSetAcceleration();
    void cmdSetMinPwm();
    void cmdSetAdcSamplesN();

    void cmdSetPidProportional();
    void cmdSetPidIntegral();
    void cmdSetPidDifferential();

    void calibrateJoint(int jointNo);
    void cmdCalibrateJoint();

    // Outcoming Commands Generation Functinos
    void ospPrepareOutputBuffer();
    void oraInfoCurrentAngle();
    void oraInfoCurrentSpeed();
    // Data Input Function
    short readAngle(int actuatorNo);

    // Service Commands 
    void setPidProportional(double value);
    void setPidIntegral(double value);
    void setPidDifferential(double value);

    double getPidProportional();
    double getPidIntegral();
    double getPidDifferential();

    void loadPidFromEeprom();
    void savePidToEeprom();
    void saveCalibrationToEeprom(int jointNo);
    void saveMotionLimitsToEeprom(int jointNo);
    void saveMinPwmToEeprom();
    void saveAdcSamplesToEeprom();

    void _updateMotorDCTunings();
    unsigned long last_millis; 
    int control_pwm = 0;
    int control_mode = 0;
    int min_pwm = 0;

  public:
    // Method to poll the Serial input. Should be called at least once per the execution loop
    ORM();
    void setup();
    void ospSerialLoop();
};

#endif /*ORM_H*/
