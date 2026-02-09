# Importing Libraries
import serial
import time
import datetime
import math
import threading
import glob
import sys
import json

# OSP DATA INDICIES WITHIN THE MESSAGE

OSP_MSG_DEV_INDEX = 2
OSP_MSG_CMD_INDEX = 3
OSP_MSG_ADDRESS_INDEX = 4
OSP_ORA_ADDRESS_INDEX = 4
OSP_BYTE_PARAM_INDEX = 4
OSP_INT_PARAM_MSB_INDEX = 5
OSP_INT_PARAM_LSB_INDEX = 4
OSP_ORM_MSB_PROPORTIONAL_PARAM = 5
OSP_ORM_LSB_PROPORTIONAL_PARAM = 4
OSP_ORM_MSB_INTEGRAL_PARAM = 5
OSP_ORM_LSB_INTEGRAL_PARAM = 4
OSP_ORM_MSB_DIFFERENTIAL_PARAM = 5
OSP_ORM_LSB_DIFFERENTIAL_PARAM = 4 

# OSP DEVICE TYPES

OSP_DEV_GENERIC = 0
OSP_DEV_ORM = 1
OSP_DEV_OBP = 2
OSP_DEV_O2D = 3
OSP_DEV_OQP = 4
OSP_DEV_ORA = 5

# OSP COMMANDS 

OSP_CMD_REQ_DEV_TYPE = 0x01
OSP_INFO_DEV_TYPE = 0x11
OSP_OBP_CMD_REQ_SOC = 0x01
OSP_OBP_CMD_REQ_VOLTAGE = 0x02
OSP_OBP_CMD_REQ_CURRENT = 0x03
OSP_OBP_CMD_REQ_STATUS = 0x04

OSP_OBP_INFO_SOC = 0x11
OSP_OBP_INFO_VOLTAGE = 0x12
OSP_OBP_INFO_CURRENT = 0x13
OSP_OBP_INFO_STATUS = 0x14


OSP_ORM_JOINT_INDEX = 4
OSP_ORM_ANGLE_MSB_INDEX = 6
OSP_ORM_ANGLE_LSB_INDEX = 5
OSP_ORM_ANGLE_FORCE_INDEX = 7


OSP_OQP_TRAJECTORY_POINT_NO_INDEX = 7

OSP_ORM_CMD_SET_ANGLE = 0x02
OSP_ORM_CMD_SET_SPEED = 0x03
OSP_ORM_CMD_SET_CORR_ANGLE= 0x05
OSP_ORM_CMD_SET_ANGLE_WIDTH= 0x06
OSP_ORM_CMD_SET_PID_PROPORTIONAL = 0x07
OSP_ORM_CMD_SET_PID_INTEGRAL = 0x08
OSP_ORM_CMD_SET_PID_DIFFERENTIAL = 0x09
OSP_ORM_CMD_SET_MOTOR_POWER = 0x0D
OSP_ORM_CMD_CALIBRATE_JOINT = 0x0E
OSP_ORM_INFO_ANGLE = 0x12
OSP_ORM_INFO_SPEED = 0x13
OSP_ORM_INFO_STATUS = 0x14
OSP_ORM_INFO_IR_STATUS = 0x15

OSP_OQP_CMD_SET_JOINT_ANGLE  = 0x02
OSP_OQP_CMD_SET_JOINT_SPEED  =  0x03
OSP_OQP_CMD_SET_JOINT_CORR_ANGLE  = 0x05
OSP_OQP_CMD_SET_JOINT_ANGLE_WIDTH  = 0x06
OSP_OQP_CMD_SET_JOINT_DEFAULT_ANGLE  = 0x07
OSP_OQP_CMD_SET_TRAJECTORY_ANGLE = 0x08
OSP_OQP_CMD_SET_TRAJECTORY_POINTS_COUNT = 0x09
OSP_OQP_CMD_SET_TRAJECTORY_DURATION = 0x0A
OSP_OQP_CMD_START_TRAJECTORY = 0x0B
OSP_OQP_CMD_STOP_TRAJECTORY = 0x0C
OSP_OQP_INFO_RANGE = 0x15
OSP_OQP_INFO_PRESSURE = 0x16
OSP_OQP_INFO_EULER_ANGLE = 0x17
OSP_OQP_CMD_GET_CORR_ANGLE  = 0x25
OSP_OQP_CMD_GET_ANGLE_WIDTH  = 0x26
OSP_OQP_CMD_GET_DEFAULT_ANGLE  = 0x27
OSP_OQP_CMD_INFO_CORR_ANGLE  = 0x35
OSP_OQP_CMD_INFO_ANGLE_WIDTH  = 0x36
OSP_OQP_CMD_INFO_DEFAULT_ANGLE  = 0x37

OSP_ORA_CMD_SET_ANGLE = 0x02
OSP_ORA_CMD_SET_FORCE_PWM = 0x0F
OSP_ORA_CMD_SET_CORR_ANGLE = 0x05
OSP_ORA_CMD_SET_ANGLE_WIDTH = 0x06
OSP_ORA_CMD_SET_PID_PROPORTIONAL = 0x07
OSP_ORA_CMD_SET_PID_INTEGRAL = 0x08
OSP_ORA_CMD_SET_PID_DIFFERENTIAL = 0x09

OSP_OQP_DURATION_MSB_INDEX = 5
OSP_OQP_DURATION_LSB_INDEX = 4

OSP_ORM_JOINT_STATUS_RUNNING_BIT_INDEX = 0
OSP_ORM_JOINT_STATUS_POWERED_BIT_INDEX = 1
OSP_ORM_JOINT_STATUS_ERROR_BIT_INDEX =   2

OSP_DEV_CURRENT = OSP_DEV_OBP

OSP_COMMAND_LENGTH = 9
OSP_BUFFER_SIZE = 10


ORM_INT_ANGLE_MAX = 65536/2

class OSP:
    lock = threading.Lock()
    
    oqp_joints_directions = [-1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1]
    
    oqp_cached_angles = [-65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535]
    
    oqp_corr_angles = [-65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535]
    
    oqp_angle_widths = [-65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535]
    
    oqp_default_angles = [-65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535, -65535]
    
    oqp_significant_delta = 50
    
    
    orm_is_recording = False
    
    orm_is_playback = False
    
    orm_record_buffer = []
    
    orm_playback_buffer = []
    
    orm_playback_index = 0
    
    orm_playback_start_time = None
    
    orm_record_start_time = None
    
    orm_playback_thread = None
    
    
    #command_buffer_pattern = [ 0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77]
    command_buffer_pattern = [ 0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]
    input_index = 0
    output_buffer = []
    input_buffer = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    osp_serial = None
    osp_dev_type = 0
    output_thread = None
    input_thread = None
    closed = False 
    
    soc = 0
    voltage = 0
    current = 0
    
    # JOINT MEASUREMENTS
    joint_angle = [0,0,0,0,0,0,0]
    joint_speed = [0,0,0,0,0,0,0]
    
    # JOINT STATUS FLAGS
    joint_status_powered = [0,0,0,0,0,0,0]
    joint_status_running = [0,0,0,0,0,0,0]
    joint_status_error = [0,0,0,0,0,0,0]
    
    # LEGS MEASUREMENTS
    legs_pressure = [0,0,0,0]
    legs_range = [0,0,0,0]
    
    # EULER ANGLES
    euler_angles = [0,0,0]
    euler_buff_len = 10
    euler_angles_buf = [[],[],[]]
    
    
    def __init__(self,port_name):
        # '/dev/ttyACM1'
        self.osp_serial = serial.Serial(port=port_name, baudrate=115200, timeout=.1)
        #print("Starting Output Thread")
        self.output_thread = threading.Thread(target=self.output_thread, daemon=True)  
        self.output_thread.start()
        #print("Starting Input Thread")
        self.input_thread = threading.Thread(target=self.input_thread, daemon=True)  
        self.input_thread.start()
    
    def stop(self):
        self.closed = True
        self.osp_serial.close()
        
    
    def get_dev_type(self):
        return self.osp_dev_type
    
    def output_thread(self):
        while self.closed is not True:
            self.lock.acquire()
            while len(self.output_buffer) > 0:
                #print("Sending Byte"+str(self.output_buffer[0]));
                self.osp_serial.write(bytes([self.output_buffer.pop(0)]))
            self.lock.release()
            time.sleep(0.00001)    
     
    def osp_send_command(self,cmd_bytes):
        self.lock.acquire()
        self.output_buffer += cmd_bytes
        #print("Output buffer = "+str(self.output_buffer))
        self.lock.release()
        
    def osp_req_device_type(self):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_GENERIC
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_CMD_REQ_DEV_TYPE
        self.osp_send_command(cmd_bytes)
     
    
    def oqp_set_angle(self, joint, angle):
        try:
            angle_int = self.oqp_joints_directions[joint]*int(angle * 16384 / math.pi)
            self.set_angle(joint, angle_int)
        except:
            print("Exception in calculations occured. Do not apply the angle to the hardware")
    
    def set_angle(self, joint, angle, force = 0):
        #print("Sending Set Angle Command: "+str(joint)+" -> "+str(angle))
        
        if abs(self.oqp_cached_angles[joint] - angle) > self.oqp_significant_delta: 
            self.oqp_cached_angles[joint] = angle
            cmd_bytes = self.command_buffer_pattern.copy()
            cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
            cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_JOINT_ANGLE
            cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
            cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
            cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
            cmd_bytes[OSP_ORM_ANGLE_FORCE_INDEX] = force
            self.osp_send_command(cmd_bytes)

    def ora_set_angle(self, address, angle, force = 0):
        #print("Sending Set Angle Command: "+str(joint)+" -> "+str(angle))
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_ANGLE
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        cmd_bytes[OSP_ORM_ANGLE_FORCE_INDEX] = force
        self.osp_send_command(cmd_bytes)

    def ora_set_force_pwm(self, address, pwm):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_FORCE_PWM
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        pwm_value = int(pwm)
        if pwm_value > 255:
            pwm_value = 255
        elif pwm_value < -255:
            pwm_value = -255
        if pwm_value < 0:
            pwm_value = (pwm_value + 0x10000) & 0xffff
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (pwm_value >> 8) & 0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = pwm_value & 0xff
        self.osp_send_command(cmd_bytes)

    def ora_set_pid_proportional(self, address, value):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_PID_PROPORTIONAL
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        value_int = int(value * 10000)
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (value_int >> 8) & 0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = value_int & 0xff
        self.osp_send_command(cmd_bytes)

    def ora_set_pid_integral(self, address, value):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_PID_INTEGRAL
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        value_int = int(value * 10000)
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (value_int >> 8) & 0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = value_int & 0xff
        self.osp_send_command(cmd_bytes)

    def ora_set_pid_differential(self, address, value):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_PID_DIFFERENTIAL
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        value_int = int(value * 10000)
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (value_int >> 8) & 0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = value_int & 0xff
        self.osp_send_command(cmd_bytes)

    def ora_set_corr_angle(self, address, angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_CORR_ANGLE
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) & 0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes)

    def ora_set_angle_width(self, address, angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORA
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORA_CMD_SET_ANGLE_WIDTH
        cmd_bytes[OSP_MSG_ADDRESS_INDEX] = address
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) & 0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes)
        
    def set_speed(self, joint, speed):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_SPEED
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (speed >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = speed & 0xff
        self.osp_send_command(cmd_bytes)
        
    def set_motor_power(self,power):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_MOTOR_POWER
        cmd_bytes[OSP_BYTE_PARAM_INDEX] = power
        self.osp_send_command(cmd_bytes)
    
    def set_pid_proportional(self, value):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_PID_PROPORTIONAL
        value_int = int(value * 10000)
        cmd_bytes[OSP_ORM_MSB_PROPORTIONAL_PARAM] = (value_int >> 8) & 0xff
        cmd_bytes[OSP_ORM_LSB_PROPORTIONAL_PARAM] = value_int & 0xff
        self.osp_send_command(cmd_bytes)
    
    def set_pid_integral(self, value):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_PID_INTEGRAL
        value_int = int(value * 10000)
        cmd_bytes[OSP_ORM_MSB_INTEGRAL_PARAM] = (value_int >> 8) & 0xff
        cmd_bytes[OSP_ORM_LSB_INTEGRAL_PARAM] = value_int & 0xff
        self.osp_send_command(cmd_bytes)
    
    def set_pid_differential(self, value):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_PID_DIFFERENTIAL
        value_int = int(value * 10000)
        cmd_bytes[OSP_ORM_MSB_DIFFERENTIAL_PARAM] = (value_int >> 8) & 0xff
        cmd_bytes[OSP_ORM_LSB_DIFFERENTIAL_PARAM] = value_int & 0xff
        self.osp_send_command(cmd_bytes)
        
        
    def orm_calibrate_joint(self, joint):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_CALIBRATE_JOINT
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        self.osp_send_command(cmd_bytes)
        
    def set_corr_angle(self,joint,angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_JOINT_CORR_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes)
        
    def set_angle_width(self,joint, angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_JOINT_ANGLE_WIDTH
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes) 
        
    def set_oqp_default_angle(self,joint,angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_JOINT_DEFAULT_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes)        
    
    def get_corr_angle(self,joint):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_GET_CORR_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        self.osp_send_command(cmd_bytes) 
    
    def get_angle_width(self,joint):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_GET_ANGLE_WIDTH
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        self.osp_send_command(cmd_bytes)  
        
    def get_oqp_default_angle(self,joint):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_GET_DEFAULT_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        self.osp_send_command(cmd_bytes)   
    
    # Angle is in radians
    def set_oqp_trajectory_angle(self, joint, angle, point_no):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_TRAJECTORY_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        angle_int = self.oqp_joints_directions[joint]*int(angle * 16384 / math.pi)
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle_int >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle_int & 0xff
        cmd_bytes[OSP_OQP_TRAJECTORY_POINT_NO_INDEX] = point_no
        
        print("SetTrajectoryAngle. JOINT: "+str(joint)+". POINT: "+str(point_no)+ ". ANGLE: "+str(angle_int))
        self.osp_send_command(cmd_bytes) 
    
    def set_oqp_trajectory_duration(self,duration_ms):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_TRAJECTORY_DURATION
        duration_int = int(duration_ms)
        cmd_bytes[OSP_OQP_DURATION_MSB_INDEX] = (duration_int >> 8) &0xff
        cmd_bytes[OSP_OQP_DURATION_LSB_INDEX] = duration_int & 0xff
        
        print("OQP: Set Trajectory Duration. T = "+str(duration_int)+" ms")
        self.osp_send_command(cmd_bytes) 
        
    def set_oqp_trajectory_points_count(self, points_count):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_SET_TRAJECTORY_POINTS_COUNT
        points_count_int = int(points_count)
        cmd_bytes[OSP_BYTE_PARAM_INDEX] = points_count_int
        print("OQP: Set Points Count. N = "+str(points_count_int))
        self.osp_send_command(cmd_bytes)
    
    def oqp_start_trajectory(self):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_START_TRAJECTORY
        self.osp_send_command(cmd_bytes)
    
    def oqp_stop_trajectory(self):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_OQP
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_OQP_CMD_STOP_TRAJECTORY
        self.osp_send_command(cmd_bytes)
                    
    def osp_info_dev_type(self):
        self.osp_dev_type = self.input_buffer[OSP_BYTE_PARAM_INDEX]
        #print("Device Type Received:",self.osp_dev_type);
                
    def osp_obp_info_soc(self):
        soc_lsb = self.input_buffer[OSP_INT_PARAM_LSB_INDEX]
        soc_msb = self.input_buffer[OSP_INT_PARAM_MSB_INDEX]
        soc = soc_lsb | (soc_msb << 8)
        self.soc = soc
        #print("SOC: "+str(soc))
        
    def osp_obp_info_current(self):
        current_lsb = self.input_buffer[OSP_INT_PARAM_LSB_INDEX]
        current_msb = self.input_buffer[OSP_INT_PARAM_MSB_INDEX]
        current = current_lsb | (current_msb << 8)
        if current >> 15 != 0:
            # Two's complement decoding
            current = -((~current & 0xffff)+1)
        self.current = current
        #print("I: "+str(current)+" mA")
        
    def osp_obp_info_voltage(self):
        v_lsb = self.input_buffer[OSP_INT_PARAM_LSB_INDEX]
        v_msb = self.input_buffer[OSP_INT_PARAM_MSB_INDEX]
        v = v_lsb | (v_msb << 8)
        self.voltage = v
        #print("V: "+str(v)+" mV")

    def orm_info_angle(self):
        #print("ORM Info Angle")
        actuator_no = self.input_buffer[4]
        angle = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if angle & 0x8000 !=0:
            angle = -((~angle & 0xffff) + 1)
        self.joint_angle[actuator_no] = angle
        if self.orm_is_recording:
            timediff = (datetime.datetime.now() - self.orm_record_start_time).total_seconds()
            self.orm_record_buffer.append({'joint':actuator_no,'angle':angle,'timestamp':timediff})
        #        if actuator_no == 5:
        #print("Current Angle For Actuator "+str(actuator_no)+" is "+str(angle))
   
    def orm_info_speed(self):
        actuator_no = self.input_buffer[4]
        speed = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if speed & 0x8000 !=0:
            speed = -((~speed & 0xffff) + 1)
        self.joint_speed[actuator_no] = speed
        #print("Current Speed For Actuator "+str(actuator_no)+" is "+str(speed))
    
    def orm_start_trajectory_record(self):
        print("ORM. Trajectory Recording Started")
        self.orm_record_buffer = []
        self.orm_record_start_time = datetime.datetime.now()
        self.orm_is_recording = True
        
    
    def orm_stop_trajectory_record(self):
        print("ORM. Trajectory Recording Finished")
        recorded_buffer = self.orm_record_buffer
        self.orm_is_recording = False
        
        return recorded_buffer
    
    def orm_playback_thread_function(self):
        
        while self.orm_playback_index<len(self.orm_playback_buffer):
            print("Running The Playback thread")
            while self.orm_playback_buffer[self.orm_playback_index]['timestamp'] > (datetime.datetime.now() - self.orm_playback_start_time).total_seconds():
                time.sleep(0.001)
            if self.orm_playback_index<(len(self.orm_playback_buffer) -1):
                self.set_angle(self.orm_playback_buffer[self.orm_playback_index]['joint'], self.orm_playback_buffer[self.orm_playback_index]['angle'],1)
            else:
                self.set_angle(self.orm_playback_buffer[self.orm_playback_index]['joint'], self.orm_playback_buffer[self.orm_playback_index]['angle'])
            self.orm_playback_index+=1
            time.sleep(0.001)
    
    def orm_start_trajectory_playback(self, playback_buffer):
        print("ORM. Trajectory Playback Started")
        self.orm_playback_index = 0
        self.orm_playback_buffer = playback_buffer
        self.orm_playback_start_time = datetime.datetime.now()
        self.orm_playback_thread = threading.Thread(target=self.orm_playback_thread_function, daemon=True)  
        self.orm_playback_thread.start()
        
    
    def oqp_info_pressure(self):
        #print("oqp_info_pressure")
        leg_no = self.input_buffer[4]
        pressure = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if pressure & 0x8000 !=0:
            pressure = -((~pressure & 0xffff) + 1)
        self.legs_pressure[leg_no] = pressure
        
        
    def oqp_info_range(self):
        #print("oqp_info_range")
        leg_no = self.input_buffer[4]
        range = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if range & 0x8000 !=0:
            range = -((~range & 0xffff) + 1)
        self.legs_range[leg_no] = range
  
    def oqp_info_euler_angle(self):
        #print("oqp_info_euler_angle")
        euler_angle_no = self.input_buffer[4]
        angle = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if angle & 0x8000 !=0:
            angle = -((~angle & 0xffff) + 1)
        #print("oqp_info_euler_angle "+str(euler_angle_no)+": "+str(angle))  
        
        self.euler_angles_buf[euler_angle_no].append(angle)
        if(len(self.euler_angles_buf[euler_angle_no])>self.euler_buff_len):
            self.euler_angles_buf[euler_angle_no].pop(0)
         
        self.euler_angles[euler_angle_no] = sum(self.euler_angles_buf[euler_angle_no]) / len(self.euler_angles_buf[euler_angle_no])
                
    
    def oqp_info_corr_angle(self):
        actuator_no = self.input_buffer[4]
        angle = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if angle & 0x8000 !=0:
            angle = -((~angle & 0xffff) + 1)
        self.oqp_corr_angles[actuator_no] = angle
        
    def oqp_info_angle_width(self):
        actuator_no = self.input_buffer[4]
        angle = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if angle & 0x8000 !=0:
            angle = -((~angle & 0xffff) + 1)
        self.oqp_angle_widths[actuator_no] = angle
        
    def oqp_info_default_angle(self): 
        actuator_no = self.input_buffer[4]
        angle = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if angle & 0x8000 !=0:
            angle = -((~angle & 0xffff) + 1)
        self.oqp_default_angles[actuator_no] = angle
    
    def oqp_get_euler_angle(self,euler_angle_no):
        return self.euler_angles[euler_angle_no]
    
    def extract_bit_with_index(self, word, bit_index):
        return (word & (1 << bit_index)) >> bit_index
        
    def orm_info_status(self):  
        actuator_no = self.input_buffer[4]
        status_word = self.input_buffer[5] | (self.input_buffer[6] << 8)
        
        running = self.extract_bit_with_index(status_word, OSP_ORM_JOINT_STATUS_RUNNING_BIT_INDEX)
        powered = self.extract_bit_with_index(status_word, OSP_ORM_JOINT_STATUS_POWERED_BIT_INDEX)
        error = self.extract_bit_with_index(status_word, OSP_ORM_JOINT_STATUS_ERROR_BIT_INDEX)
        
        self.joint_status_running[actuator_no] = running
        self.joint_status_powered[actuator_no] = powered
        self.joint_status_error[actuator_no] = error
        #print("JOINT "+str(actuator_no)+ " RUNNING = "+str(running)+"; POWERED = "+str(powered)+"; ERROR = "+str(error))
       
       
    def orm_info_ir_status(self):   
        byte0 = self.input_buffer[4]
        byte1 = self.input_buffer[5]
        byte2 = self.input_buffer[6]
        byte3 = self.input_buffer[7]
        #print("IR Bytes: "+str(byte0)+", "+str(byte1)+", "+str(byte2)+", "+str(byte3))                     
        
    
    def orm_is_running(self):
        return 1 in self.joint_status_running
        
    def orm_set_angle(self, joint, angle):
        print("Setting Angle to ORM joint"+str(joint)+": "+str(angle))
        int_angle = int(angle * ORM_INT_ANGLE_MAX / (2* math.pi))
        self.set_angle(joint, int_angle)
        
    def orm_set_speed(self, joint, speed): # In Radians/Sec
        if speed > 0: # Sanity Check
            int_speed =  int(ORM_INT_ANGLE_MAX * speed / (2*math.pi))
            self.set_speed(joint, int_speed)
        
    
    def oqp_save_eeprom_to_file(self, fname):
        for i in range(0,len(self.oqp_corr_angles)):
            self.get_corr_angle(i)
            time.sleep(0.2)
        
        for i in range(0,len(self.oqp_angle_widths)):
            self.get_angle_width(i)
            time.sleep(0.2)
            
        for i in range(0,len(self.oqp_default_angles)):
            self.get_oqp_default_angle(i)
            time.sleep(0.2)
        
        time.sleep(1)
        
        dict = {}
        
        dict["default_angles"] = self.oqp_default_angles
        dict["corr_angles"] = self.oqp_corr_angles
        dict["angle_widths"] = self.oqp_angle_widths
         
        with open(fname,'w') as f:
            json.dump(dict,f)
        
        
    def oqp_load_eeprom_from_file(self, fname):
        with open(fname) as f:
            dict = json.load(f)
            angle_widths = dict["angle_widths"]
            corr_angles = dict["corr_angles"]
            default_angles = dict["default_angles"]
            for i in range(0,len(angle_widths)):
                self.set_angle_width(i, angle_widths[i])
                time.sleep(0.1)
            for i in range(0,len(corr_angles)):
                self.set_corr_angle(i, corr_angles[i])
                time.sleep(0.1)
            for i in range(0,len(default_angles)):
                self.set_oqp_default_angle(i, default_angles[i])
                time.sleep(0.1)
        
    def input_thread(self):
        while self.closed is not True:
            try:
                #print("Input Thread....")
                bts = self.osp_serial.read()
                #print("Bytes_Read:"+str(bts))
                if len(bts)>0:
                    #print("Bytes_Read:"+str(bts))
                    bt = bts[0]
                    if self.command_buffer_pattern[self.input_index] == bt or self.command_buffer_pattern[self.input_index] ==0:
                        self.input_buffer[self.input_index] = bt
                        self.input_index += 1
                        if self.input_index >= len(self.command_buffer_pattern):
                            #print("Command Received:"+str(self.input_buffer))
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_OBP:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OBP_INFO_SOC:
                                    self.osp_obp_info_soc()
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OBP_INFO_CURRENT:
                                    self.osp_obp_info_current()
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OBP_INFO_VOLTAGE:
                                    self.osp_obp_info_voltage()
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_ORM:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_ORM_INFO_ANGLE:
                                    self.orm_info_angle();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_SPEED:
                                    self.orm_info_speed();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_STATUS:
                                    self.orm_info_status();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_IR_STATUS:
                                    self.orm_info_ir_status();  
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_ORA:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_ORM_INFO_ANGLE:
                                    self.orm_info_angle();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_SPEED:
                                    self.orm_info_speed();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_STATUS:
                                    self.orm_info_status();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_IR_STATUS:
                                    self.orm_info_ir_status();       
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_OQP:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OQP_INFO_RANGE:
                                    self.oqp_info_range();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_OQP_INFO_PRESSURE:
                                    self.oqp_info_pressure();                     
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_OQP_INFO_EULER_ANGLE:
                                    self.oqp_info_euler_angle();  
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_OQP_CMD_INFO_CORR_ANGLE:  
                                    self.oqp_info_corr_angle() 
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_OQP_CMD_INFO_ANGLE_WIDTH:  
                                    self.oqp_info_angle_width()
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_OQP_CMD_INFO_DEFAULT_ANGLE:  
                                    self.oqp_info_default_angle()                                                            
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_GENERIC:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_INFO_DEV_TYPE:
                                    self.osp_info_dev_type()
                            self.input_index = 0
                    else:
                        self.input_index = 0
                #time.sleep(0.000001)
            except serial.serialutil.SerialException:
                break # Exiting while loop
        


def find_osp_peripheral(osp_dev_type):
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty.usb*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.usb*')
    else:
        raise EnvironmentError('Unsupported platform')
    
    devs = list(map(lambda port: OSP(port), ports))
    
    time.sleep(3) # Timeout to receive the first messages from the devices
    
    devs_of_type = filter(lambda d: d.get_dev_type() == osp_dev_type, devs)
    
    devs_of_wrong_type = filter(lambda d: d.get_dev_type() != osp_dev_type, devs)
    for dev in devs_of_wrong_type:
        dev.stop()
    
    return list(devs_of_type)
