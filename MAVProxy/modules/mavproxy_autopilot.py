
import time, os, struct, math, socket, collections, threading
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from numpy.linalg import inv, det
import numpy as np
class Autopilotmodule(mp_module.MPModule):

    def __init__(self, mpstate):
      
        super(Autopilotmodule, self).__init__(mpstate, "autopilot", "autopilot command ", public = True)
        self.add_command('autopilot', self.cmd_ap, "Autopilot input control")
        self.add_command('current_imu', self.print_imu, "prints out current IMU data", [ '' ])
        self.add_command('open_socket', self.cmd_sock, "connect to socket", ['sockno'])
        self.add_command('close_socket', self.close_sock, "close socket", ['sockno'])
        self.add_command('current_depth', self.cmd_depth, "get current object depth") 
        self.add_command('kill', self.cmd_kill, "sets rc values to 0")
        #self.add_command('get_radius', self_cmd_rad, "prints current distance from circle")
        
        # camera parameters
        self.cam = None
        self.most_recent = None
        self.CAPTURE_WIDTH = 628.0
        self.CAPTURE_HEIGHT = 468.0
        self.HALF_CAPTURE_WIDTH = self.CAPTURE_WIDTH/2
        self.HALF_CAPTURE_HEIGHT = self.CAPTURE_HEIGHT/2

        #create cross-correlation templates
        self.col_temp = np.ones((self.CAPTURE_HEIGHT,1), np.uint8) #ROWS by 1 array
        self.row_temp = np.ones((1,self.CAPTURE_WIDTH), np.uint8) #1 by COLS array

        # for keeping track of vision loop speed
        self.t = 0

        # classifier output
        self.target_in_frame = False
        self.last_frames = collections.deque([0]*10, 10)
        self.frame = 0

        # RC radio data trims
        self.ch1_trim = 1500
        self.ch2_trim = 1500

        # hue parameters
        self.tracking_hue = 'Color 7'
        self.hue = 7 

        # Initialize PID coefficients
        self.x_Ap = 0.7
        self.x_Ai = 0.05
        self.x_Ad = 0.6
        self.y_Ap = 0.8
        self.y_Ai = 0.05
        self.y_Ad = 0.6

        # initialize errors
        self.x_error, self.y_error = 0, 0
        self.old_x_error, self.old_y_error = 0, 0
        self.xcenter = 0
        self.ycenter = 0

        # initialize error accumulators
        self.x_sigma = collections.deque([0,0,0,0], 4)
        self.y_sigma = collections.deque([0,0,0,0], 4)

        # initialize deltas
        self.x_delta = 0
        self.y_delta = 0

        # initialize timer
        self.t = time.time()
        self.dt = 0.5

        # flag for pid first output behavior
        self.delta_flag = True
        self.sigma_flag = True

        mpstate = None

        # socket variables
        self.sock_option = False
        self.auto = False

        # set initial PWM values for movement states
        self.pwm_val = 1400
        self.hover_pwm_val = 1530

        # Depth values from realsense
        self.depth = 0
        self.last_depth = collections.deque([0]*10, 10) # for summing last ten depth samples
        self.target_altitude, self.depth = 0, 0
        self.waiting_for_command = True

        # override commands for motors
        self.override = [ 0 ] * 16
        self.last_override = [ 0 ] * 16
        self.override_counter = 0

        # consistant IMU data update
        self.check_imu_counter = 0
        
        #AHRS Filtering algorithm variables
        self.recipNorm = 0
        self.beta = 0.1
        self.sampleFreq	= 512.0		# sample frequency in Hz
        self.q0 = 1.0
        self.q1 = 0.0 
        self.q2 = 0.0 
        self.q3 = 0.0	# quaternion of sensor frame relative to auxiliary frame
        self.port = 0
        self.s0, self.s1, self.s2, self.s3 = 0, 0, 0, 0
        self.qDot1, self.qDot2, self.qDot3, self.qDot4 = 0, 0, 0, 0
        self.hx, self.hy = 0, 0
        self._2q0mx, self._2q0my, self._2q0mz, self._2q1mx, self._2bx, self._2bz, self._4bx, self._4bz, self._2q0, self._2q1, self._2q2, self._2q3, self._2q0q2, self._2q2q3, self.q0q0, self.q0q1, self.q0q2, self.q0q3, self.q1q1, self.q1q2, self.q1q3, self.q2q2, self.q2q3, self.q3q3 = 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        

        if self.sitl_output:
            self.override_period = mavutil.periodic_event(20)
        else:
            self.override_period = mavutil.periodic_event(1)
    
                
    def idle_task(self):
        
        self.refresh_imu_data()
        if self.sock_option == True:
            if self.port is 0:
                self.cmd_sock(9999)
            self.last_depth.appendleft(self.depth)
            self.depth = self.sock.recv(14)
            self.last_frames.appendleft(self.frame)
            self.frame = self.frame
            try:
                data_string = self.depth.split(',')
                self.depth   = int(data_string[0])
                self.xcenter = int(data_string[1])
                self.ycenter = int(''.join([i for i in data_string[2] if str.isdigit(i)]))
                self.depth   = int(self.depth)
                
            except ValueError:
                pass
            except socket.timeout:
                socket.close()
                self.sock_option = False
            except socket.error:

                pass
        
        if self.auto == True:
            print("test")
            self.cmd_ap("")
        if self.override_period.trigger():
            if (self.override != [ 0 ] * 16 or
                self.override != self.last_override or
                self.override_counter > 0):
                self.last_override = self.override[:]
                self.send_rc_override()
                if self.override_counter > 0:
                    self.override_counter -= 1

    def cmd_kill(self, args):
        while 1:
            self.cmd_rc(["3", 0])
            self.cmd_disarm('force')

    def cmd_disarm(self, args):
        '''disarm motors'''
        p2 = 0
        if len(args) == 1 and args[0] == 'force':
            p2 = 21196
        self.master.mav.command_long_send(
            self.target_system,  # target_system
            0,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            0, # confirmation
            0, # param1 (0 to indicate disarm)
            p2, # param2 (all other params meaningless)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def set_mode(self, args):
        '''set arbitrary mode'''
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return
        if len(args) != 1:
            print('Available modes: ', mode_mapping.keys())
            return
        if args[0].isdigit():
            modenum = int(args[0])
        else:
            mode = args[0].upper()
            if mode not in mode_mapping:
                print('Unknown mode %s: ' % mode)
                return
            modenum = mode_mapping[mode]
        self.master.set_mode(modenum)

    def cmd_depth(self, args):
        print("center depth: ", self.depth)
        if 'RC_CHANNELS_RAW' in self.status.msgs:
            self.servos = self.status.msgs['RC_CHANNELS_RAW']          
            self.throttle = self.servos.chan3_raw
            print("Throttle", self.throttle)
        print("Average Depth: ", sum(self.last_depth)/len(self.last_depth))
        print("X center: ", self.xcenter)
        print("Y_center: ", self.ycenter)
   
    def cmd_ap(self, args):
        self.auto = True
        average = sum(self.last_depth)/10
       
        """ Set PWM autopilot PID """
        if average <= 920 and average > 880:
            if self.pwm_val is not self.hover_pwm_val:
                print("Stabilizing")
                self.pwm_val = self.hover_pwm_val

        # Coptor isn't high enough

        elif average < 900:
            if self.pwm_val is not 1570:
                print("Throttling up")
                self.pwm_val = 1570
            

        # Coptor is higher than we want     
        elif average > 900:
            if self.pwm_val is not 1300:
                print("Throttling down")
                self.pwm_val = 1300
        
        #self.cmd_rc([3, self.pwm_val])
        self.override = [0, 0, self.pwm_val, 0,0,0,0,0,0]
        self.send_rc_override()
        self.track()
        #We're right on point   
            
    def track(self):
        # determine if target is in frame or not
        if self.last_frames is True:
            self.target_in_frame = True
        else:
            self.target_in_frame = False

        if self.target_in_frame == True:
            self.xenter = self.xcenter
            self.ycenter = self.ycenter
                
            ''' PID Controller for Navigation '''
            # normalize x_error and y_error
            self.x_error = (self.xcenter - self.HALF_CAPTURE_WIDTH)/self.HALF_CAPTURE_WIDTH
            self.y_error = (self.HALF_CAPTURE_HEIGHT - self.ycenter)/self.HALF_CAPTURE_HEIGHT
            print 'x_error:     ', self.x_error
            print 'y_error:     ', self.y_error

            # compute deltas
            self.x_delta = (self.x_error - self.old_x_error)/self.dt
            self.y_delta = (self.y_error - self.old_y_error)/self.dt
            if self.delta_flag == True:    # initial deltas are undefined because dt is undefined
                self.x_delta = 0
                self.y_delta = 0
                self.delta_flag = False    # subsequent deltas are defined

            print 'x_delta:     ', self.x_delta
            print 'y_delta:     ', self.y_delta

            # update accumulators
            self.x_sigma.pop()
            self.x_sigma.appendleft(self.x_error*self.dt)
            self.y_sigma.pop()
            self.y_sigma.appendleft(self.y_error*self.dt)
            if self.sigma_flag == True:    # initial sigmas are undefined because dt is undefined
                self.x_sigma = collections.deque([0,0,0,0])
                self.y_sigma = collections.deque([0,0,0,0])
                self.sigma_flag = False    # subsequent sigmas are defined
            #print 'x_sigma:     ', self.x_sigma
            #print 'y_sigma:     ', self.y_sigma

            # compute pulse
            self.x_pulse = self.x_Ap * self.x_error + self.x_Ai * sum(self.x_sigma) + self.x_Ad * self.x_delta
            self.y_pulse = self.y_Ap * self.y_error + self.y_Ai * sum(self.y_sigma) + self.y_Ad * self.y_delta 

            # motor overrides
            roll_pwm = self.ch1_trim + int(self.x_pulse*50)
            pitch_pwm = self.ch2_trim - int(self.y_pulse*50)
            print 'updated roll_pwm:        ', roll_pwm
            print 'updated pitch_pwm:       ', pitch_pwm

            self.override = [roll_pwm,pitch_pwm,0,0,0,0,0,0]
            self.send_rc_override()  # start movement
            sleep(0.3)             # wait

            self.override = [self.ch1_trim, self.ch2_trim,0,0,0,0,0,0]
            send_rc_override()  # stop brake
            sleep(0.3)             # wait
        else:
            # release to RC radio
            self.override = [0,0,0,0,0,0,0,0]
            self.send_rc_override()
            # reset pid
            self.x_sigma = collections.deque([0,0,0,0], 4)
            self.y_sigma = collections.deque([0,0,0,0], 4)
            self.old_x_error = 0
            self.old_y_error = 0
            self.dt = 0.5
            self.delta_flag = True
            self.sigma_flag = True

        self.old_x_error = self.x_error
        self.old_y_error = self.y_error
        self.dt = time.time() - self.t
        self.t = time.time()

    def refresh_imu_data(self):
        ''' checks for existance of imu data'''
        if 'RAW_IMU' in self.status.msgs:
            self.imu_raw = self.status.msgs['RAW_IMU']
            self.time_usec_raw = self.imu_raw.time_usec
            self.xgyro_raw = self.imu_raw.xgyro
            self.ygyro_raw = self.imu_raw.ygyro
            self.zgyro_raw = self.imu_raw.zgyro
            self.xmag_raw  = self.imu_raw.xmag
            self.ymag_raw  = self.imu_raw.ymag
            self.zmag_raw  = self.imu_raw.zmag
            self.xacc_raw  = self.imu_raw.xacc
            self.yacc_raw  = self.imu_raw.yacc
            self.zacc_raw  = self.imu_raw.zacc
        if 'SCALED_IMU2' in self.status.msgs:
            self.imu_scaled = self.status.msgs['SCALED_IMU2']
            self.time_boot_ms = self.imu_scaled.time_boot_ms
            self.xgyro_scaled = self.imu_scaled.xgyro
            self.ygyro_scaled = self.imu_scaled.ygyro
            self.zgyro_scaled = self.imu_scaled.zgyro
            self.xmag_scaled  = self.imu_scaled.xmag
            self.ymag_scaled  = self.imu_scaled.ymag
            self.zmag_scaled  = self.imu_scaled.zmag
            self.xacc_scaled  = self.imu_scaled.xacc
            self.yacc_scaled  = self.imu_scaled.yacc
            self.zacc_scaled  = self.imu_scaled.zacc

        """self.AHRSUpdate(self.xgyro_raw, self.ygyro_raw, self.zgyro_raw,
                        self.xacc_raw, self.yacc_raw, self.zacc_raw,
                        self.xmag_raw, self.ymag_raw, self.zmag_raw)"""
         
    
    def print_imu(self, args):
        print (
            self.xgyro_raw,
            self.ygyro_raw,
            self.zgyro_raw,
            self.xmag_raw,
            self.ymag_raw,
            self.zmag_raw,
            self.xacc_raw,
            self.yacc_raw,
            self.zacc_raw,
            self.time_boot_ms,
            self.xgyro_scaled,
            self.ygyro_scaled,
            self.zgyro_scaled,
            self.xmag_scaled,
            self.ymag_scaled,
            self.zmag_scaled,
            self.xacc_scaled,
            self.yacc_scaled,
            self.zacc_scaled)

    def cmd_rc(self, args):
        '''handle RC value override'''
        if len(args) != 2:
            print("Usage: rc <channel|all> <pwmvalue> incorrect inside autopilot")
            return
        value = int(args[1])
        if value > 65535 or value < -1:
            raise ValueError("PWM value must be a positive integer between 0 and 65535")
        if value == -1:
            value = 65535
        channels = self.override
        if args[0] == 'all':
            for i in range(16):
                channels[i] = value
        else:
            channel = int(args[0])
            if channel < 1 or channel > 16:
                print("Channel must be between 1 and 8 or 'all'")
                return
            channels[channel - 1] = value
        self.set_override(channels)

    def set_override(self, newchannels):
        '''this is a public method for use by drone API or other scripting'''
        self.override = newchannels
        self.override_counter = 10
        self.send_rc_override()

    def send_rc_override(self):
        '''send RC override packet'''
        if self.sitl_output:
            buf = struct.pack('<HHHHHHHHHHHHHHHH',
                              *self.override)
            self.sitl_output.write(buf)
        else:
            chan8 = self.override[:8]
            self.master.mav.rc_channels_override_send(self.target_system,
                                                           self.target_component,
                                                           *chan8)
    
    def cmd_sock(self, args):
        self.sock_option = True
        print("attemping connection to ", args) 

        if type(args) is list:
            try:
                if len(args) > 1:
                    print("Usage: socket <portno>")
                    return
                args = int(args[0])
                self.port = args
            except:
                print("could not convert socket, trying port 9999")
        else:	
            self.port = 9999
            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('localhost', self.port))
        except socket.error:
            print("socket not availible")
        socket.timeout(0.1)
        print("connecting to socket:", self.port)
        time.sleep(0.2)

    def close_sock(self, args):
        self.sock_option = False
        socket.close()

   

    def AHRSUpdate(self, gx, gy, gz, ax, ay, az, mx, my, mz):
    
    # Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0) and (my == 0.0) and (mz == 0.0)):
            self.IMU_update(gx, gy, gz, ax, ay, az)
            return

        # Rate of change of quaternion from gyroscope
        self.qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        self.qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        self.qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        self.qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(not((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

            # Normalise accelerometer measurement
            self.recipNorm = (ax * ax + ay * ay + az * az)**-.5
            ax *= self.recipNorm;
            ay *= self.recipNorm;
            az *= self.recipNorm;   

            # Normalise magnetometer measurement
            self.recipNorm = (mx * mx + my * my + mz * mz)**-.5
            mx *= self.recipNorm;
            my *= self.recipNorm;
            mz *= self.recipNorm;

            # Auxiliary variables to avoid repeated arithmetic
            self._2q0mx = 2.0 * self.q0 * mx;
            self._2q0my = 2.0 * self.q0 * my;
            self._2q0mz = 2.0 * self.q0 * mz;
            self._2q1mx = 2.0 * self.q1 * mx;
            self._2q0   = 2.0 * self.q0;
            self._2q1   = 2.0 * self.q1;
            self._2q2   = 2.0 * self.q2;
            self._2q3   = 2.0 * self.q3;
            self._2q0q2 = 2.0 * self.q0 * self.q2;
            self._2q2q3 = 2.0 * self.q2 * self.q3;
            self.q0q0   = self.q0 * self.q0
            self.q0q1   = self.q0 * self.q1
            self.q0q2   = self.q0 * self.q2
            self.q0q3   = self.q0 * self.q3
            self.q1q1   = self.q1 * self.q1
            self.q1q2   = self.q1 * self.q2
            self.q1q3   = self.q1 * self.q3
            self.q2q2   = self.q2 * self.q2
            self.q2q3   = self.q2 * self.q3
            self.q3q3   = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            self.hx = mx * self.q0q0 - self._2q0my * self.q3 + self._2q0mz * self.q2 + mx * self.q1q1 + self._2q1 * my * self.q2 + self._2q1 * mz * self.q3 - mx * self.q2q2 - mx * self.q3q3;
            self.hy = self._2q0mx * self.q3 + my * self.q0q0 - self._2q0mz * self.q1 + self._2q1mx * self.q2 - my * self.q1q1 + my * self.q2q2 + self._2q2 * mz * self.q3 - my * self.q3q3;
            self._2bx = (hx * hx + hy * hy)**.5;
            self._2bz = -self._2q0mx * self.q2 + self._2q0my * self.q1 + mz * self.q0q0 + self._2q1mx * self.q3 - mz * self.q1q1 + self._2q2 * my * self.q3 - mz * self.q2q2 + mz * self.q3q3;
            self._4bx = 2.0 * self._2bx
            self._4bz = 2.0 * self._2bz

            # Gradient decent algorithm corrective step
            self.s0 = -self._2q2 * (2.0 * vq1q3 - self._2q0q2 - ax) + self._2q1 * (2.0 * self.q0q1 + self._2q2q3 - ay) - self._2bz * self.q2 * (self._2bx * (0.5 - self.q2q2 - self.q3q3) + self._2bz * (self.q1q3 - self.q0q2) - mx) + (-self._2bx * self.q3 + self._2bz * self.q1) * (self._2bx * (self.q1q2 - self.q0q3) + self._2bz * (self.q0q1 + self.q2q3) - my) + self._2bx * self.q2 * (self._2bx * (self.q0q2 + self.q1q3) + self._2bz * (0.5 - self.q1q1 - self.q2q2) - mz);
            self.s1 = self._2q3 * (2.0 * self.q1q3 - self._2q0q2 - ax) + self._2q0 * (2.0 * self.q0q1 + self._2q2q3 - ay) - 4.0 * self.q1 * (1 - 2.0 * self.q1q1 - 2.0 * self.q2q2 - az) + self._2bz * q3 * (self._2bx * (0.5 - self.q2q2 - self.q3q3) + self._2bz * (self.q1q3 - self.q0q2) - mx) + (self._2bx * self.q2 + self._2bz * self.q0) * (self._2bx * (self.q1q2 - self.q0q3) + self._2bz * (self.q0q1 + self.q2q3) - my) + (self._2bx * self.q3 - self._4bz * self.q1) * (self._2bx * (self.q0q2 + self.q1q3) + self._2bz * (0.5 - self.q1q1 - self.q2q2) - mz);
            self.s2 = -self._2q0 * (2.0 * self.q1q3 - self._2q0q2 - ax) + self._2q3 * (2.0 * self.q0q1 + self._2q2q3 - ay) - 4.0 * self.q2 * (1 - 2.0 * self.q1q1 - 2.0 * self.q2q2 - az) + (-self._4bx * self.q2 - self._2bz * self.q0) * (self._2bx * (0.5 - self.q2q2 - self.q3q3) + self._2bz * (self.q1q3 - self.q0q2) - mx) + (self._2bx * self.q1 + self._2bz * self.q3) * (self._2bx * (self.q1q2 - self.q0q3) + self._2bz * (self.q0q1 + self.q2q3) - my) + (self._2bx * self.q0 - self._4bz * self.q2) * (_2bx * (self.q0q2 + self.q1q3) + self._2bz * (0.5 - self.q1q1 - self.q2q2) - mz);
            self.s3 = self._2q1 * (2.0 * self.q1q3 - self._2q0q2 - ax) + self._2q2 * (2.0 * self.q0q1 + self._2q2q3 - ay) + (-self._4bx * self.q3 + self._2bz * self.q1) * (self._2bx * (0.5 - self.q2q2 - self.q3q3) + self._2bz * (self.q1q3 - self.q0q2) - mx) + (-self._2bx * self.q0 + self._2bz * self.q2) * (self._2bx * (self.q1q2 - self.q0q3) + self._2bz * (self.q0q1 + self.q2q3) - my) + self._2bx * self.q1 * (self._2bx * (self.q0q2 + self.q1q3) + self._2bz * (0.5 - self.q1q1 - self.q2q2) - mz);
            self.recipNorm = (self.s0 * self.s0 + self.s1 * self.s1 + self.s2 * self.s2 + self.s3 * self.s3)**-.5; # normalise step magnitudeself.
            self.s0 *= self.recipNorm
            self.s1 *= self.recipNorm
            self.s2 *= self.recipNorm
            self.s3 *= self.recipNorm

            # Apply feedback step
            self.qDot1 -= self.beta * self.s0
            self.qDot2 -= self.beta * self.s1
            self.qDot3 -= self.beta * self.s2
            self.qDot4 -= self.beta * self.s3
    

        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += self.qDot1 * (1.0 / self.sampleFreq)
        self.q1 += self.qDot2 * (1.0 / self.sampleFreq)
        self.q2 += self.qDot3 * (1.0 / self.sampleFreq)
        self.q3 += self.qDot4 * (1.0 / self.sampleFreq)

        # Normalise quaternion
        self.recipNorm = (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)**-.5
        self.q0 *= self.recipNorm
        self.q1 *= self.recipNorm
        self.q2 *= self.recipNorm
        self.q3 *= self.recipNorm

    # IMU algorithm update
    def IMU_update(gx, gy, gz, ax, ay, az):
      
        # Rate of change of quaternion from gyroscope
        self.qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        self.qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        self.qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        self.qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(not((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

            # Normalise accelerometer measurement
            self.recipNorm = (ax * ax + ay * ay + az * az)**-.5
            ax *= self.recipNorm;
            ay *= self.recipNorm;
            az *= self.recipNorm;   

            # Auxiliary variables to avoid repeated arithmetic
            self._2q0 = 2.0 * self.q0;
            self._2q1 = 2.0 * self.q1;
            self._2q2 = 2.0 * self.q2;
            self._2q3 = 2.0 * self.q3;
            self._4q0 = 4.0 * self.q0;
            self._4q1 = 4.0 * self.q1;
            self._4q2 = 4.0 * self.q2;
            self._8q1 = 8.0 * self.q1;
            self._8q2 = 8.0 * self.q2;
            self.q0q0 = self.q0 * self.q0;
            self.q1q1 = self.q1 * self.q1;
            self.q2q2 = self.q2 * self.q2;
            self.q3q3 = self.q3 * self.q3;

            # Gradient decent algorithm corrective step
            self.s0 = self._4q0 * self.q2q2 + self._2q2 * ax + self._4q0 * self.q1q1 - self._2q1 * ay;
            self.s1 = self._4q1 * self.q3q3 - self._2q3 * ax + 4.0 * self.q0q0 * self.q1 - self._2q0 * ay - self._4q1 + self._8q1 * self.q1q1 + self._8q1 * self.q2q2 + self._4q1 * az;
            self.s2 = 4.0 * self.q0q0 * self.q2 + self._2q0 * ax + self._4q2 * self.q3q3 - self._2q3 * ay - self._4q2 + self._8q2 * self.q1q1 + self._8q2 * self.q2q2 + self._4q2 * az;
            self.s3 = 4.0 * self.q1q1 * self.q3 - self._2q1 * ax + 4.0 * self.q2q2 * self.q3 - self._2q2 * ay;
            self.recipNorm = (self.s0 * self.s0 + self.s1 * self.s1 + self.s2 * self.s2 + self.s3 * self.s3)**-.5 # normalise step magnitude
            self.s0 *= self.recipNorm;
            self.s1 *= self.recipNorm;
            self.s2 *= self.recipNorm;
            self.s3 *= self.recipNorm;

            # Apply feedback step
            self.qDot1 -= self.beta * self.s0
            self.qDot2 -= self.beta * self.s1
            self.qDot3 -= self.beta * self.s2
            self.qDot4 -= self.beta * self.s3

        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += self.qDot1 * (1.0 / self.sampleFreq)
        self.q1 += self.qDot2 * (1.0 / self.sampleFreq)
        self.q2 += self.qDot3 * (1.0 / self.sampleFreq)
        self.q3 += self.qDot4 * (1.0 / self.sampleFreq)

        # Normalise quaternion
        self.recipNorm = (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)**-0.5
        self.q0 *= self.recipNorm
        self.q1 *= self.recipNorm
        self.q2 *= self.recipNorm
        self.q3 *= self.recipNorm

    def name():
        '''return module name'''
        return "Autopilot"

def init(mpstate):
    '''initialise module'''
    return Autopilotmodule(mpstate)


        



