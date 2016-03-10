
import time, os, struct, math, socket, collections, threading
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from numpy.linalg import inv, det
import numpy as np
class Autopilotmodule(mp_module.MPModule):

    def __init__(self, mpstate):

        super(Autopilotmodule, self).__init__(mpstate, "autopilot", "autopilot command ", public=True)
        self.add_command('autopilot', self.cmd_ap, "Autopilot input control")
        self.add_command('current_imu', self.print_imu, "prints out current IMU data", [ '' ])
        self.add_command('open_socket', self.cmd_sock, "connect to socket", ['sockno'])
        self.add_command('close_socket', self.close_sock, "close socket", ['sockno'])
        self.add_command('current_depth', self.cmd_depth, "get current object depth") 
        self.add_command('kill', self.cmd_kill, "sets rc values to 0")

        # camera parameters
        self.cam = None
        self.most_recent = None
        self.CAPTURE_WIDTH = 640.0
        self.CAPTURE_HEIGHT = 480.0
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
        if self.auto == True:
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
            if self.pwm_val is not 1580:
                print("Throttling up")
                self.pwm_val = 1570


        # Coptor is higher than we want     
        elif average > 900:
            if self.pwm_val is not 1300:
                print("Throttling down")
                self.pwm_val = 1300

        self.cmd_rc([3, self.pwm_val])
        #self.override = [0, 0, self.pwm_val, 0,0,0,0,0,0]
        #self.send_rc_override()
        #self.track()
        #We're right on point   

    def track(self):
        # determine if target is in frame or not
        self.target_in_frame = True
        if self.target_in_frame == True:
            self.xenter = self.xcenter
            self.ycenter = self.ycenter

            ''' PID Controller for Navigation '''
            # normalize x_error and y_error
            self.x_error = (self.xcenter - self.HALF_CAPTURE_WIDTH)/self.HALF_CAPTURE_WIDTH
            self.y_error = (self.HALF_CAPTURE_HEIGHT - self.ycenter)/self.HALF_CAPTURE_HEIGHT
            print('x_error:     ', self.x_error)
            print ('y_error:     ', self.y_error)

            # compute deltas
            self.x_delta = (self.x_error - self.old_x_error)/self.dt
            self.y_delta = (self.y_error - self.old_y_error)/self.dt
            if self.delta_flag == True:    # initial deltas are undefined because dt is undefined
                self.x_delta = 0
                self.y_delta = 0
                self.delta_flag = False    # subsequent deltas are defined

            print ('x_delta:     ', self.x_delta)
            print ('y_delta:     ', self.y_delta)

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
            print('updated roll_pwm:        ', roll_pwm)
            print('updated pitch_pwm:       ', pitch_pwm)

            self.override = [roll_pwm,pitch_pwm,0,0,0,0,0,0]
            self.send_rc_override()  # start movement
            time.sleep(0.1)             # wait

            self.override = [self.ch1_trim, self.ch2_trim,0,0,0,0,0,0]
            self.send_rc_override()  # stop brake
            time.sleep(0.1)             # wait
        else:
            # release to RC radio
            #self.override = [0,0,0,0,0,0,0,0]
            #self.send_rc_override()
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

    def name():
        '''return module name'''
        return "Autopilot"

def init(mpstate):
    '''initialise module'''
    return Autopilotmodule(mpstate)


        



