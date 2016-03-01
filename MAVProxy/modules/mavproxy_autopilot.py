
import time, os, struct, math, socket, collections, threading
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting

class Autopilotmodule(mp_module.MPModule):

    def __init__(self, mpstate):
        #initialize gains
        self.Kp = 0.75
        self.Kd = 0
        self.Ki = 0
        
        super(Autopilotmodule, self).__init__(mpstate, "autopilot", "autopilot command ", public = True)
        self.add_command('autopilot', self.cmd_ap, "Autopilot input control")
        self.add_command('current_imu', self.print_imu, "prints out current IMU data", [ '' ])
        self.add_command('open_socket', self.cmd_sock, "connect to socket", ['sockno'])
        self.add_command('close_socket', self.close_sock, "close socket", ['sockno'])
        self.add_command('current_depth', self.cmd_depth, "get current object depth") 
        self.add_command('kill', self.cmd_kill, "sets rc values to 0")
        # class variables
        self.sock_option = False
        self.auto = False
        self.pwm_val = 1400
        self.pwm_max = 1800
        self.pwm_min = 1100
        self.hover_pwm_val = 1450
        self.depth = 0
        self.last_depth = collections.deque([0]*10, 10) # for summing last ten depth samples
        self.override = [ 0 ] * 16
        self.last_override = [ 0 ] * 16
        self.override_counter = 0
        self.check_imu_counter = 0
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
        self.target_altitude, self.depth = 0, 0
        self.waiting_for_command = True

        if self.sitl_output:
            self.override_period = mavutil.periodic_event(20)
        else:
            self.override_period = mavutil.periodic_event(1)
    
        self.Initialize()
                
    def idle_task(self):
        
        self.refresh_imu_data()
        if self.sock_option == True:
            if self.port is 0:
                self.cmd_sock(9999)
            self.last_depth.appendleft(self.depth)
            self.depth = self.sock.recv(14)
            try:
                self.target_altitude = int((float(self.depth)))+3000
                self.depth = int(float(self.depth))
                
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
        #self.check_imu_counter += 1
        #if self.check_imu_counter % 100 is 0:
            #print("check_imu_counter:", self.check_imu_counter)
        if self.override_period.trigger():
            if (self.override != [ 0 ] * 16 or
                self.override != self.last_override or
                self.override_counter > 0):
                self.last_override = self.override[:]
                self.send_rc_override()
                if self.override_counter > 0:
                    self.override_counter -= 1

    def cmd_kill(self, args):
        self.cmd_rc(["all", 0])

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

    def calculate_channels(self, magnitude, angle):
        pass

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

    def cmd_ap(self, args):
        self.auto = True
        average = sum(self.last_depth)/10
        print(average)
        """ Set PWM autopilot PID """
        if average <= 920 and average > 880:		
            self.cmd_rc([3, self.hover_pwm_val])
            print("Stabilizing")
        # Coptor isn't high enough
        elif average < 900:
            self.pwm_val = 1540
            self.cmd_rc([3, self.pwm_val])
            print("Throttling up")

        # Coptor is higher than we want     
        elif average > 900:
            self.pwm_val = 1400
            self.cmd_rc([3, self.pwm_val])
            print("Throttling down")
        #We're right on point       
        
            
    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr
    
    def Initialize(self):
        #initialize delta t vars
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        #term result vars
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

    def GenOut(self, error):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        self.currtm = time.time()               # get t
        self.dt = self.currtm - self.prevtm          # get delta t
        self.de = error - self.prev_err              # get delta error

        self.Cp = self.Kp * error               # proportional term
        self.Ci += error * self.dt                   # integral term

        self.Cd = 0
        if self.dt > 0:                              # no div by zero
            self.Cd = self.de/self.dt                     # derivative term

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)
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



def init(mpstate):
    '''initialise module'''
    return Autopilotmodule(mpstate)


        



