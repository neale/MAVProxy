
import time, os, struct, math
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting

class Autopilotmodule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(Autopilotmodule, self).__init__(mpstate, "autopilot", "autopilot command ", public = True)
        self.override = [ 0 ] * 16
        self.last_override = [ 0 ] * 16
        self.override_counter = 0
        self.check_imu_counter = 0
        self.add_command('autopilot', self.cmd_ap, "Autopilot input control", ['< Magnitude, Angle >'])
        self.waiting_for_command = True
        if self.sitl_output:
            self.override_period = mavutil.periodic_event(20)
        else:
            self.override_period = mavutil.periodic_event(1)
        
    def idle_task(self):

        refresh_imu_data()
        check_iu_counter += 1
        if check_imu_counter % 100 is 0:
            print("check_imu_counter:", check_imu_counter)
        if self.override_period.trigger():
            if (self.override != [ 0 ] * 16 or
                self.override != self.last_override or
                self.override_counter > 0):
                self.last_override = self.override[:]
                send_rc_override()
                if self.override_counter > 0:
                    self.override_counter -= 1


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

    
    def refresh_imu_data(self):
        ''' checks for existance of imu data'''
        if 'RAW_IMU' in self.status.status.msgs:
            self.imu_raw = self.status.msgs['RAW_IMU']
            self.time_usec_raw = imu_raw.time_usec
            self.xgyro_raw = imu_raw.xgyro
            self.ygyro_raw = imu_raw.ygyro
            self.zgyro_raw = imu_raw.zgyro
            self.xmag_raw  = imu_raw.xmag
            self.ymag_raw  = imu_raw.ymag
            self.zmag_raw  = imu_raw.zmag
            self.xacc_raw  = imu_raw.xacc
            self.yacc_raw  = imu_raw.yacc
            self.zacc_raw  = imu_raw.zacc
        if 'SCALED_IMU2' in self.status.msgs:
            imu_scaled = self.status.msgs['SCALED_IMU2']
            self.time_boot_ms = imu_scaled.time_boot_ms
            self.xgyro_scaled = imu_scaled.xgyro
            self.ygyro_scaled = imu_scaled.ygyro
            self.zgyro_scaled = imu_scaled.zgyro
            self.xmag_scaled  = imu_scaled.xmag
            self.ymag_scaled  = imu_scaled.ymag
            self.zmag_scaled  = imu_scaled.zmag
            self.xacc_scaled  = imu_scaled.xacc
            self.yacc_scaled  = imu_scaled.yacc
            self.zacc_scaled  = imu_scaled.zacc

    def calculate_channels(self, magnitude, angle):
        pass

    def update_motors(self, args):
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
        self.override = newchannsels
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

    def cmd_ap(self, args):
        if self.waiting_for_command:
            set_mode('ALT_HOLD')
        else:
                set_mode("STABILIZED")

def init(mpstate):
    '''initialise module'''
    return Autopilotmodule(mpstate)


        



