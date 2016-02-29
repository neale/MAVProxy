
import time, os, struct, math
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting


class SetRefModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(SetRefModule, self).__init__(mpstate, "setref", "Sets IMU 0 reference point", public = True)
        self.override = [ 0 ] * 16
        self.last_override = [ 0 ] * 16
        self.override_counter = 0
        self.add_command('setref', self.cmd_ref, "Set IMU reference point", [''])
        self.waiting_for_command = True
        if self.sitl_output:
            self.override_period = mavutil.periodic_event(20)
        else:
            self.override_period = mavutil.periodic_event(1)

    def cmd_ref(self, args):
        print("Status: ", self.status.msgs)
        print("Setting reference IMU points")
        ''' checks for existance of imu data'''
        if 'RAW_IMU' in self.status.msgs:
            self.imu_raw_ref = self.status.msgs['RAW_IMU']
            self.time_usec_raw_ref = imu_raw_ref.time_usec
            self.xgyro_raw_ref = imu_raw_ref.xgyro
            self.ygyro_raw_ref = imu_raw_ref.ygyro
            self.zgyro_raw_ref = imu_raw_ref.zgyro
            self.xmag_raw_ref  = imu_raw_ref.xmag
            self.ymag_raw_ref  = imu_raw_ref.ymag
            self.zmag_raw_ref  = imu_raw_ref.zmag
            self.xacc_raw_ref  = imu_raw_ref.xacc
            self.yacc_raw_ref  = imu_raw_ref.yacc
            self.zacc_raw_ref  = imu_raw_ref.zacc
        if 'SCALED_IMU2' in self.status.msgs:
            imu_scaled = self.status.msgs['SCALED_IMU2']
            self.time_boot_ms_ref = imu_scaled.time_boot_ms
            self.xgyro_scaled_ref = imu_scaled.xgyro
            self.ygyro_scaled_ref = imu_scaled.ygyro
            self.zgyro_scaled_ref = imu_scaled.zgyro
            self.xmag_scaled_ref  = imu_scaled.xmag
            self.ymag_scaled_ref  = imu_scaled.ymag
            self.zmag_scaled_ref  = imu_scaled.zmag 
            self.xacc_scaled_ref  = imu_scaled.xacc
            self.yacc_scaled_ref  = imu_scaled.yacc
            self.zacc_scaled_ref  = imu_scaled.zacc

        print( "xgyro:", self.xgyro_scaled_ref,'\n',
            "ygyro:", self.ygyro_scaled_ref,'\n',
            "zg yro:", self.zgyro_scaled_ref,'\n',
            "xmag:", self.xmag_scaled_ref,'\n',
            "ymag:", self.ymag_scaled_ref,'\n',
            "zmag:", self.zmag_scaled_ref,'\n',
            "xacc:", self.xacc_scaled_ref,'\n',
            "yacc:", self.yacc_scaled_ref,'\n',
            "zacc:", self.zacc_scaled_ref,'\n',)


def init(mpstate):
    '''initialise module'''
    return SetRefModule(mpstate)


        
        