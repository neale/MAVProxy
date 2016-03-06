#!/usr/bin/env python
'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

'''

import sys, os, struct, math, time, socket, collections
import fnmatch, errno, threading
import serial, Queue, select

# The modules subdirectory contains the modules that come with
# mavproxy.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules'))

# find the mavlink.py module
for d in [ 'pymavlink',
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'pymavlink'),
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'mavlink', 'pymavlink') ]:
    if os.path.exists(d):
        sys.path.insert(0, d)

import select
from modules.lib import textconsole
from modules.lib import mp_settings
from modules.lib import mp_module
from modules import mavproxy_autopilot as autopilot

class MPSettings(object):
    def __init__(self):
        self.vars = [ ('link', int),
                      ('altreadout', int),
                      ('distreadout', int),
                      ('battreadout', int),
                      ('heartbeat', int),
                      ('numcells', int),
                      ('speech', int),
                      ('mavfwd', int),
                      ('mavfwd_rate', int),
                      ('streamrate', int),
                      ('streamrate2', int),
                      ('heartbeatreport', int),
                      ('radiosetup', int),
                      ('paramretry', int),
                      ('moddebug', int),
                      ('rc1mul', int),
                      ('rc2mul', int),
                      ('rc4mul', int)]
        self.link = 1
        self.altreadout = 10
        self.distreadout = 200
        self.battreadout = 0
        self.basealtitude = -1
        self.heartbeat = 1
        self.numcells = 0
        self.mavfwd = 1
        self.mavfwd_rate = 0
        self.speech = 0
        self.streamrate = 4
        self.streamrate2 = 4
        self.radiosetup = 0
        self.heartbeatreport = 1
        self.paramretry = 10
        self.rc1mul = 1
        self.rc2mul = 1
        self.rc4mul = 1
        self.moddebug = 0

    def set(self, vname, value):
        '''set a setting'''
        for (v,t) in sorted(self.vars):
            if v == vname:
                try:
                    value = t(value)
                except:
                    print("Unable to convert %s to type %s" % (value, t))
                    return
                setattr(self, vname, value)
                return  

    def show(self, v):
        '''show settings'''
        print("%20s %s" % (v, getattr(self, v)))

    def show_all(self):
        '''show all settings'''
        for (v,t) in sorted(self.vars):
            self.show(v)
class MPStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        if opts.quadcopter:
            self.rc_throttle = [ 0.0, 0.0, 0.0, 0.0 ]
        self.gps = None
        self.msgs = {}
        self.msg_count = {}
        self.setup_mode = opts.setup
        self.mav_error = 0
        self.exit = False
        self.flightmode = 'MAV'
        self.last_mode_announce = 0
        self.logdir = None
        self.last_heartbeat = 0
        self.last_message = 0
        self.heartbeat_error = False
        self.last_apm_msg = None
        self.last_apm_msg_time = 0
        self.highest_msec = 0
        self.have_gps_lock = False
        self.lost_gps_lock = False
        self.last_gps_lock = 0
        self.watch = None
        self.last_streamrate1 = -1
        self.last_streamrate2 = -1
        self.last_seq = 0
        self.armed = False
        self.x_center, self.last_x_center = 0, 0
        self.y_center, self.last_y_center = 0, 0
        self.current_depth = 0
        self.depth_stream = collections.deque([0]*5, 5)
        self.counters = {'MasterIn' : [], 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.wp_op = None
        self.wp_save_filename = None
        self.wploader = mavwp.MAVWPLoader()
        self.fenceloader = mavwp.MAVFenceLoader()
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.target_system = 1
        self.target_component = 1
        self.speech = None
        self.last_waypoint = 0
        self.exit = False
        self.override = [ 0 ] * 8
        self.last_override = [ 0 ] * 8
        self.override_counter = 0
        self.last_heartbeat = 0
        self.fence_enabled = False
        self.last_fence_breach = 0
        self.last_fence_status = 0


    def show(self, f, pattern=None):
        '''write status to status.txt'''
        if pattern is None:
            f.write('Counters: ')
            for c in self.counters:
                f.write('%s:%s ' % (c, self.counters[c]))
            f.write('\n')
            f.write('MAV Errors: %u\n' % self.mav_error)
            f.write(str(self.gps)+'\n')
        for m in sorted(self.msgs.keys()):
            if pattern is not None and not fnmatch.fnmatch(str(m).upper(), pattern.upper()):
                continue
            f.write("%u: %s\n" % (self.msg_count[m], str(self.msgs[m])))

    def write(self):
        '''write status to status.txt'''
        f = open('status.txt', mode='w')
        self.show(f)
        f.close()


def say(text, priority='important'):
    '''text and/or speech output'''
    mpstate.functions.say(text, priority)

class MAVFunctions(object):
    pass

class MPState(object):
    '''holds state of mavproxy'''
    def __init__(self):
        self.console = textconsole.SimpleConsole()
        self.map = None
        self.settings = mp_settings.MPSettings(
             [ ('link', int, 1),
              ('altreadout', int, 10),
              ('distreadout', int, 200),
              ('battreadout', int, 0),
              ('heartbeat', int, 1),
              ('numcells', int, 1),
              ('speech', int, 0),
              ('mavfwd', int, 1),
              ('mavfwd_rate', int, 0),
              ('streamrate', int, 4),
              ('streamrate2', int, 4),
              ('heartbeatreport', int, 1),
              ('moddebug', int, 0),
              ('rc1mul', int, 1),
              ('rc2mul', int, 1),
              ('rc4mul', int, 1),
              ('shownoise', int, 1),
              ('basealt', int, 0)]
            )
        self.completions = {
            "script"         : ["(FILENAME)"],
            "set"            : ["(SETTING)"],
            "status"         : ["(VARIABLE)"],
            "module"    : ["list",
                           "load (AVAILMODULES)",
                           "<unload|reload> (LOADEDMODULES)"]
        }
        self.status = MPStatus()
        # master mavlink device
        self.mav_master = None
        # mavlink outputs
        self.mav_outputs = []
        # SITL output
        self.sitl_output = None

        self.mav_param = mavparm.MAVParmDict()
        self.modules = []
        self.mav_param_count = 0
        self.mav_param_set = set()
        self.functions = MAVFunctions()
        self.functions.say = say
        self.functions.process_stdin = process_stdin
        self.select_extra = {}
        self.continue_mode = False
        self.aliases = {}
    
    def master(self):
        '''return the currently chosen mavlink master object'''

        if self.settings.link > len(self.mav_master):
            self.settings.link = 1

        # try to use one with no link error
        if not self.mav_master[self.settings.link-1].linkerror:
            return self.mav_master[self.settings.link-1]
        for m in self.mav_master:
            if not m.linkerror:
                return m
        return self.mav_master[self.settings.link-1]



def get_usec():
    '''time since 1970 in microseconds'''
    return int(time.time() * 1.0e6)

class rline(object):
    '''async readline abstraction'''
    def __init__(self, prompt):
        import threading
        self.prompt = prompt
        self.line = None
        try:
            import readline
        except Exception:
            pass

    def set_prompt(self, prompt):
        if prompt != self.prompt:
            self.prompt = prompt
            sys.stdout.write(prompt)

def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    return mpstate.mav_param.get(param, default)

def cmd_script(args):
    '''run a script'''
    if len(args) < 1:
        print("usage: script <filename>")
        return

    run_script(args[0])

def import_package(name):
   
    mod = __import__(name)   
    components = name.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

def cmd_link(args):
    for master in mpstate.mav_master:
        linkdelay = (mpstate.status.highest_msec - master.highest_msec)*1.0e-3
        if master.linkerror:
            print("link %u down" % (master.linknum+1))
        else:
            print("link %u OK (%u packets, %.2fs delay, %u lost, %.1f%% loss)" % (master.linknum+1,
                                                                                  mpstate.status.counters['MasterIn'][master.linknum],
                                                                                  linkdelay,
                                                                                  master.mav_loss,
                                                                                  master.packet_loss()))
def cmd_switch(args):
    '''handle RC switch changes'''
    mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
    if len(args) != 1:
        print("Usage: switch <pwmvalue>")
        return
    value = int(args[0])
    if value < 0 or value > 6:
        print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
        return
    if opts.quadcopter:
        default_channel = 5
    else:
        default_channel = 8
    flite_mode_ch_parm = int(get_mav_param("FLTMODE_CH", default_channel))
    mpstate.status.override[flite_mode_ch_parm-1] = mapping[value]
    mpstate.status.override_counter = 10
    send_rc_override()
    if value == 0:
        print("Disabled RC switch override")
    else:
        print("Set RC switch override to %u (PWM=%u channel=%u)" % (
            value, mapping[value], flite_mode_ch_parm))

def cmd_rc(args):
    '''handle RC value override'''
    if len(args) != 2:
        print("Usage: rc <channel|all> <pwmvalue>")
        return
    value   = int(args[1])
    if value == -1:
        value = 65535
    if args[0] == 'all':
        for i in range(8):
            mpstate.status.override[i] = value
    else:
        channel = int(args[0])
        mpstate.status.override[channel-1] = value
        if channel < 1 or channel > 8:
            print("Channel must be between 1 and 8 or 'all'")
            return
    mpstate.status.override_counter = 10
    send_rc_override()

def send_rc_override():
    '''send RC override packet'''
    if mpstate.sitl_output:
        buf = struct.pack('<HHHHHHHH',
                          *mpstate.status.override)
        mpstate.sitl_output.write(buf)
    else:
        mpstate.master().mav.rc_channels_override_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         *mpstate.status.override)
def cmd_param(args):
    '''control parameters'''
    param_wildcard = "*"
    if len(args) < 1:
        print("usage: param <fetch|edit|set|show|diff>")
        return
    if args[0] == "fetch":
        if len(args) == 1:
            mpstate.master().param_fetch_all()
            mpstate.mav_param_set = set()
            print("Requested parameter list")
        else:
            mpstate.master().param_fetch_one(args[1].upper())
            mpstate.status.fetch_one = True
            print("Requested parameter %s" % args[1])
    elif args[0] == "save":
        if len(args) < 2:
            print("usage: param save <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        mpstate.mav_param.save(args[1], param_wildcard, verbose=True)
    elif args[0] == "diff":
        if len(args) < 2:
            if opts.aircraft is not None:
                filename = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(mpstate.status.logdir))), 'mavdefault.txt')
            else:
                print("Usage: param diff <filename>")
        else:
            filename = args[1]
        if len(args) == 3:
            wildcard = args[2]
        else:
            wildcard = '*'
        mpstate.mav_param.diff(filename, wildcard=wildcard)
    elif args[0] == "set":
        if len(args) != 3:
            print("Usage: param set PARMNAME VALUE")
            return
        param = args[1]
        value = args[2]
        if not param.upper() in mpstate.mav_param:
            print("Unable to find parameter '%s'" % param)
            return
        param_set(param, value)
    elif args[0] == "load":
        if len(args) < 2:
            print("Usage: param load <filename> [wildcard]")
            return
        if len(args) > 2:
            param_wildcard = args[2]
        else:
            param_wildcard = "*"
        mpstate.mav_param.load(args[1], param_wildcard, mpstate.master())
    elif args[0] == "show":
        if len(args) > 1:
            pattern = args[1]
        else:
            pattern = "*"
        mpstate.mav_param.show(pattern)
    else:
        print("Unknown subcommand '%s' (try 'fetch', 'save', 'set', 'show', 'load')" % args[0])

def cmd_loiter(args):
    '''set LOITER mode'''
    mpstate.master().set_mode_loiter()

def cmd_auto(args):
    '''set AUTO mode'''
    mpstate.master().set_mode_auto()

def cmd_ground(args):
    '''do a ground start mode'''
    mpstate.master().calibrate_imu()

def cmd_level(args):
    '''run a accel level'''
    mpstate.master().calibrate_level()

def cmd_accelcal(args):
    '''do a full 3D accel calibration'''
    mav = mpstate.master()
    # ack the APM to begin 3D calibration of accelerometers
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                              0, 0, 0, 0, 1, 0, 0)
    count = 0
    # we expect 6 messages and acks
    while count < 6:
        m = mav.recv_match(type='STATUSTEXT', blocking=True)
        text = str(m.text)
        if not text.startswith('Place APM'):
            continue
        # wait for user to hit enter
        mpstate.rl.line = None
        while mpstate.rl.line is None:
            time.sleep(0.1)
        mpstate.rl.line = None
        count += 1
        # tell the APM that we've done as requested
        mav.mav.command_ack_send(count, 1)

def cmd_reboot(args):
    '''reboot autopilot'''
    mpstate.master().reboot_autopilot()

def cmd_calpressure(args):
    '''calibrate pressure sensors'''
    mpstate.master().calibrate_pressure()

def cmd_rtl(args):
    '''set RTL mode'''
    mpstate.master().set_mode_rtl()

def cmd_manual(args):
    '''set MANUAL mode'''
    mpstate.master().set_mode_manual()

def cmd_servo(args):
    '''set a servo'''
    if len(args) != 2:
        print("Usage: servo <channel> <pwmvalue>")
        return
    channel = int(args[0])
    value   = int(args[1])
    mpstate.master().set_servo(channel, value)

def cmd_fbwa(args):
    '''set FBWA mode'''
    mpstate.master().set_mode_fbwa()

def process_waypoint_request(m, master):
    '''process a waypoint request from the master'''
    if (not mpstate.status.loading_waypoints or
        time.time() > mpstate.status.loading_waypoint_lasttime + 10.0):
        mpstate.status.loading_waypoints = False
        mpstate.console.error("not loading waypoints")
        return
    if m.seq >= mpstate.status.wploader.count():
        mpstate.console.error("Request for bad waypoint %u (max %u)" % (m.seq, mpstate.status.wploader.count()))
        return
    wp = mpstate.status.wploader.wp(m.seq)
    wp.target_system = mpstate.status.target_system
    wp.target_component = mpstate.status.target_component
    master.mav.send(mpstate.status.wploader.wp(m.seq))
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.console.writeln("Sent waypoint %u : %s" % (m.seq, mpstate.status.wploader.wp(m.seq)))
    if m.seq == mpstate.status.wploader.count() - 1:
        mpstate.status.loading_waypoints = False
        mpstate.console.writeln("Sent all %u waypoints" % mpstate.status.wploader.count())

def load_waypoints(filename):
    '''load waypoints from a file'''
    mpstate.status.wploader.target_system = mpstate.status.target_system
    mpstate.status.wploader.target_component = mpstate.status.target_component
    try:
        mpstate.status.wploader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u waypoints from %s" % (mpstate.status.wploader.count(), filename))

    mpstate.master().waypoint_clear_all_send()
    if mpstate.status.wploader.count() == 0:
        return

    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    mpstate.master().waypoint_count_send(mpstate.status.wploader.count())


def update_waypoints(filename, wpnum):
    '''update waypoints from a file'''
    mpstate.status.wploader.target_system = mpstate.status.target_system
    mpstate.status.wploader.target_component = mpstate.status.target_component
    try:
        mpstate.status.wploader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    if mpstate.status.wploader.count() == 0:
        print("No waypoints found in %s" % filename)
        return
    if wpnum == -1:
        print("Loaded %u updated waypoints from %s" % (mpstate.status.wploader.count(), filename))
    elif wpnum >= mpstate.status.wploader.count():
        print("Invalid waypoint number %u" % wpnum)
        return
    else:
        print("Loaded updated waypoint %u from %s" % (wpnum, filename))

    mpstate.status.loading_waypoints = True
    mpstate.status.loading_waypoint_lasttime = time.time()
    if wpnum == -1:
        start = 0
        end = mpstate.status.wploader.count()-1
    else:
        start = wpnum
        end = wpnum
    mpstate.master().mav.mission_write_partial_list_send(mpstate.status.target_system,
                                                         mpstate.status.target_component,
                                                         start, end)

def save_waypoints(filename):
    '''save waypoints to a file'''
    try:
        mpstate.status.wploader.save(filename)
    except Exception, msg:
        print("Failed to save %s - %s" % (filename, msg))
        return
    print("Saved %u waypoints to %s" % (mpstate.status.wploader.count(), filename))


def cmd_wp(args):
    '''waypoint commands'''
    if len(args) < 1:
        print("usage: wp <list|load|save|set|clear>")
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: wp load <filename>")
            return
        load_waypoints(args[1])
    elif args[0] == "update":
        if len(args) < 2:
            print("usage: wp update <filename> <wpnum>")
            return
        if len(args) == 3:
            wpnum = int(args[2])
        else:
            wpnum = -1
        update_waypoints(args[1], wpnum)
    elif args[0] == "list":
        mpstate.status.wp_op = "list"
        mpstate.master().waypoint_request_list_send()
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: wp save <filename>")
            return
        mpstate.status.wp_save_filename = args[1]
        mpstate.status.wp_op = "save"
        mpstate.master().waypoint_request_list_send()
    elif args[0] == "show":
        if len(args) != 2:
            print("usage: wp show <filename>")
            return
        mpstate.status.wploader.load(args[1])
    elif args[0] == "set":
        if len(args) != 2:
            print("usage: wp set <wpindex>")
            return
        mpstate.master().waypoint_set_current_send(int(args[1]))
    elif args[0] == "clear":
        mpstate.master().waypoint_clear_all_send()
    else:
        print("Usage: wp <list|load|save|set|show|clear>")


def fetch_fence_point(i):
    '''fetch one fence point'''
    mpstate.master().mav.fence_fetch_point_send(mpstate.status.target_system,
                                                mpstate.status.target_component, i)
    tstart = time.time()
    p = None
    while time.time() - tstart < 1:
        p = mpstate.master().recv_match(type='FENCE_POINT', blocking=False)
        if p is not None:
            break
        time.sleep(0.1)
        continue
    if p is None:
        mpstate.console.error("Failed to fetch point %u" % i)
        return None
    return p



def load_fence(filename):
    '''load fence points from a file'''
    try:
        mpstate.status.fenceloader.target_system = mpstate.status.target_system
        mpstate.status.fenceloader.target_component = mpstate.status.target_component
        mpstate.status.fenceloader.load(filename)
    except Exception, msg:
        print("Unable to load %s - %s" % (filename, msg))
        return
    print("Loaded %u geo-fence points from %s" % (mpstate.status.fenceloader.count(), filename))

    # must disable geo-fencing when loading
    action = get_mav_param('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
    param_set('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
    param_set('FENCE_TOTAL', mpstate.status.fenceloader.count())
    for i in range(mpstate.status.fenceloader.count()):
        p = mpstate.status.fenceloader.point(i)
        mpstate.master().mav.send(p)
        p2 = fetch_fence_point(i)
        if p2 is None:
            param_set('FENCE_ACTION', action)
            return
        if (p.idx != p2.idx or
            abs(p.lat - p2.lat) >= 0.00003 or
            abs(p.lng - p2.lng) >= 0.00003):
            print("Failed to send fence point %u" % i)
            param_set('FENCE_ACTION', action)
            return
    param_set('FENCE_ACTION', action)


def list_fence(filename):
    '''list fence points, optionally saving to a file'''

    mpstate.status.fenceloader.clear()
    count = get_mav_param('FENCE_TOTAL', 0)
    if count == 0:
        print("No geo-fence points")
        return
    for i in range(int(count)):
        p = fetch_fence_point(i)
        if p is None:
            return
        mpstate.status.fenceloader.add(p)

    if filename is not None:
        try:
            mpstate.status.fenceloader.save(filename)
        except Exception, msg:
            print("Unable to save %s - %s" % (filename, msg))
            return
        print("Saved %u geo-fence points to %s" % (mpstate.status.fenceloader.count(), filename))
    else:
        for i in range(mpstate.status.fenceloader.count()):
            p = mpstate.status.fenceloader.point(i)
            mpstate.console.writeln("lat=%f lng=%f" % (p.lat, p.lng))
    if mpstate.status.logdir != None:
        fencetxt = os.path.join(mpstate.status.logdir, 'fence.txt')
        mpstate.status.fenceloader.save(fencetxt)
        print("Saved fence to %s" % fencetxt)

def cmd_fence(args):
    '''geo-fence commands'''
    if len(args) < 1:
        print("usage: fence <list|load|save|clear>")
        return

    if args[0] == "load":
        if len(args) != 2:
            print("usage: fence load <filename>")
            return
        load_fence(args[1])
    elif args[0] == "list":
        list_fence(None)
    elif args[0] == "save":
        if len(args) != 2:
            print("usage: fence save <filename>")
            return
        list_fence(args[1])
    elif args[0] == "show":
        if len(args) != 2:
            print("usage: fence show <filename>")
            return
        mpstate.status.fenceloader.load(args[1])
    elif args[0] == "clear":
        param_set('FENCE_TOTAL', 0)
    else:
        print("Usage: fence <list|load|save|show|clear>")

def param_set(name, value, retries=3):
    '''set a parameter'''
    name = name.upper()
    return mpstate.mav_param.mavset(mpstate.master(), name, value, retries=retries)

def cmd_set(args):
    '''control mavproxy options'''
    if len(args) == 0:
        mpstate.settings.show_all()
        return

    if getattr(mpstate.settings, args[0], None) is None:
        print("Unknown setting '%s'" % args[0])
        return
    if len(args) == 1:
        mpstate.settings.show(args[0])
    else:
        mpstate.settings.set(args[0], args[1])

def cmd_status(args):
    '''show status'''
    if len(args) == 0:
        mpstate.status.show(sys.stdout, pattern=None)
    else:
        for pattern in args:
            mpstate.status.show(sys.stdout, pattern=pattern)

def cmd_bat(args):
    '''show battery levels'''
    print("Flight battery:   %u%%" % mpstate.status.battery_level)
    print("Avionics battery: %u%%" % mpstate.status.avionics_battery_level)

def cmd_alt(args):
    '''show altitude'''
    print("Altitude:  %.1f" % mpstate.status.altitude)


def cmd_up(args):
    '''adjust TRIM_PITCH_CD up by 5 degrees'''
    if len(args) == 0:
        adjust = 5.0
    else:
        adjust = float(args[0])
    old_trim = get_mav_param('TRIM_PITCH_CD', None)
    if old_trim is None:
        print("Existing trim value unknown!")
        return
    new_trim = int(old_trim + (adjust*100))
    if math.fabs(new_trim - old_trim) > 1000:
        print("Adjustment by %d too large (from %d to %d)" % (adjust*100, old_trim, new_trim))
        return
    print("Adjusting TRIM_PITCH_CD from %d to %d" % (old_trim, new_trim))
    param_set('TRIM_PITCH_CD', new_trim)


def cmd_setup(args):
    mpstate.status.setup_mode = True
    mpstate.rl.set_prompt("")


def cmd_reset(args):
    print("Resetting master")
    mpstate.master().reset()

def cmd_link(args):
    for master in mpstate.mav_master:
        linkdelay = (mpstate.status.highest_msec - master.highest_msec)*1.0e-3
        if master.linkerror:
            print("link %u down" % (master.linknum+1))
        else:
            print("link %u OK (%u packets, %.2fs delay, %u lost, %.1f%% loss)" % (master.linknum+1,
                                                                                  mpstate.status.counters['MasterIn'][master.linknum],
                                                                                  linkdelay,
                                                                                  master.mav_loss,
                                                                                  master.packet_loss()))
def cmd_watch(args):
    '''watch a mavlink packet pattern'''
    if len(args) == 0:
        mpstate.status.watch = None
        return
    mpstate.status.watch = args[0]
    print("Watching %s" % mpstate.status.watch)

def cmd_module(args):
    '''module commands'''
    usage = "usage: module <list|load|reload|unload>"
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "list":
        for m in mpstate.modules:
            print("%s: %s" % (m.name(), m.description()))
    elif args[0] == "load":
        if len(args) < 2:
            print("usage: module load <name>")
            return
        modname = args[1]
        modpath = 'mavproxy_%s' % (modname,)
        try:
            m = import_package(modpath)
            if m in mpstate.modules:
                raise RuntimeError("module %s already loaded" % (modname,))
            m.init(mpstate)
            mpstate.modules.append(m)
            print("Loaded module %s" % (modname,))
        except Exception, msg:
            print("Unable to load module %s: %s" % (modname, msg))
    elif args[0] == "reload":
        if len(args) < 2:
            print("usage: module reload <name>")
            return
        modname = args[1]
        for m in mpstate.modules:
            if m.name() == modname:
                try:
                    m.unload()
                except Exception:
                    pass
                reload(m)
                m.init(mpstate)
                print("Reloaded module %s" % modname)
                return
        print("Unable to find module %s" % modname)
    elif args[0] == "unload":
        if len(args) < 2:
            print("usage: module unload <name>")
            return
        modname = os.path.basename(args[1])
        for m in mpstate.modules:
            if m.name() == modname:
                m.unload()
                mpstate.modules.remove(m)
                print("Unloaded module %s" % modname)
                return
        print("Unable to find module %s" % modname)
    else:
        print(usage)

def cmd_alias(args):
    '''alias commands'''
    usage = "usage: alias <add|remove|list>"
    if len(args) < 1 or args[0] == "list":
        if len(args) >= 2:
            wildcard = args[1].upper()
        else:
            wildcard = '*'
        for a in sorted(mpstate.aliases.keys()):
            if fnmatch.fnmatch(a.upper(), wildcard):
                print("%-15s : %s" % (a, mpstate.aliases[a]))
    elif args[0] == "add":
        if len(args) < 3:
            print(usage)
            return
        a = args[1]
        mpstate.aliases[a] = ' '.join(args[2:])
    elif args[0] == "remove":
        if len(args) != 2:
            print(usage)
            return
        a = args[1]
        if a in mpstate.aliases:
            mpstate.aliases.pop(a)
        else:
            print("no alias %s" % a)
    else:
        print(usage)
        return


def cmd_arm(args):
  '''arm motors'''
  mpstate.master().arducopter_arm()


def cmd_disarm(args):
  '''disarm motors'''
  mpstate.master().arducopter_disarm()
command_map = {
    'switch'  : (cmd_switch,   'set RC switch (1-5), 0 disables'),
    'rc'      : (cmd_rc,       'override a RC channel value'),
    'wp'      : (cmd_wp,       'waypoint management'),
    'fence'   : (cmd_fence,    'geo-fence management'),
    'param'   : (cmd_param,    'manage APM parameters'),
    'setup'   : (cmd_setup,    'go into setup mode'),
    'reset'   : (cmd_reset,    'reopen the connection to the MAVLink master'),
    'status'  : (cmd_status,   'show status'),
    'auto'    : (cmd_auto,     'set AUTO mode'),
    'ground'  : (cmd_ground,   'do a ground start'),
    'level'   : (cmd_level,    'set level on a multicopter'),
    'accelcal': (cmd_accelcal, 'do 3D accelerometer calibration'),
    'calpress': (cmd_calpressure,'calibrate pressure sensors'),
    'loiter'  : (cmd_loiter,   'set LOITER mode'),
    'rtl'     : (cmd_rtl,      'set RTL mode'),
    'manual'  : (cmd_manual,   'set MANUAL mode'),
    'fbwa'    : (cmd_fbwa,     'set FBWA mode'),
    'set'     : (cmd_set,      'mavproxy settings'),
    'bat'     : (cmd_bat,      'show battery levels'),
    'alt'     : (cmd_alt,      'show relative altitude'),
    'link'    : (cmd_link,     'show link status'),
    'servo'   : (cmd_servo,    'set a servo value'),
    'reboot'  : (cmd_reboot,   'reboot the autopilot'),
    'up'      : (cmd_up,       'adjust TRIM_PITCH_CD up by 5 degrees'),
    'watch'   : (cmd_watch,    'watch a MAVLink pattern'),
    'module'  : (cmd_module,   'module commands'),
    'alias'   : (cmd_alias,    'command aliases'),
    'arm'     : (cmd_arm,      'ArduCopter arm motors'),
    'disarm'  : (cmd_disarm,   'ArduCopter disarm motors')
    }




def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)
    
    line = line.strip()

    if mpstate.status.setup_mode:
        # in setup mode we send strings straight to the master
        if line == '.':
            mpstate.status.setup_mode = False
            mpstate.status.flightmode = "MAV"
            mpstate.rl.set_prompt("MAV> ")
            return
        if line == '+++':
            mpstate.master().write(line)
        else:
            mpstate.master().write(line + '\r')
        return

    if not line:
        return

    args = line.split()
    cmd = args[0]
    while cmd in mpstate.aliases:
        line = mpstate.aliases[cmd]
        args = line.split() + args[1:]
        cmd = args[0]
        
    if cmd == 'help':
        k = command_map.keys()
        k.sort()
        for cmd in k:
            (fn, help) = command_map[cmd]
            print("%-15s : %s" % (cmd, help))
        return
    if cmd == 'exit' and mpstate.settings.requireexit:
        mpstate.status.exit = True
        return

    if not cmd in command_map:
        for (m,pm) in mpstate.modules:
            if hasattr(m, 'unknown_command'):
                try:
                    if m.unknown_command(args):
                        return
                except Exception as e:
                    print("ERROR in command: %s" % str(e))
        print("Unknown command '%s'" % line)
        return
    (fn, help) = command_map[cmd]
    try:
        fn(args[1:])
    except Exception as e:
        print("ERROR in command %s: %s" % (args[1:], str(e)))
        if mpstate.settings.moddebug > 1:
            traceback.print_exc()


def process_master(m):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        time.sleep(0.1)
        return
    # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
    # cause it to try and reconnect
    if len(s) == 0:
        time.sleep(0.1)
        return

    if (mpstate.settings.compdebug & 1) != 0:
        return

    if mpstate.logqueue_raw:
        mpstate.logqueue_raw.put(str(s))

    if mpstate.status.setup_mode:
        if mpstate.system == 'Windows':
           # strip nsh ansi codes
           s = s.replace("\033[K","")
        sys.stdout.write(str(s))
        sys.stdout.flush()
        return

    if m.first_byte and opts.auto_protocol:
        m.auto_mavlink_version(s)
    msgs = m.mav.parse_buffer(s)
    if msgs:
        for msg in msgs:
            sysid = msg.get_srcSystem()
            if sysid in mpstate.sysid_outputs:
                  # the message has been handled by a specialised handler for this system
                  continue
            if getattr(m, '_timestamp', None) is None:
                m.post_message(msg)
            if msg.get_type() == "BAD_DATA":
                if opts.show_errors:
                    mpstate.console.writeln("MAV error: %s" % msg)
                mpstate.status.mav_error += 1



def process_mavlink(slave):
    '''process packets from MAVLink slaves, forwarding to the master'''
    try:
        buf = slave.recv()
    except socket.error:
        return
    try:
        if slave.first_byte and opts.auto_protocol:
            slave.auto_mavlink_version(buf)
        msgs = slave.mav.parse_buffer(buf)
    except mavutil.mavlink.MAVError as e:
        mpstate.console.error("Bad MAVLink slave message from %s: %s" % (slave.address, e.message))
        return
    if msgs is None:
        return
    if mpstate.settings.mavfwd and not mpstate.status.setup_mode:
        for m in msgs:
            if mpstate.status.watch is not None:
                if fnmatch.fnmatch(m.get_type().upper(), mpstate.status.watch.upper()):
                    mpstate.console.writeln('> '+ str(m))
            mpstate.master().write(m.get_msgbuf())
    mpstate.status.counters['Slave'] += 1


def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def log_writer():
    '''log writing thread'''
    while True:
        mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue_raw.empty():
            mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue.empty():
            mpstate.logfile.write(mpstate.logqueue.get())
        if mpstate.settings.flushlogs:
            mpstate.logfile.flush()
            mpstate.logfile_raw.flush()

# If state_basedir is NOT set then paths for logs and aircraft
# directories are relative to mavproxy's cwd
def log_paths():
    '''Returns tuple (logdir, telemetry_log_filepath, raw_telemetry_log_filepath)'''
    if opts.aircraft is not None:
        if opts.mission is not None:
            print(opts.mission)
            dirname = "%s/logs/%s/Mission%s" % (opts.aircraft, time.strftime("%Y-%m-%d"), opts.mission)
        else:
            dirname = "%s/logs/%s" % (opts.aircraft, time.strftime("%Y-%m-%d"))
        # dirname is currently relative.  Possibly add state_basedir:
        if mpstate.settings.state_basedir is not None:
            dirname = os.path.join(mpstate.settings.state_basedir,dirname)
        mkdir_p(dirname)
        highest = None
        for i in range(1, 10000):
            fdir = os.path.join(dirname, 'flight%u' % i)
            if not os.path.exists(fdir):
                break
            highest = fdir
        if mpstate.continue_mode and highest is not None:
            fdir = highest
        elif os.path.exists(fdir):
            print("Flight logs full")
            sys.exit(1)
        logname = 'flight.tlog'
        logdir = fdir
    else:
        logname = os.path.basename(opts.logfile)
        dir_path = os.path.dirname(opts.logfile)
        if not os.path.isabs(dir_path) and mpstate.settings.state_basedir is not None:
            dir_path = os.path.join(mpstate.settings.state_basedir,dir_path)
        logdir = dir_path

    mkdir_p(logdir)
    return (logdir,
            os.path.join(logdir, logname),
            os.path.join(logdir, logname + '.raw'))

def set_stream_rates():
    '''set mavlink stream rates'''
    if (not msg_period.trigger() and
        mpstate.status.last_streamrate1 == mpstate.settings.streamrate and
        mpstate.status.last_streamrate2 == mpstate.settings.streamrate2):
        return
    mpstate.status.last_streamrate1 = mpstate.settings.streamrate
    mpstate.status.last_streamrate2 = mpstate.settings.streamrate2
    for master in mpstate.mav_master:
        if master.linknum == 0:
            rate = mpstate.settings.streamrate
        else:
            rate = mpstate.settings.streamrate2
        if rate != -1:
            master.mav.request_data_stream_send(mpstate.settings.target_system, mpstate.settings.target_component,
                                                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                rate, 1)

def check_link_status():
    '''check status of master links'''
    tnow = time.time()
    if mpstate.status.last_message != 0 and tnow > mpstate.status.last_message + 5:
        say("no link")
        mpstate.status.heartbeat_error = True
    for master in mpstate.mav_master:
        if not master.linkerror and (tnow > master.last_message + 5 or master.portdead):
            say("link %u down" % (master.linknum+1))
            master.linkerror = True

def send_heartbeat(master):
    if master.mavlink10():
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
    else:
        MAV_GROUND = 5
        MAV_AUTOPILOT_NONE = 4
        master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE)

def open_socket():
    sock_option = True
    print("attemping connection to vision system")
    port = 9999
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(('localhost', port))
    except socket.error:
        print("socket not availible")
    socket.timeout(0.1)
    print("connecting to socket: ", port)
    time.sleep(0.2)

def close_sock():

    self.sock_option = False
    socket.close()

def get_vision_data():
    self.sock_stream = self.sock.recv(14)
    try:
        data_string = self.sock_stream.split(',')
        self.current_depth  = int(data_string[0])
        self.xcenter        = int(data_string[1])
        self.ycenter        = int(''.join([i for i in data_string[2] if str.isdigit(i)]))
        self.depth_stream.appendleft(self.current_depth)
    except:
        print("could not convert network data")

def load_module(modname, quiet=False):
    '''load a module'''
    modpaths = ['MAVProxy.modules.mavproxy_%s' % modname, modname]
    for (m,pm) in mpstate.modules:
        if m.name == modname:
            if not quiet:
                print("module %s already loaded" % modname)
            return False
    for modpath in modpaths:
        try:
            m = import_package(modpath)
            reload(m)
            module = m.init(mpstate)
            if isinstance(module, mp_module.MPModule):
                mpstate.modules.append((module, m))
                if not quiet:
                    print("Loaded module %s" % (modname,))
                return True
            else:
                ex = "%s.init did not return a MPModule instance" % modname
                break
        except ImportError as msg:
            ex = msg
            if mpstate.settings.moddebug > 1:
                import traceback
                print(traceback.format_exc())
    print("Failed to load module: %s. Use 'set moddebug 3' in the MAVProxy console to enable traceback" % ex)
    return False
def periodic_tasks():
    '''run periodic checks'''
    if mpstate.status.setup_mode:
        return

    if mpstate.settings.heartbeat != 0:
        heartbeat_period.frequency = mpstate.settings.heartbeat

    if heartbeat_period.trigger() and mpstate.settings.heartbeat != 0:
        mpstate.status.counters['MasterOut'] += 1
        for master in mpstate.mav_master:
            send_heartbeat(master)

    if heartbeat_check_period.trigger():
        check_link_status()

    set_stream_rates()

    # call optional module idle tasks. These are called at several hundred Hz
    for (m,pm) in mpstate.modules:
        if hasattr(m, 'idle_task'):
            try:
                m.idle_task()
            except Exception as msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(exc_type, exc_value, exc_traceback,
                                              limit=2, file=sys.stdout)

        # also see if the module should be unloaded:
        if m.needs_unloading:
            unload_module(m.name)

'''Main loop of the program'''
def main_loop():
    '''main processing loop'''
    if not mpstate.status.setup_mode and not opts.nowait:
        for master in mpstate.mav_master:
            send_heartbeat(master)
            if master.linknum == 0:
                print("Waiting for heartbeat from %s" % master.address)
                master.wait_heartbeat()
        set_stream_rates()

    while True:
        if mpstate is None or mpstate.status.exit:
            return
        if mpstate.rl.line is not None:
            cmds = mpstate.rl.line.split(';')
            for c in cmds:
                process_stdin(c)
            mpstate.rl.line = None

        for master in mpstate.mav_master:
            if master.fd is None:
                if master.port.inWaiting() > 0:
                    process_master(master)

        periodic_tasks()

        rin = []
        for master in mpstate.mav_master:
            if master.fd is not None:
                rin.append(master.fd)
        for m in mpstate.mav_outputs:
            rin.append(m.fd)
        if rin == []:
            time.sleep(0.001)
            continue

        for fd in mpstate.select_extra:
            rin.append(fd)
        try:
            (rin, win, xin) = select.select(rin, [], [], 0.001)
        except select.error:
            continue

        if mpstate is None:
            return

        for fd in rin:
            for master in mpstate.mav_master:
                if fd == master.fd:
                    process_master(master)
                    continue
            for m in mpstate.mav_outputs:
                if fd == m.fd:
                    process_mavlink(m)
                    continue

            # this allow modules to register their own file descriptors
            # for the main select loop
            if fd in mpstate.select_extra:
                try:
                    # call the registered read function
                    (fn, args) = mpstate.select_extra[fd]
                    fn(args)
                except Exception, msg:
                    if mpstate.settings.moddebug == 1:
                        print(msg)
                    # on an exception, remove it from the select list
                    mpstate.select_extra.pop(fd)


''' Infinite Main Thread of the program '''
''' Other main loop found at main_loop() '''
def input_loop():
    '''wait for user input'''
    while mpstate.status.exit != True:
        try:
            if mpstate.status.exit != True:
                line = raw_input(mpstate.rl.prompt)
        except EOFError:
            mpstate.status.exit = True
            sys.exit(1)
        mpstate.input_queue.put(line)

def run_script(scriptfile):
    '''run a script file'''
    try:
        f = open(scriptfile, mode='r')
    except Exception:
        return
    mpstate.console.writeln("Running script %s" % scriptfile)
    for line in f:
        line = line.strip()
        if line == "" or line.startswith('#'):
            continue
        if line.startswith('@'):
            line = line[1:]
        else:
            mpstate.console.writeln("-> %s" % line)
        process_stdin(line)
    f.close()

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("mavproxy.py [options]")

    parser.add_option("--master", dest="master", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink master port and optional baud rate",
                      default=[])
    parser.add_option("--out", dest="output", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink output port and optional baud rate",
                      default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=57600)
    parser.add_option("--sitl", dest="sitl",  default=None, help="SITL output port")
    parser.add_option("--streamrate",dest="streamrate", default=4, type='int',
                      help="MAVLink stream rate")
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help='MAVLink source system for this GCS')
    parser.add_option("--source-component", dest='SOURCE_COMPONENT', type='int',
                      default=0, help='MAVLink source component for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=0, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=0, help='MAVLink target master component')
    parser.add_option("--logfile", dest="logfile", help="MAVLink master logfile",
                      default='mav.tlog')
    parser.add_option("-a", "--append-log", dest="append_log", help="Append to log files",
                      action='store_true', default=False)
    parser.add_option("--quadcopter", dest
                      ="quadcopter", help="use quadcopter controls",
                      action='store_true', default=False)
    parser.add_option("--setup", dest="setup", help="start in setup mode",
                      action='store_true', default=False)
    parser.add_option("--nodtr", dest="nodtr", help="disable DTR drop on close",
                      action='store_true', default=False)
    parser.add_option("--show-errors", dest="show_errors", help="show MAVLink error packets",
                      action='store_true', default=False)
    parser.add_option("--speech", dest="speech", help="use text to speach",
                      action='store_true', default=False)
    parser.add_option("--aircraft", dest="aircraft", help="aircraft name", default=None)
    parser.add_option("--cmd", dest="cmd", help="initial commands", default=None, action='append')
    parser.add_option("--console", action='store_true', help="use GUI console")
    parser.add_option("--map", action='store_true', help="load map module")
    parser.add_option(
        '--load-module',
        action='append',
        default=[],
        help='Load the specified module. Can be used multiple times, or with a comma separated list')
    parser.add_option("--mav09", action='store_true', default=False, help="Use MAVLink protocol 0.9")
    parser.add_option("--mav20", action='store_true', default=False, help="Use MAVLink protocol 2.0")
    parser.add_option("--auto-protocol", action='store_true', default=False, help="Auto detect MAVLink protocol version")
    parser.add_option("--nowait", action='store_true', default=False, help="don't wait for HEARTBEAT on startup")
    parser.add_option("-c", "--continue", dest='continue_mode', action='store_true', default=False, help="continue logs")
    parser.add_option("--dialect",  default="ardupilotmega", help="MAVLink dialect")
    parser.add_option("--rtscts",  action='store_true', help="enable hardware RTS/CTS flow control")
    parser.add_option("--mission", dest="mission", help="mission name", default=None)
    parser.add_option("--daemon", action='store_true', help="run in daemon mode, do not start interactive shell")
    parser.add_option("--profile", action='store_true', help="run the Yappi python profiler")
    parser.add_option("--state-basedir", default=None, help="base directory for logs and aircraft directories")
    parser.add_option("--version", action='store_true', help="version information")

    (opts, args) = parser.parse_args()

    # warn people about ModemManager which interferes badly with APM and Pixhawk
    if os.path.exists("/usr/sbin/ModemManager"):
        print("WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk")

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    if opts.mav20:
        os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil, mavparm, mavwp
    mavutil.set_dialect(opts.dialect)
    
    # global mavproxy state
    mpstate = MPState()
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode


    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        load_module('speech')
        say("STARTING")

    if not opts.master:
        serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*'])
        print('Auto-detected serial ports are:')
        for port in serial_list:
              print("%s" % port)

    # container for status information
    mpstate.status.target_system = opts.TARGET_SYSTEM
    mpstate.status.target_component = opts.TARGET_COMPONENT

    mpstate.mav_master = []
    # open master link
    for mdev in opts.master:
        if mdev.startswith('tcp:'):
            m = mavutil.mavtcp(mdev[4:])
        elif mdev.find(':') != -1:
            m = mavutil.mavudp(mdev, input=True)
        else:
            m = mavutil.mavserial(mdev, baud=opts.baudrate, autoreconnect=True)
        m.mav.set_callback(master_callback, m)
        m.linknum = len(mpstate.mav_master)
        m.linkerror = False
        m.link_delayed = False
        m.last_heartbeat = 0
        m.last_message = 0
        m.highest_msec = 0
        mpstate.mav_master.append(m)
        mpstate.status.counters['MasterIn'].append(0)


    # open any mavlink output ports
    for port in opts.output:
        mpstate.mav_outputs.append(mavutil.mavlink_connection(port, baud=int(opts.baudrate), input=False))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    if opts.state_basedir is not None:
        mpstate.settings.state_basedir = opts.state_basedir

    msg_period = mavutil.periodic_event(1.0/15)
    heartbeat_period = mavutil.periodic_event(1)
    heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.rl = rline("MAV> ")
    if opts.setup:
        mpstate.rl.set_prompt("")

    if opts.console:
        process_stdin('module load console')
        modname = 'autopilot'
        modpaths = ['MAVProxy.modules.mavproxy_%s' % modname, modname]
        
        modname = 'autopilot'
        modpath = 'mavproxy_%s' % (modname,)
        try:
            m = import_package(modpath)
            if m in mpstate.modules:
                raise RuntimeError("module %s already loaded" % (modname,))
            m.init(mpstate)
            mpstate.modules.append(m)
        except:
            print("could not load autopilot")

    if opts.map:
        process_stdin('module load map')

    for module in opts.load_module:
        modlist = module.split(',')
        for mod in modlist:
            process_stdin('module load %s' % mod)

    if 'HOME' in os.environ and not opts.setup:
        start_script = os.path.join(os.environ['HOME'], ".mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)

    if opts.aircraft is not None:
        start_script = os.path.join(opts.aircraft, "mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)
        else:
            print("no script %s" % start_script)

    if opts.cmd is not None:
        for cstr in opts.cmd:
            cmds = cstr.split(';')
            for c in cmds:
                process_stdin(c)

    # run main loop as a thread
    mpstate.status.thread = threading.Thread(target=main_loop, name='main_loop')
    mpstate.status.socket = threading.Thread(target=get_vision_data, name='get_vision_data')
    mpstate.status.thread.daemon = True
    mpstate.status.thread.start()

    # open socket to port 9999 to import vision data
    open_socket()
    # use main program for input. This ensures the terminal cleans
    # up on exit
    while (mpstate.status.exit != True):
        try:
            if opts.daemon:
                time.sleep(0.1)
            else:
                input_loop()
        except KeyboardInterrupt:
            if mpstate.settings.requireexit:
                print("Interrupt caught.  Use 'exit' to quit MAVProxy.")

                #Just lost the map and console, get them back:
                for (m,pm) in mpstate.modules:
                    if m.name in ["map", "console"]:
                        if hasattr(m, 'unload'):
                            try:
                                m.unload()
                            except Exception:
                                pass
                        reload(m)
                        m.init(mpstate)

            else:
                mpstate.status.exit = True
                sys.exit(1)

    if opts.profile:
        yappi.get_func_stats().print_all()
        yappi.get_thread_stats().print_all()

    #this loop executes after leaving the above loop and is for cleanup on exit
    for (m,pm) in mpstate.modules:
        if hasattr(m, 'unload'):
            print("Unloading module %s" % m.name)
            m.unload()
        
    sys.exit(1)
