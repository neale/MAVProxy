#!/usr/bin/env python
'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

'''

import sys, os, time, socket, signal, collections
import fnmatch, errno, threading
import serial, Queue, select
import traceback
import select
import shlex

from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import dumpstacks

# adding all this allows pyinstaller to build a working windows executable
# note that using --hidden-import does not work for these modules
try:
      from multiprocessing import freeze_support
      from pymavlink import mavwp, mavutil
      import matplotlib, HTMLParser
      try:
            import readline
      except ImportError:
            import pyreadline as readline
except Exception:
      pass

if __name__ == '__main__':
      freeze_support()

class MPStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        self.gps	 = None
        self.msgs = {}
        self.msg_count = {}
        self.sock = 0
        self.counters = {'MasterIn' : [], 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.setup_mode = opts.setup
        self.mav_error = 0
        self.altitude = 0
        self.last_altitude_announce = 0.0
        self.last_distance_announce = 0.0
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
        self.current_depth = 0
        self.xcenter = 0   
        self.ycenter = 0   
        self.circle_depth = 0
        self.isCircle = 0
        self.depth_stream = collections.deque([0]*5, 5)
        self.sock_failure_data = False
        self.socket_open = False
        self.pwm_val = 0
        self.auto = False
        # integrate rc module
        self.override = [ 0 ] * 16
        self.last_override = [ 0 ] * 16
        self.override_counter = 0
        self.sitl_output = False
        self.disarm_flag = False
        self.auto_t_started = False
        if  self.sitl_output:
             self.override_period = mavutil.periodic_event(20)
        else:
             self.override_period = mavutil.periodic_event(1)

    

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

def say_text(text, priority='important'):
    '''text output - default function for say()'''
    mpstate.console.writeln(text)

def say(text, priority='important'):
    '''text and/or speech output'''
    mpstate.functions.say(text, priority)

def add_input(cmd, immediate=False):
    '''add some command input to be processed'''
    if immediate:
        process_stdin(cmd)
    else:
        mpstate.input_queue.put(cmd)

class MAVFunctions(object):
    '''core functions available in modules'''
    def __init__(self):
        self.process_stdin = add_input
        self.param_set = param_set
        self.get_mav_param = get_mav_param
        self.say = say_text
        # input handler can be overridden by a module
        self.input_handler = None

class MPState(object):
    '''holds state of mavproxy'''
    def __init__(self):
        self.console = textconsole.SimpleConsole()
        self.map = None
        self.map_functions = {}
        self.vehicle_type = None
        self.vehicle_name = None
        from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
        self.settings = MPSettings(
            [ MPSetting('link', int, 1, 'Primary Link', tab='Link', range=(0,4), increment=1),
              MPSetting('streamrate', int, 4, 'Stream rate link1', range=(-1,20), increment=1),
              MPSetting('streamrate2', int, 4, 'Stream rate link2', range=(-1,20), increment=1),
              MPSetting('heartbeat', int, 1, 'Heartbeat rate', range=(0,5), increment=1),
              MPSetting('mavfwd', bool, True, 'Allow forwarded control'),
              MPSetting('mavfwd_rate', bool, False, 'Allow forwarded rate control'),
              MPSetting('shownoise', bool, True, 'Show non-MAVLink data'),
              MPSetting('baudrate', int, opts.baudrate, 'baudrate for new links', range=(0,10000000), increment=1),
              MPSetting('rtscts', bool, opts.rtscts, 'enable flow control'),
              MPSetting('select_timeout', float, 0.01, 'select timeout'),

              MPSetting('altreadout', int, 10, 'Altitude Readout',
                        range=(0,100), increment=1, tab='Announcements'),
              MPSetting('distreadout', int, 200, 'Distance Readout', range=(0,10000), increment=1),

              MPSetting('moddebug', int, opts.moddebug, 'Module Debug Level', range=(0,3), increment=1, tab='Debug'),
              MPSetting('compdebug', int, 0, 'Computation Debug Mask', range=(0,3), tab='Debug'),
              #MPSetting('flushlogs', bool, False, 'Flush logs on every packet'),
              MPSetting('requireexit', bool, False, 'Require exit command'),
              MPSetting('wpupdates', bool, True, 'Announce waypoint updates'),

              MPSetting('basealt', int, 0, 'Base Altitude', range=(0,30000), increment=1, tab='Altitude'),
              MPSetting('wpalt', int, 100, 'Default WP Altitude', range=(0,10000), increment=1),
              MPSetting('rallyalt', int, 90, 'Default Rally Altitude', range=(0,10000), increment=1),
              MPSetting('terrainalt', str, 'Auto', 'Use terrain altitudes', choice=['Auto','True','False']),
              MPSetting('rally_breakalt', int, 40, 'Default Rally Break Altitude', range=(0,10000), increment=1),
              MPSetting('rally_flags', int, 0, 'Default Rally Flags', range=(0,10000), increment=1),

              MPSetting('source_system', int, 255, 'MAVLink Source system', range=(0,255), increment=1, tab='MAVLink'),
              MPSetting('source_component', int, 0, 'MAVLink Source component', range=(0,255), increment=1),
              MPSetting('target_system', int, 0, 'MAVLink target system', range=(0,255), increment=1),
              MPSetting('target_component', int, 0, 'MAVLink target component', range=(0,255), increment=1),
              #MPSetting('state_basedir', str, None, 'base directory for logs and aircraft directories'),
              MPSetting('allow_unsigned', bool, True, 'whether unsigned packets will be accepted')
            ])

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
        self.sysid_outputs = {}

        # SITL output
        self.sitl_output = None

        self.mav_param = mavparm.MAVParmDict()
        self.modules = []
        self.public_modules = {}
        self.functions = MAVFunctions()
        self.select_extra = {}
        self.continue_mode = False
        self.aliases = {}
        import platform
        self.system = platform.system()

    def module(self, name):
        '''Find a public module (most modules are private)'''
        if name in self.public_modules:
            return self.public_modules[name]
        return None
    
    def master(self):
        '''return the currently chosen mavlink master object'''
        if len(self.mav_master) == 0:
              return None
        if self.settings.link > len(self.mav_master):
            self.settings.link = 1

        # try to use one with no link error
        if not self.mav_master[self.settings.link-1].linkerror:
            return self.mav_master[self.settings.link-1]
        for m in self.mav_master:
            if not m.linkerror:
                return m
        return self.mav_master[self.settings.link-1]


def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    return mpstate.mav_param.get(param, default)

def param_set(name, value, retries=3):
    '''set a parameter'''
    name = name.upper()
    return mpstate.mav_param.mavset(mpstate.master(), name, value, retries=retries)

def cmd_script(args):
    '''run a script'''
    if len(args) < 1:
        print("usage: script <filename>")
        return

    run_script(args[0])

def cmd_set(args):
    '''control mavproxy options'''
    mpstate.settings.command(args)

def cmd_status(args):
    '''show status'''
    if len(args) == 0:
        mpstate.status.show(sys.stdout, pattern=None)
    else:
        for pattern in args:
            mpstate.status.show(sys.stdout, pattern=pattern)

def cmd_setup(args):
    mpstate.status.setup_mode = True
    print("In setup mode")
    mpstate.rl.set_prompt("")


def cmd_reset(args):
    print("Resetting master")
    mpstate.master().reset()

def cmd_watch(args):
    '''watch a mavlink packet pattern'''
    if len(args) == 0:
        mpstate.status.watch = None
        return
    mpstate.status.watch = args[0]
    print("Watching %s" % mpstate.status.watch)

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

def unload_module(modname):
    '''unload a module'''
    for (m,pm) in mpstate.modules:
        if m.name == modname:
            if hasattr(m, 'unload'):
                m.unload()
            mpstate.modules.remove((m,pm))
            print("Unloaded module %s" % modname)
            return True
    print("Unable to find module %s" % modname)
    return False

def cmd_module(args):
    '''module commands'''
    usage = "usage: module <list|load|reload|unload>"
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "list":
        for (m,pm) in mpstate.modules:
            print("%s: %s" % (m.name, m.description))
    elif args[0] == "load":
        if len(args) < 2:
            print("usage: module load <name>")
            return
        load_module(args[1])
    elif args[0] == "reload":
        if len(args) < 2:
            print("usage: module reload <name>")
            return
        modname = args[1]
        pmodule = None
        for (m,pm) in mpstate.modules:
            if m.name == modname:
                pmodule = pm
        if pmodule is None:
            print("Module %s not loaded" % modname)
            return
        if unload_module(modname):
            import zipimport
            try:
                reload(pmodule)
            except ImportError:
                clear_zipimport_cache()
                reload(pmodule)                
            if load_module(modname, quiet=True):
                print("Reloaded module %s" % modname)
    elif args[0] == "unload":
        if len(args) < 2:
            print("usage: module unload <name>")
            return
        modname = os.path.basename(args[1])
        unload_module(modname)
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

def send_rc_override():
    '''send RC override packet'''
    if mpstate.status.sitl_output:
        buf = struct.pack('<HHHHHHHHHHHHHHHH',
                            *self.override)
        mpstate.status.sitl_output.write(buf)
    else:
        chan8 = mpstate.status.override[:8]
        mpstate.master().mav.rc_channels_override_send(mpstate.settings.target_system,
                                                        mpstate.settings.target_component,
                                                        *chan8)


def set_override(newchannels):
    '''this is a public method for use by drone API or other scripting'''
    mpstate.status.override = newchannels
    mpstate.status.override_counter = 10
    send_rc_override()


def cmd_rc(args):
    '''handle RC value override'''
    if len(args) != 2:
        print("Usage: rc <channel|all> <pwmvalue>")
        return
    value = int(args[1])
    if value > 65535 or value < -1:
        raise ValueError("PWM value must be a positive integer between 0 and 65535")
    if value == -1:
        value = 65535
    channels = mpstate.status.override
    if args[0] == 'all':
        for i in range(16):
            channels[i] = value
    else:
        channel = int(args[0])
        if channel < 1 or channel > 16:
            print("Channel must be between 1 and 8 or 'all'")
            return
        channels[channel - 1] = value
    set_override(channels)

def cmd_arm(args):
        arming_masks = {
            "all"     : 0x0001,
            "baro"    : 0x0002,
            "compass" : 0x0004,
            "gps"     : 0x0008,
            "ins"     : 0x0010,
            "params"  : 0x0020,
            "rc"      : 0x0040,
            "voltage" : 0x0080,
            "battery" : 0x0100
        }

        '''arm commands'''
        usage = "usage: arm <check|uncheck|list|throttle|safetyon|safetyoff>"
        checkables = "<all|baro|compass|gps|ins|params|rc|voltage|battery>"

        if len(args) <= 0:
            print(usage)
            return

        if args[0] == "check":
            if (len(args) < 2):
                print("usage: arm check", checkables)
                return

            arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
            name = args[1].lower()
            if name == 'all':
                for name in arming_masks.keys():
                    arming_mask |= arming_masks[name]
            elif name in arming_masks:
                arming_mask |= arming_masks[name]
            else:
                print("unrecognized arm check:", name)
                return
            self.param_set("ARMING_CHECK", arming_mask)
            return

        if args[0] == "uncheck":
            if (len(args) < 2):
                print("usage: arm uncheck", checkables)
                return

            arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
            name = args[1].lower()
            if name == 'all':
                arming_mask = 0
            elif name in arming_masks:
                arming_mask &= ~arming_masks[name]
            else:
                print("unrecognized arm check:", args[1])
                return

            self.param_set("ARMING_CHECK", arming_mask)
            return

        if args[0] == "list":
            arming_mask = int(self.get_mav_param("ARMING_CHECK",0))
            if arming_mask == 0:
                print("NONE")
            for name in arming_masks.keys():
                if arming_masks[name] & arming_mask:
                    print(name)
            return

        if args[0] == "throttle":
            mpstate.master().arducopter_arm()
            return

        if args[0] == "safetyon":
            mpstate.master().mav.set_mode_send(mpstate.settings.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                          1)
            return

        if args[0] == "safetyoff":
            mpstate.master().mav.set_mode_send(mpstate.settings.target_system,
                                          mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                                          0)
            return

        print(usage)

def cmd_disarm(args):
    '''disarm motors'''
    mpstate.status.disarm_flag = True
    p2 = 0
    if len(args) == 1 and args[0] == 'force':
        p2 = 21196
    mpstate.master().mav.command_long_send(
        mpstate.settings.target_system,  # target_system
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

def cmd_pvision(args):

    try:
        print ("current depth to object : {}\nCurrent relative center point : ({},{})\n Circle depth {}\nCircle in frame: {}\nSocket open : {}\n".format(
            mpstate.status.current_depth,
            mpstate.status.xcenter, 
            mpstate.status.ycenter,   
            mpstate.status.socket_open,
            mpstate.status.circle_depth, 
            mpstate.status.isCircle
            ))
    except:
        print("data not availible")

def cmd_ap(args):
    mpstate.status.auto = True
    try:
        mpstate.status.auto_t.start()
    except:
        cmd_disarm('force')

def autopilot_t():
    mpstate.status.auto_t_started = True
    while mpstate.status.isCircle:
        
        if mpstate.status.disarm_flag is True:
            cmd_disarm('force')
            break

        average = sum(mpstate.status.depth_stream)/5
        circle_depth = mpstate.status.circle_depth
        """ Set PWM autopilot PID """
        if circle_depth <= 600 and circle_depth >= 500:
            if mpstate.status.pwm_val is not 1500:
                print("Stabilizing", circle_depth)
                mpstate.status.pwm_val = 1500
                cmd_rc([3, mpstate.status.pwm_val])

        # Coptor isn't high enough

        elif circle_depth < 500 and circle_depth > 10:
            if mpstate.status.pwm_val is not 1550:
                print("Throttling up", circle_depth)
                mpstate.status.pwm_val = 1550
                cmd_rc([3, mpstate.status.pwm_val])


        # Coptor is higher than we want     
        elif circle_depth > 600 and circle_depth < 1000:
            if mpstate.status.pwm_val is not 1400:
                print("Throttling down", average)
                mpstate.status.pwm_val = 1400
                cmd_rc([3, mpstate.status.pwm_val])
        


def clear_zipimport_cache():
    """Clear out cached entries from _zip_directory_cache.
    See http://www.digi.com/wiki/developer/index.php/Error_messages"""
    import sys, zipimport
    syspath_backup = list(sys.path)
    zipimport._zip_directory_cache.clear()
 
    # load back items onto sys.path
    sys.path = syspath_backup
    # add this too: see https://mail.python.org/pipermail/python-list/2005-May/353229.html
    sys.path_importer_cache.clear()

# http://stackoverflow.com/questions/211100/pythons-import-doesnt-work-as-expected
# has info on why this is necessary.

def import_package(name):
    """Given a package name like 'foo.bar.quux', imports the package
    and returns the desired module."""
    import zipimport
    try:
        mod = __import__(name)
    except ImportError:
        clear_zipimport_cache()
        mod = __import__(name)
        
    components = name.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod


command_map = {
    'script'    : (cmd_script,   'run a script of MAVProxy commands'),
    'setup'     : (cmd_setup,    'go into setup mode'),
    'reset'     : (cmd_reset,    'reopen the connection to the MAVLink master'),
    'status'    : (cmd_status,   'show status'),
    'set'       : (cmd_set,      'mavproxy settings'),
    'watch'     : (cmd_watch,    'watch a MAVLink pattern'),
    'module'    : (cmd_module,   'module commands'),
    'alias'     : (cmd_alias,    'command aliases'),
    'rc'        : (cmd_rc,       'give rc channel commands'),
    'pvision'   : (cmd_pvision,  'prints out user data related to vision system'),
    'autopilot' : (cmd_ap,       'start the autopilot'), # altitude only for now
    'arm'       : (cmd_arm,      'arm coptor in various modes'),
    'disarm'    : (cmd_disarm,   'disarm coptor')
    }

def process_stdin(line):
    '''handle commands from user'''
    if line is None:
        sys.exit(0)

    # allow for modules to override input handling
    if mpstate.functions.input_handler is not None:
          mpstate.functions.input_handler(line)
          return
    
    line = line.strip()

    if mpstate.status.setup_mode:
        # in setup mode we send strings straight to the master
        if line == '.':
            mpstate.status.setup_mode = False
            mpstate.status.flightmode = "ALT_HOLD"
            mpstate.rl.set_prompt("MAV> ")
            return
        if line != '+++':
            line += '\r'
        for c in line:
            time.sleep(0.01)
            mpstate.master().write(c)
        return

    if not line:
        return

    args = shlex.split(line)
    cmd = args[0]
    while cmd in mpstate.aliases:
        line = mpstate.aliases[cmd]
        args = shlex.split(line) + args[1:]
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

    #if mpstate.logqueue_raw:
    #    mpstate.logqueue_raw.put(str(s))

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

def periodic_tasks():
    '''run periodic checks'''
    if not mpstate.status.auto_t.isAlive() and mpstate.status.auto_t_started:
            print("autopilot thread died")
            mpstate.status.auto_t_started = False
    if mpstate.status.setup_mode:
        return

    if (mpstate.settings.compdebug & 2) != 0:
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
    if mpstate.status.override_period.trigger():
        if (mpstate.status.override != [ 0 ] * 16 or
            mpstate.status.override != mpstate.status.last_override or
            mpstate.status.override_counter > 0):
            mpstate.status.last_override = mpstate.status.override[:]
            send_rc_override()
            if mpstate.status.override_counter > 0:
                mpstate.status.override_counter -= 1
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
        while not mpstate.input_queue.empty():
            line = mpstate.input_queue.get()
            mpstate.input_count += 1
            cmds = line.split(';')
            if len(cmds) == 1 and cmds[0] == "":
                  mpstate.empty_input_count += 1                 
            for c in cmds:
                process_stdin(c)

        for master in mpstate.mav_master:
            if master.fd is None:
                if master.port.inWaiting() > 0:
                    process_master(master)

        periodic_tasks()

        rin = []
        for master in mpstate.mav_master:
            if master.fd is not None and not master.portdead:
                rin.append(master.fd)
        for m in mpstate.mav_outputs:
            rin.append(m.fd)
        for sysid in mpstate.sysid_outputs:
            m = mpstate.sysid_outputs[sysid]
            rin.append(m.fd)
        if rin == []:
            time.sleep(0.0001)
            continue

        for fd in mpstate.select_extra:
            rin.append(fd)
        try:
            (rin, win, xin) = select.select(rin, [], [], mpstate.settings.select_timeout)
        except select.error:
            continue

        if mpstate is None:
            return

        for fd in rin:
            if mpstate is None:
                  return
            for master in mpstate.mav_master:
                  if fd == master.fd:
                        process_master(master)
                        if mpstate is None:
                              return
                        continue
            for m in mpstate.mav_outputs:
                if fd == m.fd:
                    process_mavlink(m)
                    if mpstate is None:
                          return
                    continue

            for sysid in mpstate.sysid_outputs:
                m = mpstate.sysid_outputs[sysid]
                if fd == m.fd:
                    process_mavlink(m)
                    if mpstate is None:
                          return
                    continue

            # this allow modules to register their own file descriptors
            # for the main select loop
            if fd in mpstate.select_extra:
                try:
                    # call the registered read function
                    (fn, args) = mpstate.select_extra[fd]
                    fn(args)
                except Exception as msg:
                    if mpstate.settings.moddebug == 1:
                        print(msg)
                    # on an exception, remove it from the select list
                    mpstate.select_extra.pop(fd)

def open_socket():
    port = 9999
    mpstate.status.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        mpstate.status.sock.connect(('localhost', port))
        mpstate.status.socket_open = True    
        print("connecting to socket: {}\n".format(port))

    except socket.error:
        if not mpstate.status.sock_failure_data:
            print("socket not availible\n")
        mpstate.status.socket_open = False
    socket.timeout(0.1)
    time.sleep(0.2)

def get_vision_data():
    
    while 1:
        if not mpstate.status.socket_open:
            open_socket()
        try:
            sock_stream = mpstate.status.sock.recv(21)
        except:
            if not mpstate.status.sock_failure_data:
                print("Socket data could not be resolved, vision system offline\n")
            mpstate.status.sock_failure_data = True
 
        try:
            data_string = sock_stream.split(',')

            mpstate.status.current_depth  = int(data_string[0])
            mpstate.status.xcenter        = int(data_string[1])
            mpstate.status.ycenter        = int(data_string[2])
            mpstate.status.circle_depth   = int(data_string[3])
            mpstate.status.isCircle       = int(data_string[4][0])
            mpstate.status.depth_stream.appendleft(mpstate.status.circle_depth)
            mpstate.status.sock_failure_data = False

        except: 
            if not mpstate.status.sock_failure_data:
                print("could not convert network data\n")
                if sock_stream:
                    print("data printback: {}\n".format(sock_stream))


def input_loop():
    '''wait for user input'''
    while mpstate.status.exit != True:
        print(mpstate.status.current_depth)
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
    #parser.add_option("--logfile", dest="logfile", help="MAVLink master logfile",
    #                  default='mav.tlog')
    #parser.add_option("-a", "--append-log", dest="append_log", help="Append to log files",
    #                  action='store_true', default=False)
    parser.add_option("--quadcopter", dest="quadcopter", help="use quadcopter controls",
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
    parser.add_option("--moddebug",  type=int, help="module debug level", default=0)
    parser.add_option("--mission", dest="mission", help="mission name", default=None)
    parser.add_option("--daemon", action='store_true', help="run in daemon mode, do not start interactive shell")
    parser.add_option("--profile", action='store_true', help="run the Yappi python profiler")
    parser.add_option("--state-basedir", default=None, help="base directory for logs and aircraft directories")
    parser.add_option("--version", action='store_true', help="version information")
    parser.add_option("--default-modules", default="signing,wp,rally,fence,param,relay,tuneopt,mode,calibration,auxopt,misc,cmdlong,battery,terrain,output", help='default module list')

    (opts, args) = parser.parse_args()

    # warn people about ModemManager which interferes badly with APM and Pixhawk
    if os.path.exists("/usr/sbin/ModemManager"):
        print("WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk")

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    if opts.mav20:
        os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil, mavparm
    mavutil.set_dialect(opts.dialect)

    #version information
    if opts.version:
        import pkg_resources
        version = pkg_resources.require("mavproxy")[0].version
        print "MAVProxy is a modular ground station using the mavlink protocol"
        print "MAVProxy Version: " + version
        sys.exit(1)
    
    # global mavproxy state
    mpstate = MPState()
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode
    # queues for logging
    #mpstate.logqueue = Queue.Queue()
    #mpstate.logqueue_raw = Queue.Queue()


    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        load_module('speech')

    if not opts.master:
        serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*'])
        print('Auto-detected serial ports are:')
        for port in serial_list:
              print("%s" % port)

    # container for status information
    mpstate.settings.target_system = opts.TARGET_SYSTEM
    mpstate.settings.target_component = opts.TARGET_COMPONENT

    mpstate.mav_master = []

    mpstate.rl = rline.rline("MAV> ", mpstate)

    def quit_handler(signum = None, frame = None):
        #print 'Signal handler called with signal', signum
        if mpstate.status.exit:
            print 'Clean shutdown impossible, forcing an exit'
            sys.exit(0)
        else:
            mpstate.status.exit = True

    # Listen for kill signals to cleanly shutdown modules
    fatalsignals = [signal.SIGTERM]
    try:
        fatalsignals.append(signal.SIGHUP)
        fatalsignals.append(signal.SIGQUIT)
    except Exception:
        pass
    if opts.daemon: # SIGINT breaks readline parsing - if we are interactive, just let things die
        fatalsignals.append(signal.SIGINT)

    for sig in fatalsignals:
        signal.signal(sig, quit_handler)

    load_module('link', quiet=True)

    mpstate.settings.source_system = opts.SOURCE_SYSTEM
    mpstate.settings.source_component = opts.SOURCE_COMPONENT

    # open master link
    for mdev in opts.master:
        if not mpstate.module('link').link_add(mdev):
            sys.exit(1)

    if not opts.master and len(serial_list) == 1:
          print("Connecting to %s" % serial_list[0])
          mpstate.module('link').link_add(serial_list[0].device)
    elif not opts.master:
          wifi_device = '0.0.0.0:14550'
          mpstate.module('link').link_add(wifi_device)


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

    mpstate.input_queue = Queue.Queue()
    mpstate.input_count = 0
    mpstate.empty_input_count = 0
    if opts.setup:
        mpstate.rl.set_prompt("")

    # call this early so that logdir is setup based on --aircraft
    #(mpstate.status.logdir, logpath_telem, logpath_telem_raw) = log_paths()

    if not opts.setup:
        # some core functionality is in modules
        standard_modules = opts.default_modules.split(',')
        for m in standard_modules:
            load_module(m, quiet=True)

    if opts.console:
        process_stdin('module load console')

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
    if 'LOCALAPPDATA' in os.environ and not opts.setup:
        start_script = os.path.join(os.environ['LOCALAPPDATA'], "MAVProxy", "mavinit.scr")
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

    if opts.profile:
        import yappi    # We do the import here so that we won't barf if run normally and yappi not available
        yappi.start()

    # log all packets from the master, for later replay
    #open_telemetry_logs(logpath_telem, logpath_telem_raw)
    #process_stdin("mode ALT_HOLD")
    # run main loop as a thread
    print "here"
    mpstate.status.thread = threading.Thread(target=main_loop, name="main_loop")
    print "nope"
    mpstate.status.socket = threading.Thread(target=get_vision_data, name='get_vision_data')
    mpstate.status.auto_t  = threading.Thread(target=autopilot_t, name='autopilot_t')
    mpstate.status.auto_t.daemon = True
    mpstate.status.thread.daemon = False
    mpstate.status.socket.daemon = True
    mpstate.status.thread.start()
    mpstate.status.socket.start()
    print "nope"

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
    mpstate.status.sock.close()

    for (m,pm) in mpstate.modules:
        if hasattr(m, 'unload'):
            print("Unloading module %s" % m.name)
            m.unload()    
    sys.exit(1)
