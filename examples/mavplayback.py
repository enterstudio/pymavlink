#!/usr/bin/env python

'''
play back a mavlink log as a FlightGear FG NET stream, and as a
realtime mavlink stream

Useful for visualising flights
'''

import sys, time, os, struct
import Tkinter

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

import fgFDM

from optparse import OptionParser
parser = OptionParser("mavplayback.py [options]")

parser.add_option("--planner",dest="planner", action='store_true', help="use planner file format")
parser.add_option("--robust",dest="robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("--gpsalt", action='store_true', default=False, help="Use GPS altitude")
parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
parser.add_option("--out",   help="MAVLink output port (IP:port)",
                  action='append', default=['127.0.0.1:14550'])
parser.add_option("--fgout", action='append', default=['127.0.0.1:5503'],
                  help="flightgear FDM NET output (IP:port)")
(opts, args) = parser.parse_args()

if opts.mav10:
    os.environ['MAVLINK10'] = '1'
import mavutil

if len(args) < 1:
    parser.print_help()
    sys.exit(1)

filename = args[0]

class App():
    def __init__(self, filename):
        self.root = Tkinter.Tk()

        self.mlog = mavutil.mavlink_connection(filename, planner_format=opts.planner,
                                               robust_parsing=opts.robust)
        self.mout = []
        for m in opts.out:
            self.mout.append(mavutil.mavudp(m, input=False))

        self.fgout = []
        for f in opts.fgout:
            self.fgout.append(mavutil.mavudp(f, input=False))
    
        self.fdm = fgFDM.fgFDM()

        self.msg = self.mlog.recv_match(condition=opts.condition)
        if self.msg is None:
            sys.exit(1)
        self.last_timestamp = getattr(self.msg, '_timestamp')

        self.frame = Tkinter.Frame(self.root)
        self.frame.pack()

        self.quit = Tkinter.Button(self.frame, text="QUIT", command=self.frame.quit)
        self.quit.pack(side=Tkinter.LEFT)

        self.clock = Tkinter.Label(self.frame,text="")
        self.clock.pack()

        self.playback = Tkinter.Spinbox(self.frame, from_=0, to=20, increment=0.5)
        self.playback.pack()
        self.playback.delete(0, "end")
        self.playback.insert(0, 1)

        self.next_message()
        self.root.mainloop()

    def next_message(self):
        '''called as each msg is ready'''
        
        msg = self.msg
        if msg is None:
            return

        speed = float(self.playback.get())
        if speed == 0.0:
            self.root.after(100, self.next_message)
            return

        timestamp = getattr(msg, '_timestamp')

        now = time.strftime("%H:%M:%S", time.localtime(timestamp))
        self.clock.configure(text=now)

        self.root.after(int(1000*(timestamp - self.last_timestamp) / speed), self.next_message)
        self.last_timestamp = timestamp
        self.msg = self.mlog.recv_match(condition=opts.condition)

        for m in self.mout:
            m.write(struct.pack('>Q', timestamp*1.0e6))
            m.write(msg.get_msgbuf().tostring())

        if msg.get_type() == "GPS_RAW":
            self.fdm.set('latitude', msg.lat, units='degrees')
            self.fdm.set('longitude', msg.lon, units='degrees')
            if opts.gpsalt:
                self.fdm.set('altitude', msg.alt, units='meters')

        if msg.get_type() == "VFR_HUD":
            if not opts.gpsalt:
                self.fdm.set('altitude', msg.alt, units='meters')
            self.fdm.set('num_engines', 1)
            self.fdm.set('vcas', msg.airspeed, units='mps')

        if msg.get_type() == "ATTITUDE":
            self.fdm.set('phi', msg.roll, units='radians')
            self.fdm.set('theta', msg.pitch, units='radians')
            self.fdm.set('psi', msg.yaw, units='radians')
            self.fdm.set('phidot', msg.rollspeed, units='rps')
            self.fdm.set('thetadot', msg.pitchspeed, units='rps')
            self.fdm.set('psidot', msg.yawspeed, units='rps')

        if msg.get_type() == "RC_CHANNELS_SCALED":
            self.fdm.set("right_aileron", msg.chan1_scaled*0.0001)
            self.fdm.set("left_aileron", -msg.chan1_scaled*0.0001)
            self.fdm.set("rudder",        msg.chan4_scaled*0.0001)
            self.fdm.set("elevator",      msg.chan2_scaled*0.0001)
            self.fdm.set('rpm',           msg.chan3_scaled*0.01)

        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()

        if msg.get_type() == 'STATUSTEXT':
            print("APM: %s" % msg.text)

        if self.fdm.get('latitude') != 0:
            for f in self.fgout:
                f.write(self.fdm.pack())


app=App(filename)
