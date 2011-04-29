#!/usr/bin/python

# The MIT License
#
# Copyright (c) 2007 Damon Kohler
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Modified for use in ROS by Ken Conley. API names have been modified
# for consistency ROS Python coding guidelines.

from __future__ import with_statement

"""iRobot Roomba Serial control Interface (SCI) and Turtlebot Open Interface (OI).

turtlebot.py is a fork of PyRobot.

PyRobot was originally based on openinterface.py, developed by iRobot
Corporation. Many of the docstrings from openinterface.py, particularly those
which describe the specification, are also used here. Also, docstrings may
contain specification text copied directly from the Roomba SCI Spec Manual and
the Turtlebot Open Interface specification.

Since SCI is a subset of OI, PyRobot first defines the Roomba's functionality
in the Roomba class and then extends that with the Turtlebot's additional
functionality in the Turtlebot class. In addition, since OI is built on SCI the
SerialCommandInterface class is also used for OI.

"""
__author__ = "damonkohler@gmail.com (Damon Kohler)"

import logging
import math
import serial
import struct
import time
import threading
import traceback

ROOMBA_OPCODES = dict(
    start = 128,
    baud = 129,
    control = 130,
    safe = 131,
    full = 132,
    power = 133,
    spot = 134,
    clean = 135,
    max = 136,
    drive = 137,
    motors = 138,
    leds = 139,
    song = 140,
    play = 141,
    sensors = 142,
    force_seeking_dock = 143,
    )

CREATE_OPCODES = dict(
    soft_reset = 7,  # Where is this documented?
    low_side_drivers = 138,
    song = 140,
    play_song = 141,
    pwm_low_side_drivers = 144,
    direct_drive = 145,
    digital_outputs = 147,
    stream = 148,
    query_list = 149,
    pause_resume_stream = 150,
    send_ir = 151,
    script = 152,
    play_script = 153,
    show_script = 154,
    wait_time = 155,
    wait_distance = 156,
    wait_angle = 157,
    wait_event = 158,
    )

REMOTE_OPCODES = {
    # Remote control.
    129: 'left',
    130: 'forward',
    131: 'right',
    132: 'spot',
    133: 'max',
    134: 'small',
    135: 'medium',
    136: 'large',
    136: 'clean',
    137: 'pause',
    138: 'power',
    139: 'arc-left',
    140: 'arc-right',
    141: 'drive-stop',
    # Scheduling remote.
    142: 'send-all',
    143: 'seek-dock',
    # Home base.
    240: 'reserved',
    242: 'force-field',
    244: 'green-buoy',
    246: 'green-buoy-and-force-field',
    248: 'red-buoy',
    250: 'red-buoy-and-force-field',
    252: 'red-buoy-and-green-buoy',
    254: 'red-buoy-and-green-buoy-and-force-field',
    255: 'none',
    }

BAUD_RATES = (  # In bits per second.
    300,
    600,
    1200,
    2400,
    4800,
    9600,
    14400,
    19200,
    28800,
    38400,
    57600,  # Default.
    115200)

CHARGING_STATES = (
    'not-charging',
    'charging-recovery',
    'charging',
    'trickle-charging',
    'waiting',
    'charging-error')

OI_MODES = (
    'off',
    'passive',
    'safe',
    'full')

# Various state flag masks
WHEEL_DROP_CASTER = 0x10
WHEEL_DROP_LEFT = 0x08
WHEEL_DROP_RIGHT = 0x04
BUMP_LEFT = 0x02
BUMP_RIGHT = 0x01

OVERCURRENTS_DRIVE_LEFT = 0x10
OVERCURRENTS_DRIVE_RIGHT = 0x08
OVERCURRENTS_MAIN_BRUSH = 0x04
OVERCURRENTS_VACUUM = 0x02
OVERCURRENTS_SIDE_BRUSH = 0x01

BUTTON_POWER = 0x08
BUTTON_SPOT = 0x04
BUTTON_CLEAN = 0x02
BUTTON_MAX = 0x01

SENSOR_GROUP_PACKET_LENGTHS = (26, 10, 6, 10, 14, 12, 52)

# From: http://www.harmony-central.com/MIDI/Doc/table2.html
MIDI_TABLE = {'rest': 0, 'R': 0, 'pause': 0,
              'G1': 31, 'G#1': 32, 'A1': 33,
              'A#1': 34, 'B1': 35,

              'C2': 36, 'C#2': 37, 'D2': 38,
              'D#2': 39, 'E2': 40, 'F2': 41,
              'F#2': 42, 'G2': 43, 'G#2': 44,
              'A2': 45, 'A#2': 46, 'B2': 47,

              'C3': 48, 'C#3': 49, 'D3': 50,
              'D#3': 51, 'E3': 52, 'F3': 53,
              'F#3': 54, 'G3': 55, 'G#3': 56,
              'A3': 57, 'A#3': 58, 'B3': 59,

              'C4': 60, 'C#4': 61, 'D4': 62,
              'D#4': 63, 'E4': 64, 'F4': 65,
              'F#4': 66, 'G4': 67, 'G#4': 68,
              'A4': 69, 'A#4': 70, 'B4': 71,

              'C5': 72, 'C#5': 73, 'D5': 74,
              'D#5': 75, 'E5': 76, 'F5': 77,
              'F#5': 78, 'G5': 79, 'G#5': 80,
              'A5': 81, 'A#5': 82, 'B5': 83,

              'C6': 84, 'C#6': 85, 'D6': 86,
              'D#6': 87, 'E6': 88, 'F6': 89,
              'F#6': 90, 'G6': 91, 'G#6': 92,
              'A6': 93, 'A#6': 94, 'B6': 95,

              'C7': 96, 'C#7': 97, 'D7': 98,
              'D#7': 99, 'E7': 100, 'F7': 101,
              'F#7': 102, 'G7': 103, 'G#7': 104,
              'A7': 105, 'A#7': 106, 'B7': 107,

              'C8': 108, 'C#8': 109, 'D8': 110,
              'D#8': 111, 'E8': 112, 'F8': 113,
              'F#8': 114, 'G8': 115, 'G#8': 116,
              'A8': 117, 'A#8': 118, 'B8': 119,

              'C9': 120, 'C#9': 121, 'D9': 122,
              'D#9': 123, 'E9': 124, 'F9': 125,
              'F#9': 126, 'G9': 127}

# drive constants.
RADIUS_TURN_IN_PLACE_CW = -1
RADIUS_TURN_IN_PLACE_CCW = 1
RADIUS_STRAIGHT = 32768
RADIUS_MAX = 2000

VELOCITY_MAX = 500  # mm/s
VELOCITY_SLOW = int(VELOCITY_MAX * 0.33)
VELOCITY_FAST = int(VELOCITY_MAX * 0.66)

MAX_WHEEL_SPEED = 500
WHEEL_SEPARATION = 260  # mm

SERIAL_TIMEOUT = 2  # Number of seconds to wait for reads. 2 is generous.
START_DELAY = 5  # Time it takes the Roomba/Turtlebot to boot.


assert struct.calcsize('H') == 2, 'Expecting 2-byte shorts.'


class DriverError(Exception):
  pass


class SerialCommandInterface(object):

  """A higher-level wrapper around PySerial specifically designed for use with
  iRobot's SCI.

  """
  def __init__(self, tty, baudrate):
    self.ser = serial.Serial(tty, baudrate=baudrate, timeout=SERIAL_TIMEOUT)
    self.ser.open()
    self.opcodes = {}

    #TODO: kwc all locking code should be outside of the driver. Instead,
    #could place a lock object in Roomba and let people "with" it
    self.lock = threading.RLock()

  def wake(self):
    """wake up robot."""
    self.ser.setRTS(0)
    time.sleep(0.25)
    self.ser.setRTS(1)
    time.sleep(1)  # Technically it should wake after 500ms.

  def add_opcodes(self, opcodes):
    """Add available opcodes to the SCI."""
    self.opcodes.update(opcodes)

  def send(self, bytes):
    """send a string of bytes to the robot."""
    with self.lock:
      self.ser.write(struct.pack('B' * len(bytes), *bytes))

  #TODO: kwc the locking should be done at a higher level
  def read(self, num_bytes):
    """Read a string of 'num_bytes' bytes from the robot."""
    logging.debug('Attempting to read %d bytes from SCI port.' % num_bytes)
    with self.lock:
      data = self.ser.read(num_bytes)
    logging.debug('Read %d bytes from SCI port.' % len(data))
    if not data:
      raise DriverError('Error reading from SCI port. No data.')
    if len(data) != num_bytes:
      raise DriverError('Error reading from SCI port. Wrong data length.')
    return data

  def flush_input(self):
    """Flush input buffer, discarding all its contents."""
    logging.debug('Flushing serial input buffer.')
    self.ser.flushInput()

  def __getattr__(self, name):
    """Turtlebots methods for opcodes on the fly.

    Each opcode method sends the opcode optionally followed by a string of
    bytes.

    """
    #TODO: kwc do static initialization instead
    if name in self.opcodes:
      def send_opcode(*bytes):
        logging.debug('sending opcode %s.' % name)
        self.send([self.opcodes[name]] + list(bytes))
      return send_opcode
    raise AttributeError


class RoombaSensors(object):

  """Retrive and decode the Roomba's sensor data.

  Some of the specification is included in the docstrings. For a complete
  description, see the Roomba SCI Sepc Manual.

  """
  def __init__(self, robot):
    self.robot = robot
    self.data = {}  # Last sensor readings.
    self.data['user-digital-outputs']=0

  def clear(self):
    """clear out old sensor data."""
    self.data = {}

  def __getitem__(self, name):
    """Indexes into sensor data."""
    return self.data[name]

  def __contains__(self, name):
    """Indexes into sensor data."""
    return name in self.data

  def _decode_group_packet_0(self, bytes, stamp):
    """Decode sensord data from a request for group 0 (all data)."""
    # NOTE(damonkohler): We decode sensor data in reverse order for better pop
    # performance.
    self.data['timestamp'] = stamp
    self.decode_unsigned_short('capacity', bytes.pop(), bytes.pop())  # mAh
    self.decode_unsigned_short('charge', bytes.pop(), bytes.pop())  # mAh
    self.decode_byte('temperature', bytes.pop())  # C
    self.decode_short('current', bytes.pop(), bytes.pop())  # mA
    self.decode_unsigned_short('voltage', bytes.pop(), bytes.pop())  # mV
    self.decode_unsigned_byte('charging-state', bytes.pop())
    self.angle(bytes.pop(), bytes.pop(), 'degrees')
    self.decode_short('distance', bytes.pop(), bytes.pop())  # mm
    self.decode_unsigned_byte('buttons', bytes.pop())
    
    self.decode_unsigned_byte('remote-opcode', bytes.pop())
    self.decode_unsigned_byte('dirt-detector-right', bytes.pop())
    self.decode_unsigned_byte('dirt-detector-left', bytes.pop())
    self.decode_unsigned_byte('motor-overcurrents', bytes.pop())
    #self.motor_overcurrents(bytes.pop())
    self.decode_bool('virtual-wall', bytes.pop())
    self.decode_bool('cliff-right', bytes.pop())
    self.decode_bool('cliff-front-right', bytes.pop())
    self.decode_bool('cliff-front-left', bytes.pop())
    self.decode_bool('cliff-left', bytes.pop())
    self.decode_bool('wall', bytes.pop())
    self.decode_unsigned_byte('bumps-wheeldrops', bytes.pop())    
    #self.bumps_wheeldrops(bytes.pop())

  def request_packet(self, packet_id):
    """Reqeust a sensor packet."""
    with self.robot.sci.lock:
      logging.debug('Requesting sensor packet %d.' % packet_id)
      self.robot.sci.flush_input()
      self.robot.sci.sensors(packet_id)
      #kwc: there appears to be a 10-20ms latency between sending the
      #sensor request and fully reading the packet.  Based on
      #observation, we are preferring the 'before' stamp rather than
      #after.
      stamp = time.time()
      length = SENSOR_GROUP_PACKET_LENGTHS[packet_id]
      data = list(self.robot.sci.read(length))
      return data, stamp

  def get_all(self):
    """Request and decode all available sensor data."""
    bytes, stamp = self.request_packet(0)
    if bytes is not None:
      self._decode_group_packet_0(bytes, stamp)

  def angle(self, low, high, unit=None):
    """The angle that Roomba has turned through since the angle was last
    requested. The angle is expressed as the difference in the distance
    traveled by Roomba's two wheels in millimeters, specifically the right
    wheel distance minus the left wheel distance, divided by two. This makes
    counter-clockwise angles positive and clockwise angles negative. This can
    be used to directly calculate the angle that Roomba has turned through
    since the last request. Since the distance between Roomba's wheels is
    258mm, the equations for calculating the angles in familiar units are:

    angle in radians = (2 * difference) / 258
    angle in degrees = (360 * difference) / (258 * Pi).

    If the value is not polled frequently enough, it will be capped at its
    minimum or maximum.

    Note: Reported angle and distance may not be accurate. Roomba measures
    these by detecting its wheel revolutions. If for example, the wheels slip
    on the floor, the reported angle of distance will be greater than the
    actual angle or distance.

    """
    if unit not in (None, 'radians', 'degrees'):
      raise DriverError('Invalid angle unit specified.')
    self.decode_short('angle', low, high)
    #if unit == 'degrees':  ## Changed by tfoote for the turtlebot.  Need to verify if this is correct for the roomba too.  
    #  self.data['angle'] = (2 * self.data['angle']) / 258
    if unit == 'radians':
      self.data['angle'] = self.data['angle'] * math.pi / 180 ### 

  def decode_bool(self, name, byte):
    """Decode 'byte' as a bool and map it to 'name'."""
    self.data[name] = bool(struct.unpack('B', byte)[0])

  # NOTE(damonkohler): We specify the low byte first to make it easier when
  # popping bytes off a list.
  def decode_unsigned_short(self, name, low, high):
    """Map an unsigned short from a 'high' and 'low' bytes to 'name'."""
    self.data[name] = struct.unpack('>H', high + low)[0]
    #print name, self.data[name]

  def decode_short(self, name, low, high):
    """Map a short from a 'high' and 'low' bytes to 'name'."""
    self.data[name] = struct.unpack('>h', high + low)[0]

  def decode_byte(self, name, byte):
    """Map signed 'byte' to 'name'."""
    self.data[name] = struct.unpack('b', byte)[0]

  def decode_unsigned_byte(self, name, byte):
    """Map unsigned 'byte' to 'name'."""
    self.data[name] = struct.unpack('B', byte)[0]


  def store_digital_state(self,byte):
    self.data['user-digital-outputs']=byte

class Roomba(object):

  """Represents a Roomba robot."""

  def __init__(self, tty='/dev/ttyUSB0'):
    self.tty = tty
    self.sci = SerialCommandInterface(tty, 57600)
    self.sci.add_opcodes(ROOMBA_OPCODES)
    self.sensors = RoombaSensors(self)
    self.safe = True

  def change_baud_rate(self, baud_rate):
    """Sets the baud rate in bits per second (bps) at which SCI commands and
    data are sent according to the baud code sent in the data byte.

    The default baud rate at power up is 57600 bps. (See Serial Port Settings,
    above.) Once the baud rate is changed, it will persist until Roomba is
    power cycled by removing the battery (or until the battery voltage falls
    below the minimum required for processor operation). You must wait 100ms
    after sending this command before sending additional commands at the new
    baud rate. The SCI must be in passive, safe, or full mode to accept this
    command. This command puts the SCI in passive mode.

    """
    if baud_rate not in BAUD_RATES:
      raise DriverError('Invalid baud rate specified.')
    self.sci.baud(baud_rate)
    self.sci = SerialCommandInterface(self.tty, baud_rate)

  def passive(self):
    """Put the robot in passive mode."""
    self.sci.start()
    time.sleep(0.5)

  def control(self):
    """Start the robot's SCI interface and place it in safe mode."""
    self.passive()
    self.sci.control()  # Also puts the Roomba in to safe mode.
    if not self.safe:
      self.sci.full()
    else:
      self.passive()
    time.sleep(0.5)

  def direct_drive(self, velocity_left, velocity_right):
    # Mask integers to 2 bytes.
    vl = int(velocity_left) & 0xffff
    vr = int(velocity_right) & 0xffff
    self.sci.direct_drive(*struct.unpack('4B', struct.pack('>2H', vr, vl)))
    
  def drive(self, velocity, radius):
    """controls Roomba's drive wheels.

    NOTE(damonkohler): The following specification applies to both the Roomba
    and the Turtlebot.

    The Roomba takes four data bytes, interpreted as two 16-bit signed values
    using two's complement. The first two bytes specify the average velocity
    of the drive wheels in millimeters per second (mm/s), with the high byte
    being sent first. The next two bytes specify the radius in millimeters at
    which Roomba will turn. The longer radii make Roomba drive straighter,
    while the shorter radii make Roomba turn more. The radius is measured from
    the center of the turning circle to the center of Roomba.

    A drive command with a positive velocity and a positive radius makes
    Roomba drive forward while turning toward the left. A negative radius
    makes Roomba turn toward the right. Special cases for the radius make
    Roomba turn in place or drive straight, as specified below. A negative
    velocity makes Roomba drive backward.

    Also see drive_straight and turn_in_place convenience methods.

    """
    # Mask integers to 2 bytes.
    velocity = int(velocity) & 0xffff
    radius = int(radius) & 0xffff

    # Pack as shorts to get 2 x 2 byte integers. Unpack as 4 bytes to send.
    # TODO(damonkohler): The 4 unpacked bytes will just be repacked later,
    # that seems dumb to me.
    bytes = struct.unpack('4B', struct.pack('>2H', velocity, radius))
    self.sci.drive(*bytes)

  def stop(self):
    """Set velocity and radius to 0 to stop movement."""
    self.drive(0, 0)

  def slow_stop(self, velocity):
    """Slowly reduce the velocity to 0 to stop movement."""
    velocities = xrange(velocity, VELOCITY_SLOW, -25)
    if velocity < 0:
      velocities = xrange(velocity, -VELOCITY_SLOW, 25)
    for v in velocities:
      self.drive(v, RADIUS_STRAIGHT)
      time.sleep(0.05)
    self.stop()

  def drive_straight(self, velocity):
    """drive in a straight line."""
    self.drive(velocity, RADIUS_STRAIGHT)

  def turn_in_place(self, velocity, direction):
    """Turn in place either clockwise or counter-clockwise."""
    valid_directions = {'cw': RADIUS_TURN_IN_PLACE_CW,
                        'ccw': RADIUS_TURN_IN_PLACE_CCW}
    self.drive(velocity, valid_directions[direction])

  def dock(self):
    """Start looking for the dock and then dock."""
    # NOTE(damonkohler): We should be able to call dock from any mode, however
    # it only seems to work from passive.
    self.sci.start()
    time.sleep(0.5)
    self.sci.force_seeking_dock()


class TurtlebotSensors(RoombaSensors):

  """Handles retrieving and decoding the Turtlebot's sensor data."""

  def _decode_group_packet_6(self, bytes, stamp):
    """Decode sensor group packet 6."""
    #TODO: kwc use struct unpacking on bytestr instead of list
    self.decode_short('requested-left-velocity', bytes.pop(), bytes.pop())  # mm/s
    self.decode_short('requested-right-velocity', bytes.pop(), bytes.pop())  # mm/s
    self.decode_short('requested-radius', bytes.pop(), bytes.pop())  # mm
    self.decode_short('requested-velocity', bytes.pop(), bytes.pop())  # mm/s
    self.decode_unsigned_byte('number-of-stream-packets', bytes.pop())
    self.decode_bool('song-playing', bytes.pop())
    self.decode_unsigned_byte('song-number', bytes.pop())
    self.decode_unsigned_byte('oi-mode', bytes.pop())
    self.decode_unsigned_byte('charging-sources-available', bytes.pop())
    self.decode_unsigned_short('user-analog-input', bytes.pop(), bytes.pop())

    self.decode_unsigned_byte('user-digital-inputs', bytes.pop())
    self.decode_unsigned_short('cliff-right-signal', bytes.pop(), bytes.pop())
    self.decode_unsigned_short(
        'cliff-front-right-signal', bytes.pop(), bytes.pop())
    self.decode_unsigned_short(
        'cliff-front-left-signal', bytes.pop(), bytes.pop())
    self.decode_unsigned_short('cliff-left-signal', bytes.pop(), bytes.pop())
    self.decode_unsigned_short('wall-signal', bytes.pop(), bytes.pop())
    self._decode_group_packet_0(bytes, stamp)

  def get_all(self):
    """Request and decode all available sensor data."""
    bytes, stamp = self.request_packet(6)
    if bytes is not None:
      self._decode_group_packet_6(bytes, stamp)


class Turtlebot(Roomba):

  """Represents a Turtlebot robot."""

  def __init__(self, tty='/dev/ttyUSB0'):
    super(Turtlebot, self).__init__(tty)
    self.sci.add_opcodes(CREATE_OPCODES)
    self.sensors = TurtlebotSensors(self)


  def control(self):
    """Start the robot's SCI interface and place it in safe or full mode."""
    logging.info('sending control opcodes.')
    self.passive()
    if self.safe:
      self.sci.safe()
      self.passive()
    else:
      self.sci.full()
    time.sleep(0.5)

  def power_low_side_drivers(self, drivers):
    """Enable or disable power to low side drivers.

    'drivers' should be a list of booleans indicating which low side drivers
    should be powered.

    """
    assert len(drivers) == 3, 'Expecting 3 low side driver power settings.'
    byte = 0
    for driver, power in enumerate(drivers):
      byte += (2 ** driver) * int(power)
    self.sci.low_side_drivers(byte)

  def set_digital_outputs(self, outputs):
    """Enable or disable digital outputs.
    """
    assert len(outputs) == 3, 'Expecting 3 output states.'
    byte = 0
    for output, state in enumerate(outputs):
      byte += (2 ** output) * int(state)
      self.sensors.data['user-digital-outputs']=byte
    self.sci.digital_outputs(byte)

  def soft_reset(self):
    """Do a soft reset of the Turtlebot."""
    logging.info('sending soft reset.')
    self.sci.soft_reset()
    time.sleep(START_DELAY)
    self.passive()