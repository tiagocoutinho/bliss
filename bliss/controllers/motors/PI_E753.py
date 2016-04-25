from bliss.controllers.motor import Controller
from bliss.common import log as elog
from bliss.controllers.motor import add_axis_method
from bliss.common.axis import AxisState

import pi_gcs
from bliss.comm import tcp

import sys
import time

"""
Bliss controller for ethernet PI E753 piezo controller.
Cyril Guilloud ESRF BLISS  2014-2016
"""


class PI_E753(Controller):

    def __init__(self, name, config, axes, encoders):
        Controller.__init__(self, name, config, axes, encoders)

        self.host = self.config.get("host")
        self.cname = "E753"

    # Init of controller.
    def initialize(self):
        """
        Controller intialization : opens a single socket for all 3 axes.
        """
        self.sock = tcp.Command(self.host, 50000)

    def finalize(self):
        """
        Closes the controller socket.
        """
        # not called at end of device server ??? :(
        # called on a new axis creation ???

        if self.sock:
            self.sock.close()

    # Init of each axis.
    def initialize_axis(self, axis):
        elog.debug("axis initialization")

        '''Closed loop'''
        add_axis_method(axis, self.open_loop, types_info=("None", "None"))
        add_axis_method(axis, self.close_loop, types_info=("None", "None"))

        # To purge controller.
        try:
            self.sock._raw_read()
        except:
            pass

        # Enables the closed-loop.
        self._set_closed_loop(True)

    def initialize_encoder(self, encoder):
        pass

    """
    ON / OFF
    """
    def set_on(self, axis):
        pass

    def set_off(self, axis):
        pass

    def read_position(self, axis):
        _ans = self._get_target_pos()
        elog.debug("read_position = %f" % _ans)
        return _ans

    def read_encoder(self, encoder):
        _ans = self._get_pos()
        elog.debug("read_position measured = %f" % _ans)
        return _ans

    """ VELOCITY """
    def read_velocity(self, axis):
        return self._get_velocity(axis)

    def set_velocity(self, axis, new_velocity):
        elog.debug("set_velocity new_velocity = %f" % new_velocity)
        self.sock.write("VEL 1 %f\n" % new_velocity)
        return self.read_velocity(axis)

    """ STATE """
    def state(self, axis):
        if self._get_closed_loop_status():
            if self._get_on_target_status():
                return AxisState("READY")
            else:
                return AxisState("MOVING")
        else:
            raise RuntimeError("closed loop disabled")

    """ MOVEMENTS """
    def prepare_move(self, motion):
        self._target_pos = motion.target_pos

    def start_one(self, motion):
        elog.debug("start_one target_pos = %f" % self._target_pos)
        self.sock.write("MOV 1 %g\n" % self._target_pos)

    def stop(self, axis):
        # to check : copy of current position into target position ???
        self.sock.write("STP\n")

    """ RAW COMMANDS """
    def raw_write(self, com):
        self.sock.write(com)

    def raw_write_read(self, com):
        return self.sock.write_readline(com)

    def raw_write_readlines(self, com, lines):
        return "\n".join(self.sock.write_readlines("%s\n" % com, lines))

    def get_identifier(self, axis):
        return self.sock.write_readline("IDN?\n")

    """
    E753 specific
    """

    def get_voltage(self, axis):
        """ Returns voltage read from controller."""
        _ans = self.sock.write_readline("SVA?\n")
        _voltage = float(_ans[2:])
        return _voltage

    def set_voltage(self, axis, new_voltage):
        """ Sets Voltage to the controller."""
        self.sock.write("SVA 1 %g\n" % new_voltage)


    def _get_velocity(self, axis):
        """
        Returns velocity taken from controller.
        """
        _ans = self.sock.write_readline("VEL?\n")
        _velocity = float(_ans[2:])

        return _velocity

    def _get_pos(self):
        """
        Returns real position read by capcitive captor.
        """
        _ans = self.sock.write_readline("POS?\n")

        # _ans should looks like "1=-8.45709419e+01\n"
        # "\n" removed by tcp lib.
        _pos = float(_ans[2:])

        return _pos

    """ON TARGET """
    def _get_target_pos(self):
        """
        Returns last target position (setpoint value).
        """
        _ans = self.sock.write_readline("MOV?\n")

        # _ans should looks like "1=-8.45709419e+01\n"
        # "\n" removed by tcp lib.
        _pos = float(_ans[2:])

        return _pos

    def _get_on_target_status(self):
        _ans = self.sock.write_readline("ONT?\n")

        if _ans == "":
            return True
        elif _ans == "":
            return False
        else:
            return -1

    """ CLOSED LOOP"""
    def _get_closed_loop_status(self):
        _ans = self.sock.write_readline("SVO?\n")

        if _ans == "1=1":
            return True
        elif _ans == "1=0":
            return False
        else:
            return -1

    def _set_closed_loop(self, state):
        if state:
            self.sock.write("SVO 1 1\n")
        else:
            self.sock.write("SVO 1 0\n")

    def open_loop(self, axis):
        self._set_closed_loop(False)

    def close_loop(self, axis):
        self._set_closed_loop(True)

    def _get_error(self):
        _error_number = self.sock.write_readline("ERR?\n")
        _error_str = pi_gcs.get_error_str(int(_error_number))

        return (_error_number, _error_str)

    def _stop(self):
        self.sock.write("STP\n")

    def _test_melange(self, sleep_time=0.1):
        ii = 0
        _vel0 = self.sock.write_readline("VEL?\n")
        _ans_pos0 = self.sock.write_readline("POS?\n")[2:]
        _pos0 = int(round(float(_ans_pos0), 2))
        while True:
            time.sleep(sleep_time)
            sys.stdout.write(".")
            _vel = self.sock.write_readline("VEL?\n")
            _ans_pos = self.sock.write_readline("POS?\n")[2:]
            _pos = int(round(float(_ans_pos), 2))
            if _vel != _vel0:
                print "%d VEL = %s " % (ii, _vel)
            if abs(_pos - _pos0) > 1:
                print "%d POS = %s" % (ii, _ans_pos)
            ii = ii + 1

    def get_info(self, axis):
        """
        Returns a set of usefull information about controller.
        Helpful to tune the device.

        Args:
            <axis> : bliss axis
        Returns:
            None
        Raises:
            ?
        """
        (error_nb, err_str) = self._get_error()
        _txt = "      ERR nb=%s  : \"%s\"\n" % (error_nb, err_str)

        _infos = [
            ("Identifier                 ", "IDN?\n"),
            ("Com level                  ", "CCL?\n"),
            ("Firmware name              ", "SEP? 1 0xffff0007\n"),
            ("Firmware version           ", "SEP? 1 0xffff0008\n"),
            ("Firmware description       ", "SEP? 1 0xffff000d\n"),
            ("Firmware date              ", "SEP? 1 0xffff000e\n"),
            ("Firmware developer         ", "SEP? 1 0xffff000f\n"),
            ("Real Position              ", "POS?\n"),
            ("Setpoint Position          ", "MOV?\n"),
            ("Position low limit         ", "SPA? 1 0x07000000\n"),
            ("Position High limit        ", "SPA? 1 0x07000001\n"),
            ("Velocity                   ", "VEL?\n"),
            ("On target                  ", "ONT?\n"),
            ("On target window           ", "SPA? 1 0x07000900\n"),
            ("Target tolerance           ", "SPA? 1 0X07000900\n"),
            ("Settling time              ", "SPA? 1 0X07000901\n"),
            ("Sensor Offset              ", "SPA? 1 0x02000200\n"),
            ("Sensor Gain                ", "SPA? 1 0x02000300\n"),
            ("Motion status              ", "#5\n"),
            ("Closed loop status         ", "SVO?\n"),
            ("Auto Zero Calibration ?    ", "ATZ?\n"),
            ("Analog input setpoint      ", "AOS?\n"),
            ("Low    Voltage Limit       ", "SPA? 1 0x07000A00\n"),
            ("High Voltage Limit         ", "SPA? 1 0x07000A01\n")
        ]


        for i in _infos:
            _txt = _txt + "        %s %s\n" % \
                (i[0], self.sock.write_readline(i[1]))

        _txt = _txt + "        %s    \n%s\n" %  \
            ("Communication parameters",
             "\n".join(self.sock.write_readlines("IFC?\n", 5)))

        _txt = _txt + "        %s    \n%s\n" %  \
            ("Analog setpoints",
             "\n".join(self.sock.write_readlines("TSP?\n", 2)))

        _txt = _txt + "        %s    \n%s\n" %   \
            ("ADC value of analog input",
             "\n".join(self.sock.write_readlines("TAD?\n", 2)))

# ###  TAD[1]==131071  => broken cable ??
# 131071 = pow(2,17)-1

        return _txt
