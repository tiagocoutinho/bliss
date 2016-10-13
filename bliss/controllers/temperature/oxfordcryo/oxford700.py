"""
Oxford 700 Series Cryostream, acessible via serial line

yml configuration example:
name: cryo
class: oxford700
SLdevice: "rfc2217://lid30b2:28003"       #serial line name

 - Chars have a size of 1 byte, shorts have a size of 2 bytes.
 - All temperatures are in centi-Kelvin, i.e. 80 K is reported as 8000
 - Command packets are small, variable length packets sent to the Cryostream.
   If the packet contains a valid command, then Cryostream will immediately
   act upon that command, potentially over-writing any existing command and
   starting the gas flow if necessary.
 - In order to determine whether a command has been received and acted upon
   the status packets should be monitored. There is no handshake!!
 - The controller issues status packets of a fixed length at 1 second
   intervals.
"""
from bliss.comm.serial import Serial
import math
import time
import numpy
from bliss.controllers.temperature.oxfordcryo.oxfordcryo import StatusPacket, CSCOMMAND, splitBytes
import logging

""" TempController import """
from bliss.controllers.temp import Controller
from bliss.common.temperature import Output
from bliss.common import log
from bliss.common.utils import object_method, object_method_type
from bliss.common.utils import object_attribute_get, object_attribute_type_get
from bliss.common.utils import object_attribute_set, object_attribute_type_set


class OxfordCryostream:
    """
    OXCRYO_ALARM = {0:"No Alarms",
                 1:"Stop button has been pressed",
                 2:"Stop command received",
                 3:"End phase complete",
                 4:"Purge phase complete",
                 5:"Temperature error > 5 K",
                 6:"Back pressure > 0.5 bar",
                 7:"Evaporator reduction at maximum",
                 8:"Self-check failed",
                 9:"Gas flow < 2 l/min",
                 10:"Temperature error >25 K",
                 11:"Sensor detects wrong gas type",
                 12:"Unphysical temperature reported",
                 13:"Suct temperature out of range",
                 14:"Invalid ADC reading",
                 15:"Degradation of power supply",
                 16:"Heat sink overheating",
                 17:"Power supply overheating",
                 18:"Power failure",
                 19:"Refrigerator stage too cold",
                 20:"Refrigerator stage failed to reach base in time",
                 21:"Cryodrive is not responding ",
                 22:"Cryodrive reports an error ",
                 23:"No nitrogen available ",
                 24:"No helium available ",
                 25:"Vacuum gauge is not responding ",
                 26:"Vacuum is out of range ",
                 27:"RS232 communication error ",
                 28:"Coldhead temp > 315 K ",
                 29:"Coldhead temp > 325 K ",
                 30:"Wait for End to complete ",
                 31:"Do not open the cryostat ",
                 32:"Disconnect Xtal sensor ",
                 33:"Cryostat is open ",
                 34:"Cryostat open for more than 10 min ",
                 35:"Sample temp > 320 K ",
                 36:"Sample temp > 325 K"}
    """

    def __init__(self, port=None, debug=False):
        #RS232 settings: 9600 baud, 8 bits, no parity, 1 stop bit
        self.serial = Serial(port, baudrate=9600, eol='\r')

    def __del__(self):
        self.serial.close()

    def exit(self):
        self.serial.close()

    def restart(self):
        """Restart a Cryostream which has shutdown
           Returns:
              None
        """
        self.send_cmd(2, CSCOMMAND.RESTART)

    def purge(self):
        """Warm up the Coldhead as quickly as possible
           Returns:
              None
        """
        self.send_cmd(2,CSCOMMAND.PURGE)

    def stop(self):
        """Immediately halt the Cryostream Cooler,turning off the pump and
           all the heaters - used for emergency only

           Returns:
              None
        """
        self.send_cmd(2,CSCOMMAND.STOP)

    def hold(self):
        """Maintain temperature fixed indefinitely, until start issued.
           Returns:
              None
        """
        self.send_cmd(2,CSCOMMAND.HOLD)

    def turbo(self, off):
        """Switch on/off the turbo gas flow
           Args:
              on (bool): True when turbo is on (gas flow 10 l/min)
           Returns:
              None
        """
        self.send_cmd(3, CSCOMMAND.TURBO, int(off))

    def cool(self, temp=None):
        """Make gas temperature decrease to a set value as quickly as possible
           Args:
              temp (float): final temperature [K]
           Returns:
              (float): current gas temperature setpoint
        """
        if temp:
            temp = int(temp*100)
            self.send_cmd(4, CSCOMMAND.COOL, temp)
        else:
            self.update_cmd()
            return self.statusPacket.gas_set_point

    def plat(self, duration=None):
        """Maintain temperature fixed for a certain time.
           Args:
              duration (int): time [minutes]
           Returns:
              (int): remaining time [minutes]
        """
        if duration:
            duration = int(duration)
            self.send_cmd(4, CSCOMMAND.COOL, duration)
        else:
            self.update_cmd()
            return self.statusPacket.remaining

    def end(self, rate):
        """System shutdown with Ramp Rate to go back to temperature of 300K

           Args:
              rate (int): ramp rate [K/hour]
           Returns:
              None
        """
        self.send_cmd(4, CSCOMMAND.END, rate)

    def ramp(self, rate=None, temp=None):
        """Change gas temperature to a set value at a controlled rate
           Args:
              rate (int): ramp rate [K/hour], values 1 to 360
              temp (float): target temperature [K]
           Returns:
              (float, float): current ramp rate [K/hour], target temperature [K]
        """
        if rate and temp:
            temp = int(temp * 100) #transfering to centi-Kelvin
            self.send_cmd(6, CSCOMMAND.RAMP, int(rate), temp)
        else:
            self.update_cmd()
            return self.statusPacket.ramp_rate, self.statusPacket.target_temp

    def read_temperature(self):
        self.update_cmd()
        return self.statusPacket.gas_temp

    def read_ramp_rate(self):
        self.update_cmd()
        return self.statusPacket.ramp_rate

    def send_cmd(self, size, command, *args):
        """Create a command packet and write it to the controller
           Args:
              size (int): The variable size of the command packet
              command (int): The command packet identifier (command name)
              args: Possible variable number of parameters
           Returns:
              None
        """
        data = [chr(size), chr(command)]
        if size == 3:
            data.append(str(args[0]))
        elif size > 3:
            HIBYTE, LOBYTE = splitBytes(args[0])
            data.append(HIBYTE)
            data.append(LOBYTE)
            try:
                HIBYTE, LOBYTE = splitBytes(args[1])
                data.append(HIBYTE)
                data.append(LOBYTE)
            except Exception:
                pass
        dataStr = ''.join(data)
        self.serial.write(dataStr)

    def update_cmd(self):
        """Read the controller and update all the parameter variables
           Args:
              None
           Returns:
              None
        """     
        #flush the buffer to clean old status packages
        self.serial.flush()

        #read the data
        data = self.serial.read(32, 10)

        #check if data
        if not data.__len__():
            raise RuntimeError('Invalid answer from Cryostream')

        if data.__len__() != 32:
            data = ""
            data = self.serial.read(32, 10)
            print data.__len__()

        data = map(ord,data)
        if data[0] == 32:
            self.statusPacket = StatusPacket(data)
        else:
            print "Flushing serial line to start from skratch"
            self.serial.flush()


class oxford700(Controller):
    def __init__(self, config, *args):
        Controller.__init__(self, config, *args)
        self._oxford = OxfordCryostream(config["SLdevice"])

    def initialize_output(self,toutput):
        self.__ramp_rate = None
        self.__set_point = None

    def read_output(self, toutput):
        return self._oxford.read_temperature()

    def start_ramp(self, toutput, sp, **kwargs):
        try:
            rate = int(kwargs.get("rate", self.__ramp_rate))
        except TypeError:
            raise RuntimeError("Cannot start ramping, ramp rate not set")
        self._oxford.ramp(rate, sp)

    def set_ramprate(self, toutput, rate):
        self.ramp_rate = rate

    def read_ramprate(self, toutput):
        self.__ramp_rate, self.__set_point = self._oxford.ramp()
        return self.__ramp_rate
        
    def set(self, toutput, sp, **kwargs):
        return self._oxford.cool(sp)
    
    def get_setpoint(self, toutput):
        self.__set_point = self._oxford.cool()
        return self.__set_point

    def state_output(self,toutput):
        self._oxford.update_cmd()
        state = str(self._oxford.statusPacket.run_mode)+" "+str(self._oxford.statusPacket.phase)
        return state


    @object_method_type(types_info=("bool", "None"), type=Output)
    def turbo(self, toutput, off):
        self._oxford.turbo(off)

    def read_status(self):
        self._oxford.update_cmd()
        #data.append(self._oxford.statusPacket.gas_temp)
        #data.append(self._oxford.statusPacket.gas_error)
        #data.append(self._oxford.statusPacket.cryo_state)
        #data.append(self._oxford.statusPacket.evap_temp)
        #data.append(self._oxford.statusPacket.ramp_rate)
        #return data
        return self._oxford.statusPacket



if __name__ == '__main__':
    cryo = OxfordCryostream("rfc2217://lid292:28003")

    for i in range(100):
        print cryo.read_temperature()
        #time.sleep(10)
    
    print cryo.ramp()

    #cryo.turbo(True)
