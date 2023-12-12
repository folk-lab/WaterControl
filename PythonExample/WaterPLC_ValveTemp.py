#!/usr/bin/env python3
from pyModbusTCP.client import ModbusClient
from pyModbusTCP import utils as mbutils

import time

#PLC register 0-1 are valve-on temperature
#register 2-3 are valve-off delta temperature
#i.e. valve-on temp = 18 and valve-off delta = 3 will turn on at 18 and off at 15

valve_on = 20.0
valve_off_delta = 3.0

plc_ip = '192.168.1.20'

class FloatModbusClient(ModbusClient):
    """A ModbusClient class with float support."""

    def read_float(self, address, number=1):
        """Read float(s) with read holding registers."""
        reg_l = self.read_holding_registers(address, number * 2)
        if reg_l:
            return [mbutils.decode_ieee(f) for f in mbutils.word_list_to_long(reg_l)]
        else:
            return None

    def write_float(self, address, floats_list):
        """Write float(s) with write multiple registers."""
        b32_l = [mbutils.encode_ieee(f) for f in floats_list]
        b16_l = mbutils.long_list_to_word(b32_l)
        return self.write_multiple_registers(address, b16_l)


try:
    waterplc = FloatModbusClient(host= plc_ip, port=502, auto_close=False, auto_open=True) 
except ValueError:
    print("Error with host or port params")

print("Existing valve setpoints:")
print(waterplc.read_float(0))
print(waterplc.read_float(2))

waterplc.write_float(0, [valve_on])
waterplc.write_float(2, [valve_off_delta])

print("New valve setpoints:")
print(waterplc.read_float(0))
print(waterplc.read_float(2))
waterplc.close()
