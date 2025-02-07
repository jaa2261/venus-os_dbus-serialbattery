# -*- coding: utf-8 -*-
from typing import List

from battery import Battery, Cell
from utils import bytearray_to_string, read_serial_data, unpack_from, logger
from struct import unpack
import struct
import sys
from enum import Enum


class ProtectionCodes(Enum):
    ALARM = 2
    WARNING = 1
    OK = 0


class Function(Enum):
    READ = 0x03
    WRITE = 0x06


class Status(Enum):
    READ_OK = 0x03
    WRITE_OK = 0x06
    READ_ERROR = 0x83
    WRITE_ERROR = 0x86


class Error(Enum):
    UNSUPPORTED_FUNCTION_CODE = 0x01
    ADDRESS_OR_LENGTH_WRONG = 0x02
    DATA_TOO_LONG = 0x03
    CLIENT_ERROR = 0x04
    DATA_CHECK_WRONG = 0x05


class Alarm(Enum):
    NONE = 0
    BELOW = 1
    ABOVE = 2
    OTHER = 3


class ProtectionState(Enum):
    NORMAL = 0
    TRIGGER = 1


class WarningState(Enum):
    NORMAL = 0
    TRIGGER = 1


class UsingState(Enum):
    NOT = 0
    USING = 1


class State(Enum):
    OFF = 0
    ON = 1


class ChargedState(Enum):
    NORMAL = 0
    FULL = 1


class EffectiveState(Enum):
    NORMAL = 0
    EFFECTIVE = 1


class ErrorState(Enum):
    NORMAL = 0
    ERROR = 1


class ChargeRequest(Enum):
    NORMAL = 0
    YES = 1


class ChargeEnableRequest(Enum):
    NORMAL = 0
    REQUEST_STOP_CHARGE = 1


class DischargeEnableRequest(Enum):
    NORMAL = 0
    REQUEST_STOP_DISCHARGE = 1


# fmt: off
CRC16_LOW_BYTES = (
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
)

CRC16_HIGH_BYTES = (
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
)
# fmt: on


# Calculate CRC-16 for Modbus
def crc16_modbus(data: bytes):
    crc_high = 0xFF
    crc_low = 0xFF

    for byte in data:
        index = crc_high ^ int(byte)
        crc_high = crc_low ^ CRC16_HIGH_BYTES[index]
        crc_low = CRC16_LOW_BYTES[index]

    return bytes([crc_high, crc_low])


class RenogyCell(Cell):
    def __init__(self, balance: bool = None):
        super(RenogyCell, self).__init__(balance)
        self.temperature: float = None
        self.voltage_alarm = Alarm.NONE
        self.temperature_alarm = Alarm.NONE


class Renogy(Battery):
    def __init__(self, port, baud, address):
        super(Renogy, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE
        self.cells: List[RenogyCell] = []
        self.battery_data = {}

        # The RBT100LFP12SH-G1 uses 0xF7, another battery uses 0x30
        self.address = address

    BATTERYTYPE = "Renogy"
    LENGTH_CHECK = 4
    LENGTH_POS = 2

    # command bytes [Address field][Function code (03 = Read register)]
    #                   [Register Address (2 bytes)][Data Length (2 bytes)][CRC (2 bytes little endian)]
    command_read = b"\x03"
    # Core data = voltage, temp, current, soc
    command_cell_count = b"\x13\x88\x00\x01"  # Register  5000
    command_cell_voltages = b"\x13\x89\x00\x10"  # Registers 5001-5016
    command_cell_temperatures = b"\x13\x9A\x00\x10"  # Registers 5018-5033
    command_bms_temperatures = b"\x13\xAB\x00\x07"  # Registers  5035-5041
    command_soc = b"\x13\xB2\x00\x04"  # Registers 5042-5045 (amps, volts, soc as long)
    command_capacity = b"\x13\xB6\x00\x02"  # Registers 5046-5047 (long)
    command_cycles = b"\x13\xB8\x00\x01"  # Registers 5048
    command_charge_limits = b"\x13\xB9\x00\x04"  # Registers 5049-5052
    command_cell_alarm_info = b"\x13\xEC\x00\x04"  # Registers 5100-5103
    command_other_alarm_info = b"\x13\xF0\x00\x02"  # Registers 5104-5105
    command_status_info = b"\x13\xF2\x00\x04"  # Registers 5106-5109
    # Battery info
    command_serial_number = b"\x13\xF6\x00\x08"  # Registers 5110-5117 (8 byte string)
    command_version_info = b"\x13\xFE\x00\x04"  # Registers 5118-5121 (2 char, 4 char, 2 char)
    command_model = b"\x14\x02\x00\x08"  # Registers 5122-5129 (8 byte string)
    command_firmware_version = b"\x14\x0A\x00\x02"  # Registers 5130-5131 (2 byte string)
    command_manufacturer = b"\x14\x0C\x00\x0A"  # Registers 5132-5139 (10 byte string)
    # BMS warning and protection config
    command_limits = b"\x14\x50\x00\x16"  # Registers 5200-5221
    command_device_id = b"\x14\x67\x00\x01"  # Registers 5223

    def unique_identifier(self) -> str:
        """
        Override the unique_identifier method to return the serial number if it exists, else restore default method.
        """
        if self.serial_number is not None:
            return self.serial_number
        else:
            return self.port + ("__" + bytearray_to_string(self.address).replace("\\", "0") if self.address is not None else "")

    def test_connection(self):
        """
        call a function that will connect to the battery, send a command and retrieve the result.
        The result or call should be unique to this BMS. Battery name or version, etc.
        Return True if success, False for failure
        """
        result = False
        try:
            # get settings to check if the data is valid and the connection is working
            result = self.get_settings()
            # get the rest of the data to be sure, that all data is valid and the correct battery type is recognized
            # only read next data if the first one was successful, this saves time when checking multiple battery types
            result = result and self.refresh_data()
        except Exception:
            (
                exception_type,
                exception_object,
                exception_traceback,
            ) = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")
            result = False

        return result

    def get_settings(self):
        """
        After successful connection get_settings() will be called to set up the battery
        Set all values that only need to be set once
        Return True if success, False for failure
        """
        try:
            result = self.get_cell_count()

            # init the cell array once
            if len(self.cells) == 0:
                for _ in range(self.cell_count):
                    self.cells.append(RenogyCell(False))

            result = result and self.read_gen_data()
            result = result and self.read_bms_config()

            # MANDATORY values to set
            # does not need to be in this function, but has to be set at least once
            # could also be read in a function that is called from refresh_data()
            #
            # if not available from battery, then add a section in the `config.default.ini`
            # under ; --------- BMS specific settings ---------
            if result:
                # number of connected cells (int)
                self.cell_count = self.battery_data["cell_count"]

                # capacity of the battery in ampere hours (float)
                self.capacity = self.battery_data["capacity"]

                # OPTIONAL values to set
                # does not need to be in this function
                # could also be read in a function that is called from refresh_data()

                # maximum charge current in amps (float)
                self.max_battery_charge_current = self.battery_data["charge_current_limit"]

                # maximum discharge current in amps (float)
                self.max_battery_discharge_current = self.battery_data["discharge_current_limit"]

                # custom field, that the user can set in the BMS software (str)
                self.custom_field = self.battery_data["device_id"]

                # maximum voltage of the battery in V (float)
                self.max_battery_voltage_bms = self.battery_data["charge_voltage_limit"]

                # minimum voltage of the battery in V (float)
                self.min_battery_voltage_bms = self.battery_data["discharge_voltage_limit"]

                # hardware version of the BMS (str)
                self.hardware_version = f"{self.battery_data["manufacturer_name"]} {self.battery_data['battery_name']}"  # noqa: E999

                # serial number of the battery (str)
                self.serial_number = self.battery_data["serial_number"]

        except Exception:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")
            result = False

        return result

    def refresh_data(self):
        """
        call all functions that will refresh the battery data.
        This will be called for every iteration (1 second)
        Return True if success, False for failure
        """
        try:
            result = self.read_status_data()
        except Exception:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")
            result = False

        return result

    def read_status_data(self):
        # read the status data
        try:
            result = self.read_soc_data()
            result = result and self.read_cell_data()
            result = result and self.read_temperature_data()
            result = result and self.read_alarm_data()
            result = result and self.read_protection_data()

            if result:

                for i in range(0, self.cell_count):
                    self.cells[i].voltage = self.battery_data[f"cell_voltage_{i}"]
                    self.cells[i].temperature = self.battery_data[f"cell_temp_{i}"]
                    self.cells[i].voltage_alarm = self.battery_data[f"cell_voltage_alarm_{i}"]
                    self.cells[i].voltage_alarm = self.battery_data[f"cell_temperature_alarm_{i}"]

                # voltage of the battery in volts (float)
                self.voltage = self.battery_data["voltage"]

                # current of the battery in amps (float)
                self.current = self.battery_data["current"]

                # remaining capacity of the battery in ampere hours (float)
                # if not available, then it's calculated from the SOC and the capacity
                self.capacity_remain = self.battery_data["remaining_charge"]

                # state of charge in percent (float)
                self.soc = (self.capacity_remain / self.capacity) * 100

                # status of the battery if charging is enabled (bool)
                self.charge_fet = self.battery_data["charge_MOSFET"] == State.ON

                # status of the battery if discharging is enabled (bool)
                self.discharge_fet = self.battery_data["discharge_MOSFET"] == State.ON

                # OPTIONAL values to set
                if self.battery_data["environment_temperature_count"] > 0:
                    # temperature sensor 1 in °C (float)
                    temperature_1 = self.battery_data["environment_temperature_1"] / 10.0
                    self.to_temperature(1, temperature_1)
                    if self.battery_data["environment_temperature_count"] == 2:
                        # temperature sensor 2 in °C (float)
                        temperature_2 = self.battery_data["environment_temperature_2"] / 10.0
                        self.to_temperature(2, temperature_2)

                if self.battery_data["heater_temperature_count"] > 0:
                    # temperature sensor 3 in °C (float)
                    temperature_3 = self.battery_data["heater_temperature_1"] / 10.0
                    self.to_temperature(3, temperature_3)
                    if self.battery_data["heater_temperature_count"] == 2:
                        temperature_4 = self.battery_data["heater_temperature_2"] / 10.0
                        self.to_temperature(4, temperature_4)

                # temperature sensor MOSFET in °C (float)
                temperature_mos = self.battery_data["bms_board_temp"]
                self.to_temperature(0, temperature_mos)

                if self.battery_data["environment_temperature_count"] == 0:
                    self.get_cell_count()

                    tempSum = 0
                    for i in range(0, self.cell_count):
                        tempSum += self.cells[i].temperature

                    temperature_1 = tempSum / self.cell_count
                    self.to_temperature(1, temperature_1)

                    if temperature_mos == 0:
                        self.to_temperature(0, temperature_1)

                # PROTECTION values
                # 2 = alarm, 1 = warningm 0 = ok
                # high battery voltage alarm (int)
                if self.battery_data["module_over_voltage"] == ProtectionState.TRIGGER:
                    self.protection.high_voltage = ProtectionCodes.ALARM
                else:
                    self.protection.high_voltage = ProtectionCodes.OK

                # low battery voltage alarm (int)
                if self.battery_data["module_under_voltage"] == ProtectionState.TRIGGER:
                    self.protection.low_voltage = ProtectionCodes.ALARM
                else:
                    self.protection.low_voltage = ProtectionCodes.OK

                # high cell voltage alarm (int)
                cell_high_voltage_alarm = False
                cell_low_voltage_alarm = False
                cell_high_temp_alarm = False
                cell_low_temp_alarm = False
                for i in range(0, self.cell_count - 1):
                    if self.cells[i].voltage_alarm == Alarm.ABOVE:
                        cell_high_voltage_alarm = True
                    if self.cells[i].voltage_alarm == Alarm.BELOW:
                        cell_low_voltage_alarm = True
                    if self.cells[i].temperature_alarm == Alarm.ABOVE:
                        cell_high_temp_alarm = True
                    if self.cells[i].temperature_alarm == Alarm.BELOW:
                        cell_low_temp_alarm = True

                if self.battery_data["cell_over_voltage"] == ProtectionState.TRIGGER or cell_high_voltage_alarm:
                    self.protection.high_cell_voltage = ProtectionCodes.ALARM
                else:
                    self.protection.high_cell_voltage = ProtectionCodes.OK

                # low cell voltage alarm (int)
                if self.battery_data["cell_under_voltage"] == ProtectionState.TRIGGER or cell_low_voltage_alarm:
                    self.protection.low_cell_voltage = ProtectionCodes.ALARM
                else:
                    self.protection.low_cell_voltage = ProtectionCodes.OK

                # low SOC alarm (int)
                # NB Pure guess about if this is correct
                if self.battery_data["charge_immediately_1"] == ChargeRequest.YES or self.battery_data["charge_immediately_2"] == ChargeRequest.YES:
                    self.protection.low_soc = ProtectionCodes.ALARM
                else:
                    self.protection.low_soc = ProtectionCodes.OK

                # high charge current alarm (int)
                # There are 3 limits: config_charge_over_current_limit, config_charge_over2_current_limit,
                # and config_charge_high_current_limit - not obvious how these relate to these alarms being triggered
                # Possibly charge_over_current_1 gets triggered if current > config_charge_over_current_limit
                # and charge_over_current_2 gets triggered if current > config_charge_over2_current_limit
                # and charge_current_alarm gets triggered if current > config_charge_high_current_limit
                # Maybe if current > config_charge_over_current_limit then you treat that as a warning,
                # and if current > config_charge_over2_current_limit then you treat that as an alarm
                # To be safe I'll assume if any are flagged it's an alarm
                if (
                    self.battery_data["charge_over_current_1"] == ProtectionState.NORMAL
                    or self.battery_data["charge_over_current_2"] == ProtectionState.NORMAL
                    or self.battery_data["charge_current_alarm"] == Alarm.NONE
                ):
                    self.protection.high_charge_current = ProtectionCodes.OK
                if (
                    self.battery_data["charge_over_current_1"] == ProtectionState.TRIGGER
                    or self.battery_data["charge_over_current_2"] == ProtectionState.TRIGGER
                    or self.battery_data["charge_current_alarm"] == Alarm.ABOVE
                ):
                    self.protection.high_charge_current = ProtectionCodes.ALARM

                # high discharge current alarm (int)
                # There are 3 limits: config_discharge_over_current_limit, config_discharge_over2_current_limit,
                # and config_discharge_high_current_limit - not obvious how these relate to these alarms being triggered
                # Possibly discharge_over_current_1 gets triggered if current < config_discharge_over_current_limit
                # and discharge_over_current_2 gets triggered if current < config_discharge_over2_current_limit
                # and discharge_current_alarm gets triggered if current < config_discharge_high_current_limit
                # Maybe if current < config_discharge_over_current_limit then you treat that as a warning,
                # and if current < config_discharge_over2_current_limit then you treat that as an alarm
                # To be safe I'll assume if any are flagged it's an alarm
                if (
                    self.battery_data["discharge_over_current_1"] == ProtectionState.NORMAL
                    or self.battery_data["discharge_over_current_2"] == ProtectionState.NORMAL
                    or self.battery_data["discharge_current_alarm"] == Alarm.NONE
                ):
                    self.protection.high_discharge_current = ProtectionCodes.OK

                if (
                    self.battery_data["discharge_over_current_1"] == ProtectionState.TRIGGER
                    or self.battery_data["discharge_over_current_2"] == ProtectionState.TRIGGER
                    or self.battery_data["discharge_current_alarm"] == Alarm.ABOVE
                ):
                    self.protection.high_discharge_current = ProtectionCodes.ALARM

                # cell imbalance alarm (int)
                # self.protection.cell_imbalance = VALUE_FROM_BMS

                # high charge temperature alarm (int)
                if self.battery_data["charge_high_temp"] == WarningState.NORMAL and self.battery_data["charge_over_temp"] == ProtectionState.NORMAL:
                    self.protection.high_charge_temperature = ProtectionCodes.OK
                if self.battery_data["charge_high_temp"] == WarningState.TRIGGER:
                    self.protection.high_charge_temperature = ProtectionCodes.WARNING
                if self.battery_data["charge_over_temp"] == ProtectionState.TRIGGER or cell_high_temp_alarm:
                    self.protection.high_charge_temperature = ProtectionCodes.ALARM

                # low charge temperature alarm (int)
                if self.battery_data["charge_low_temp"] == WarningState.NORMAL and self.battery_data["charge_under_temp"] == ProtectionState.NORMAL:
                    self.protection.high_charge_temperature = ProtectionCodes.OK
                if self.battery_data["charge_low_temp"] == WarningState.TRIGGER:
                    self.protection.high_charge_temperature = ProtectionCodes.WARNING
                if self.battery_data["charge_under_temp"] == ProtectionState.TRIGGER or cell_low_temp_alarm:
                    self.protection.high_charge_temperature = ProtectionCodes.ALARM

                # high temperature alarm (int)
                if (
                    self.battery_data["bms_board_temperature_alarm"] == Alarm.ABOVE
                    or self.battery_data["environment_temperature_1_alarm"] == Alarm.ABOVE
                    or self.battery_data["environment_temperature_2_alarm"] == Alarm.ABOVE
                    or self.battery_data["heater_temperature_1_alarm"] == Alarm.ABOVE
                    or self.battery_data["heater_temperature_2_alarm"] == Alarm.ABOVE
                    or cell_high_temp_alarm
                ):
                    self.protection.high_temperature = ProtectionCodes.ALARM
                    # high internal temperature alarm (int)
                    self.protection.high_internal_temperature = ProtectionCodes.ALARM
                else:
                    self.protection.high_temperature = ProtectionCodes.OK

                # low temperature alarm (int)
                if (
                    self.battery_data["bms_board_temperature_alarm"] == Alarm.BELOW
                    or self.battery_data["environment_temperature_1_alarm"] == Alarm.BELOW
                    or self.battery_data["environment_temperature_2_alarm"] == Alarm.BELOW
                    or self.battery_data["heater_temperature_1_alarm"] == Alarm.BELOW
                    or self.battery_data["heater_temperature_2_alarm"] == Alarm.BELOW
                    or cell_low_temp_alarm
                ):
                    self.protection.low_temperature = ProtectionCodes.ALARM
                else:
                    self.protection.low_temperature = ProtectionCodes.OK

                # fuse blown alarm (int)
                if self.battery_data["short_circuit"] == ProtectionState.TRIGGER:
                    self.protection.fuse_blown = ProtectionCodes.ALARM
                    self.protection.internal_failure = ProtectionCodes.ALARM
                else:
                    self.protection.fuse_blown = ProtectionCodes.OK
        except Exception:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")
            result = False

        return result

    def read_cell_data(self):
        self.get_cell_count()

        data = self.read_serial_data_renogy(self.command_cell_voltages)
        if data is False:
            return False

        cell_volts = struct.unpack(">HHHHHHHHHHHHHHHH", data)

        data = self.read_serial_data_renogy(self.command_cell_temperatures)
        if data is False:
            return False

        cell_temps = struct.unpack(">hhhhhhhhhhhhhhhh", data)

        for i in range(0, self.cell_count):
            self.battery_data[f"cell_voltage_{i}"] = cell_volts[i] / 10.0
            self.battery_data[f"cell_temp_{i}"] = cell_temps[i] / 10.0

        return True

    def get_cell_count(self):
        if self.cell_count is None:
            data = self.read_serial_data_renogy(self.command_cell_count)
            if data is False:
                logger.warning("Failed to read cell count")
                self.battery_data["cell_count"] = 0
            else:
                self.battery_data["cell_count"] = struct.unpack(">H", data)[0]
                self.cell_count = self.battery_data["cell_count"]
        return True

    def read_gen_data(self):
        data = self.read_serial_data_renogy(self.command_model)
        # check if connection success
        if data is False:
            return False
        # may contain null bytes that we don't want
        self.battery_data["battery_name"], _, _ = unpack("16s", data)[0].decode("utf-8").partition("\0")

        data = self.read_serial_data_renogy(self.command_manufacturer)
        if data is False:
            self.battery_data["manufacturer_name"] = self.battery_data["battery_name"]
        else:
            # may contain null bytes that we don't want
            self.battery_data["manufacturer_name"], _, _ = unpack("20s", data)[0].decode("utf-8").partition("\0")

        data = self.read_serial_data_renogy(self.command_firmware_version)
        if data is False:
            return False

        firmware_major, firmware_minor = unpack_from("2s2s", data)
        firmware_major = firmware_major.decode("utf-8")
        firmware_minor = firmware_minor.decode("utf-8")
        self.battery_data["software_version"] = float(f"{firmware_major}.{firmware_minor}")

        data = self.read_serial_data_renogy(self.command_capacity)
        if data is False:
            return False
        self.battery_data["capacity"] = unpack(">L", data)[0] / 1000.0

        try:
            data = self.read_serial_data_renogy(self.command_serial_number)
            if data is False:
                return False
            self.battery_data["serial_number"] = unpack("16s", data)[0].decode("utf-8")
        except Exception:
            logger.debug(f"serial number: {data}")
            self.battery_data["serial_number"] = None
            pass

        data = self.read_serial_data_renogy(self.command_charge_limits)
        if data is False:
            return False

        (charge_voltage_limit, discharge_voltage_limit, charge_current_limit, discharge_current_limit) = unpack(">HHHh", data)
        self.battery_data["charge_voltage_limit"] = charge_voltage_limit / 10.0
        self.battery_data["discharge_voltage_limit"] = discharge_voltage_limit / 10.0
        self.battery_data["charge_current_limit"] = charge_current_limit / 100.0
        self.battery_data["discharge_current_limit"] = discharge_current_limit / 100.0

        data = self.read_serial_data_renogy(self.command_version_info)
        if data is False:
            return False
        (manufacture_version, version_major, version_minor, communication_protocol_version) = unpack(">2s2s2s2s", data)

        self.battery_data["manufacture_version"] = manufacture_version.decode("utf-8")
        self.battery_data["version_major"] = version_major.decode("utf-8")
        self.battery_data["version_minor"] = version_minor.decode("utf-8")
        self.battery_data["communication_protocol_version"] = communication_protocol_version.decode("utf-8")
        self.battery_data["main_line_version"] = float(f"{self.battery_data["version_major"]}.{self.battery_data["version_minor"]}")  # noqa: E999
        self.battery_data["version_info"] = (
            f"{self.battery_data["manufacture_version"]} {self.battery_data["main_line_version"]} {self.battery_data["communication_protocol_version"]}"
        )

        data = self.read_serial_data_renogy(self.command_device_id)
        if data is False:
            return False
        self.battery_data["device_id"] = unpack(">H", data)[0]

        return True

    def read_soc_data(self):
        data = self.read_serial_data_renogy(self.command_soc)
        # check if connection success
        if data is False:
            return False

        current, voltage, capacity_remain = unpack_from(">hhL", data)

        self.battery_data["remaining_charge"] = capacity_remain / 1000.0
        self.battery_data["current"] = current / 100.0
        self.battery_data["voltage"] = voltage / 10.0

        return True

    def read_temperature_data(self):
        data = self.read_serial_data_renogy(self.command_bms_temperatures)
        if data is False:
            return False
        (
            bms_board_temp,
            environment_temperature_count,
            environment_temperature_1,
            environment_temperature_2,
            heater_temperature_count,
            heater_temperature_1,
            heater_temperature_2,
        ) = unpack(">HHHHHHH", data)

        self.battery_data["bms_board_temp"] = bms_board_temp / 10.0
        self.battery_data["environment_temperature_count"] = environment_temperature_count
        self.battery_data["environment_temperature_1"] = environment_temperature_1 / 10.0
        self.battery_data["environment_temperature_2"] = environment_temperature_2 / 10.0
        self.battery_data["heater_temperature_count"] = heater_temperature_count
        self.battery_data["heater_temperature_1"] = heater_temperature_1 / 10.0
        self.battery_data["heater_temperature_2"] = heater_temperature_2 / 10.0

        return True

    def read_alarm_data(self):
        data = self.read_serial_data_renogy(self.command_cell_alarm_info)
        if data is False:
            return False

        (self.battery_data["cell_voltage_alarm"], self.battery_data["cell_temperature_alarm"]) = unpack(">II", data)
        self.get_cell_count()

        for i in range(0, self.cell_count):
            self.battery_data[f"cell_voltage_alarm_{i}"] = Alarm((self.battery_data["cell_voltage_alarm"] >> i * 2) & 3)
            self.battery_data[f"cell_temperature_alarm_{i}"] = Alarm((self.battery_data["cell_temperature_alarm"] >> i * 2) & 3)

        data = self.read_serial_data_renogy(self.command_other_alarm_info)
        if data is False:
            return False

        other_alarm = unpack(">I", data)[0]
        self.battery_data["other_alarm"] = other_alarm
        self.battery_data["discharge_current_alarm"] = Alarm((other_alarm >> 18) & 3)
        self.battery_data["charge_current_alarm"] = Alarm((other_alarm >> 20) & 3)
        self.battery_data["heater_temperature_2_alarm"] = Alarm((other_alarm >> 22) & 3)
        self.battery_data["heater_temperature_1_alarm"] = Alarm((other_alarm >> 24) & 3)
        self.battery_data["environment_temperature_2_alarm"] = Alarm((other_alarm >> 26) & 3)
        self.battery_data["environment_temperature_1_alarm"] = Alarm((other_alarm >> 28) & 3)
        self.battery_data["bms_board_temperature_alarm"] = Alarm((other_alarm >> 30) & 3)

        return True

    def read_protection_data(self):
        data = self.read_serial_data_renogy(self.command_cycles)
        if data is False:
            return False

        self.battery_data["cycle_count"] = unpack(">H", data)[0]

        data = self.read_serial_data_renogy(self.command_status_info)
        if data is False:
            return False

        (status1, status2, status3, chargeStatus) = unpack(">HHHH", data)

        # Status 1 values
        self.battery_data["status1"] = status1
        self.battery_data["short_circuit"] = ProtectionState((status1 >> 0) & 1)
        self.battery_data["charge_MOSFET"] = State((status1 >> 1) & 1)
        self.battery_data["discharge_MOSFET"] = State((status1 >> 2) & 1)
        self.battery_data["using_battery_module_power"] = UsingState((status1 >> 3) & 1)
        self.battery_data["charge_over_current_2"] = ProtectionState((status1 >> 4) & 1)
        self.battery_data["discharge_over_current_2"] = ProtectionState((status1 >> 5) & 1)
        self.battery_data["module_over_voltage"] = ProtectionState((status1 >> 6) & 1)
        self.battery_data["cell_under_voltage"] = ProtectionState((status1 >> 7) & 1)
        self.battery_data["cell_over_voltage"] = ProtectionState((status1 >> 8) & 1)
        self.battery_data["charge_over_current_1"] = ProtectionState((status1 >> 9) & 1)
        self.battery_data["discharge_over_current_1"] = ProtectionState((status1 >> 10) & 1)
        self.battery_data["discharge_under_temp"] = ProtectionState((status1 >> 11) & 1)
        self.battery_data["discharge_over_temp"] = ProtectionState((status1 >> 12) & 1)
        self.battery_data["charge_under_temp"] = ProtectionState((status1 >> 13) & 1)
        self.battery_data["charge_over_temp"] = ProtectionState((status1 >> 14) & 1)
        self.battery_data["module_under_voltage"] = ProtectionState((status1 >> 15) & 1)

        # Status 2 values
        self.battery_data["status2"] = status2
        self.battery_data["cell_low_voltage"] = WarningState((status2 >> 0) & 1)
        self.battery_data["cell_high_voltage"] = WarningState((status2 >> 1) & 1)
        self.battery_data["module_low_voltage"] = WarningState((status2 >> 2) & 1)
        self.battery_data["module_high_voltage"] = WarningState((status2 >> 3) & 1)
        self.battery_data["charge_low_temp"] = WarningState((status2 >> 4) & 1)
        self.battery_data["charge_high_temp"] = WarningState((status2 >> 5) & 1)
        self.battery_data["discharge_low_temp"] = WarningState((status2 >> 6) & 1)
        self.battery_data["discharge_high_temp"] = WarningState((status2 >> 7) & 1)
        self.battery_data["buzzer"] = State((status2 >> 8) & 1)
        self.battery_data["fully_charged"] = ChargedState((status2 >> 11) & 1)
        self.battery_data["heater_on"] = State((status2 >> 13) & 1)
        self.battery_data["effective_discharge_current"] = EffectiveState((status2 >> 14) & 1)
        self.battery_data["effective_charge_current"] = EffectiveState((status2 >> 15) & 1)

        # Status 3 values
        self.get_cell_count()
        self.battery_data["status3"] = status3
        for i in range(0, self.cell_count):
            self.battery_data[f"cell_voltage_error_state_{i}"] = ErrorState((status3 >> i) & 1)

        # Charge status
        self.battery_data["chargeStatus"] = chargeStatus
        self.battery_data["full_charge_request"] = ChargeRequest((chargeStatus >> 3) & 1)
        self.battery_data["charge_immediately_1"] = ChargeRequest((chargeStatus >> 4) & 1)
        self.battery_data["charge_immediately_2"] = ChargeRequest((chargeStatus >> 5) & 1)
        self.battery_data["discharge_enable_request"] = DischargeEnableRequest((chargeStatus >> 6) & 1)
        self.battery_data["charge_enable_request"] = ChargeEnableRequest((chargeStatus >> 7) & 1)

        return True

    def read_bms_config(self):
        data = self.read_serial_data_renogy(self.command_limits)
        if data is False:
            return False

        (
            config_cell_over_voltage_limit,
            config_cell_high_voltage_limit,
            config_cell_low_voltage_limit,
            config_cell_under_voltage_limit,
            config_charge_over_temp_limit,
            config_charge_high_temp_limit,
            config_charge_low_temp_limit,
            config_charge_under_temp_limit,
            config_charge_over2_current_limit,
            config_charge_over_current_limit,
            config_charge_high_current_limit,
            config_module_over_voltage_limit,
            config_module_high_voltage_limit,
            config_module_low_voltage_limit,
            config_module_under_voltage_limit,
            config_discharge_over_temp_limit,
            config_discharge_high_temp_limit,
            config_discharge_low_temp_limit,
            config_discharge_under_temp_limit,
            config_discharge_over2_current_limit,
            config_discharge_over_current_limit,
            config_discharge_high_current_limit,
        ) = unpack(">HHHHHHhhHHHHHHHHHhhhhh", data)

        self.battery_data["config_cell_over_voltage_limit"] = config_cell_over_voltage_limit / 10.0
        self.battery_data["config_cell_high_voltage_limit"] = config_cell_high_voltage_limit / 10.0
        self.battery_data["config_cell_low_voltage_limit"] = config_cell_low_voltage_limit / 10.0
        self.battery_data["config_cell_under_voltage_limit"] = config_cell_under_voltage_limit / 10.0
        self.battery_data["config_charge_over_temp_limit"] = config_charge_over_temp_limit / 10.0
        self.battery_data["config_charge_high_temp_limit"] = config_charge_high_temp_limit / 10.0
        self.battery_data["config_charge_low_temp_limit"] = config_charge_low_temp_limit / 10.0
        self.battery_data["config_charge_under_temp_limit"] = config_charge_under_temp_limit / 10.0
        self.battery_data["config_charge_over2_current_limit"] = config_charge_over2_current_limit / 100.0
        self.battery_data["config_charge_over_current_limit"] = config_charge_over_current_limit / 100.0
        self.battery_data["config_charge_high_current_limit"] = config_charge_high_current_limit / 100.0
        self.battery_data["config_module_over_voltage_limit"] = config_module_over_voltage_limit / 10.0
        self.battery_data["config_module_high_voltage_limit"] = config_module_high_voltage_limit / 10.0
        self.battery_data["config_module_low_voltage_limit"] = config_module_low_voltage_limit / 10.0
        self.battery_data["config_module_under_voltage_limit"] = config_module_under_voltage_limit / 10.0
        self.battery_data["config_discharge_over_temp_limit"] = config_discharge_over_temp_limit / 10.0
        self.battery_data["config_discharge_high_temp_limit"] = config_discharge_high_temp_limit / 10.0
        self.battery_data["config_discharge_low_temp_limit"] = config_discharge_low_temp_limit / 10.0
        self.battery_data["config_discharge_under_temp_limit"] = config_discharge_under_temp_limit / 10.0
        self.battery_data["config_discharge_over2_current_limit"] = config_discharge_over2_current_limit / 100.0
        self.battery_data["config_discharge_over_current_limit"] = config_discharge_over_current_limit / 100.0
        self.battery_data["config_discharge_high_current_limit"] = config_discharge_high_current_limit / 100.0
        return True

    def generate_command(self, command):
        data = []

        data.append(int(self.address, 16))
        data.append(int.from_bytes(self.command_read[0:1], byteorder="big"))
        data.append(int.from_bytes(command[0:1], byteorder="big"))
        data.append(int.from_bytes(command[1:2], byteorder="big"))
        data.append(int.from_bytes(command[2:3], byteorder="big"))
        data.append(int.from_bytes(command[3:4], byteorder="big"))

        crc = crc16_modbus(bytes(data))
        data.append(crc[0])
        data.append(crc[1])
        logger.info(f"create_request_payload=> {data}")

        req = bytearray(data)
        logger.info(f"create_request_payload=> {req}")
        return req

    def read_serial_data_renogy(self, command):
        # use the read_serial_data() function to read the data and then do BMS spesific checks (crc, start bytes, etc)
        data = read_serial_data(
            self.generate_command(command),
            self.port,
            self.baud_rate,
            self.LENGTH_POS,
            self.LENGTH_CHECK,
        )

        return self.validate_packet(data)

    @staticmethod
    def validate_packet(data):
        if data is False:
            return False

        device, status, payload_length = unpack_from("BBB", data)
        logger.debug("bytearray: " + bytearray_to_string(data))
        logger.debug("Device: " + str(device))
        logger.debug("Status: " + Status(status).name)
        logger.debug("Payload Length: " + str(payload_length))

        chk_sum = unpack_from(">H", data, payload_length + 3)
        logger.debug("Checksum: " + str(chk_sum[0]))

        if Status.READ_OK != Status(status):
            # In error states the payload field contains the error code
            logger.warning(f">>> WARN: BMS rejected request. Status {Status(status).name} Error: {Error(payload_length).name}")
            return False

        length = len(data)
        if length != payload_length + 5:
            logger.error(">>> ERROR: BMS send insufficient data. Received " + str(len(data)) + " expected " + str(payload_length + 7))
            return False

        sent_chk_sum = chk_sum[0]
        received_chk_sum = int.from_bytes(crc16_modbus(data[0:-2]))

        if sent_chk_sum != received_chk_sum:
            logger.error(">>> ERROR: Invalid checksum.")
            return False

        payload = data[3 : payload_length + 3]

        return payload
