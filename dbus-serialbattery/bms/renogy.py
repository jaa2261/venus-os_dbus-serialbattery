# -*- coding: utf-8 -*-

from battery import Protection, Battery, Cell
from utils import bytearray_to_string, read_serial_data, unpack_from, logger
from struct import unpack
import struct
import sys
from enum import Enum

FUNCTION = {3: "READ", 6: "WRITE", 131: "READ_ERROR", 134: "WRITE_ERROR"}


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


# Reads data from a list of bytes, and converts to an int
def bytes_to_int(bs, offset, length, signed=False, scale=1):
    ret = 0
    if len(bs) < (offset + length):
        return ret
    if length > 0:
        byteorder = "big"
        start = offset
        end = offset + length
    else:
        byteorder = "little"
        start = offset + length + 1
        end = offset + 1
    return round(int.from_bytes(bs[start:end], byteorder=byteorder, signed=signed) * scale, 2)


# Converts an integer into 2 bytes (16 bits)
# Returns either the first or second byte as an int
def int_to_bytes(i, pos=0):
    if pos == 0:
        return int(format(i, "016b")[:8], 2)
    if pos == 1:
        return int(format(i, "016b")[8:], 2)
    return 0


class RenogyProtection(Protection):
    def __init__(self):
        super(RenogyProtection, self).__init__()
        self.voltage_cell_high = False
        self.voltage_cell_low = False
        self.short = False
        self.IC_inspection = False
        self.software_lock = False

    def set_voltage_cell_high(self, value):
        self.voltage_cell_high = value
        self.cell_imbalance = 2 if self.voltage_cell_low or self.voltage_cell_high else 0

    def set_voltage_cell_low(self, value):
        self.voltage_cell_low = value
        self.cell_imbalance = 2 if self.voltage_cell_low or self.voltage_cell_high else 0

    def set_short(self, value):
        self.short = value
        self.set_cell_imbalance(2 if self.short or self.IC_inspection or self.software_lock else 0)

    def set_ic_inspection(self, value):
        self.IC_inspection = value
        self.set_cell_imbalance(2 if self.short or self.IC_inspection or self.software_lock else 0)

    def set_software_lock(self, value):
        self.software_lock = value
        self.set_cell_imbalance(2 if self.short or self.IC_inspection or self.software_lock else 0)


class Renogy(Battery):
    def __init__(self, port, baud, address):
        super(Renogy, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE

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
    command_other_alarm_info = b"\x13\xFC\x00\x02"  # Registers 5104-5105
    command_status_info = b"\x13\xF2\x00\x04"  # Registers 5106-5109
    # Battery info
    command_serial_number = b"\x13\xF6\x00\x08"  # Registers 5110-5117 (8 byte string)
    command_version_info = b"\x13\xFE\x00\x04"  # Registers 5118-5121 (2 char, 4 char, 2 char)
    command_model = b"\x14\x02\x00\x08"  # Registers 5122-5129 (8 byte string)
    command_firmware_version = b"\x14\x0A\x00\x02"  # Registers 5130-5131 (2 byte string)
    command_manufacturer = b"\x14\x0C\x00\x0A"  # Registers 5132-5139 (10 byte string)
    # BMS warning and protection config
    command_limits = b"\x14\x50\x00\x16"  # Registers 5200-5221
    command_device_id = b"\x14\x67\x00\x01"  # Registers 5200-5221

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
            result = result and self.read_gen_data()
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
        # After successful connection get_settings() will be called to set up the battery
        # Set the current limits, populate cell count, etc
        # Return True if success, False for failure
        result = self.get_cell_count()
        result = result and self.read_status_data()
        result = result and self.read_gen_data()
        result = result and self.read_bms_config()

        if result:
            self.charge_fet = self.status1.charge_MOSFET == State.ON
            self.discharge_fet = self.status1.discharge_MOSFET == State.ON
            # self.balance_fet = self.status1.balance_MOSFET == State.ON

        return result

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure
        result = self.read_soc_data()
        result = result and self.read_cell_data()
        result = result and self.read_temperature_data()
        result = result and self.read_status_data()
        result = result and self.read_alarm_data

        return result

    def get_cell_count(self):
        if self.cell_count is None:
            cc = self.read_serial_data_renogy(self.command_cell_count)
            if cc is False:
                return False
            else:
                self.cell_count = struct.unpack(">H", cc)[0]
        return True

    def read_gen_data(self):
        model = self.read_serial_data_renogy(self.command_model)
        # check if connection success
        if model is False:
            return False
        # may contain null bytes that we don't want
        model_num, _, _ = unpack("16s", model)[0].decode("utf-8").partition("\0")

        manufacturer = self.read_serial_data_renogy(self.command_manufacturer)
        if manufacturer is False:
            self.hardware_version = model_num
        else:
            # may contain null bytes that we don't want
            manufacturer, _, _ = unpack("20s", manufacturer)[0].decode("utf-8").partition("\0")
            self.hardware_version = f"{manufacturer} {model_num}"

        logger.info(self.hardware_version)

        self.get_cell_count()

        for c in range(self.cell_count):
            self.cells.append(Cell(False))

        firmware = self.read_serial_data_renogy(self.command_firmware_version)
        firmware_major, firmware_minor = unpack_from("2s2s", firmware)
        firmware_major = firmware_major.decode("utf-8")
        firmware_minor = firmware_minor.decode("utf-8")
        self.version = float(f"{firmware_major}.{firmware_minor}")

        capacity = self.read_serial_data_renogy(self.command_capacity)
        if capacity is False:
            return False
        self.capacity = unpack(">L", capacity)[0] / 1000.0

        try:
            serial_number = self.read_serial_data_renogy(self.command_serial_number)
            self.serial_number = unpack("16s", serial_number)[0].decode("utf-8")
        except Exception:
            logger.debug(f"serial number: {serial_number}")
            self.serial_number = None
            pass

        data = self.read_serial_data_renogy(self.command_charge_limits)
        if data is False:
            return False

        (charge_voltage_limit, discharge_voltage_limit, charge_current_limit, discharge_current_limit) = unpack(">HHHh", data)
        self.max_battery_voltage_bms = charge_voltage_limit / 10.0
        self.min_battery_voltage_bms = discharge_voltage_limit / 10.0
        self.max_battery_current_bms = charge_current_limit / 100.0
        self.max_battery_discharge_current = discharge_current_limit / 100.0

        data = self.read_serial_data_renogy(self.command_version_info)
        if data is False:
            return False
        (manufacture_version, version_major, version_minor, communication_protocol_version) = unpack(">2s2s2s2s", data)

        self.manufacture_version = manufacture_version.decode("utf-8")
        self.version_major = version_major.decode("utf-8")
        self.version_minor = version_minor.decode("utf-8")
        self.communication_protocol_version = communication_protocol_version.decode("utf-8")
        self.main_line_version = float(f"{version_major}.{version_minor}")
        self.version_info = f"{self.manufacture_version} {self.version_major} {self.version_minor} {self.communication_protocol_version}"

        data = self.read_serial_data_renogy(self.command_device_id)
        if data is False:
            return False
        self.device_id = unpack(">H", data)

        return True

    def read_soc_data(self):
        soc_data = self.read_serial_data_renogy(self.command_soc)
        # check if connection success
        if soc_data is False:
            return False

        current, voltage, capacity_remain = unpack_from(">hhL", soc_data)
        self.capacity_remain = capacity_remain / 1000.0
        self.current = current / 100.0
        self.voltage = voltage / 10.0
        self.soc = (self.capacity_remain / self.capacity) * 100
        return True

    def read_cell_data(self):
        self.get_cell_count()
        cell_volt_data = self.read_serial_data_renogy(self.command_cell_voltages)
        cell_temperature_data = self.read_serial_data_renogy(self.command_cell_temperatures)
        for c in range(self.cell_count):
            try:
                cell_volts = unpack_from(">H", cell_volt_data, c * 2)
                cell_temperature = unpack_from(">H", cell_temperature_data, c * 2)
                if len(cell_volts) != 0:
                    self.cells[c].voltage = cell_volts[0] / 10
                    self.cells[c].temperature = cell_temperature[0] / 10
            except struct.error:
                self.cells[c].voltage = 0
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

        if environment_temperature_count > 0:
            # temperature sensor 1 in °C (float)
            temperature_1 = environment_temperature_1 / 10.0
            self.to_temperature(1, temperature_1)
            if environment_temperature_count == 2:
                # temperature sensor 2 in °C (float)
                temperature_2 = environment_temperature_2 / 10.0
                self.to_temperature(2, temperature_2)

        if heater_temperature_count > 0:
            # temperature sensor 3 in °C (float)
            temperature_3 = heater_temperature_1 / 10.0
            self.to_temperature(3, temperature_3)
            if heater_temperature_count == 2:
                temperature_4 = heater_temperature_2 / 10.0
                self.to_temperature(4, temperature_4)

        temperature_mos = bms_board_temp
        self.to_temperature(0, temperature_mos)

        if environment_temperature_count == 0:
            self.get_cell_count()

            tempSum = 0
            for i in range(0, self.cell_count):
                tempSum += self.cells[i].temperature

            temperature_1 = tempSum / self.cell_count
            self.to_temperature(1, temperature_1)

            if temperature_mos == 0:
                self.to_temperature(0, temperature_1)

        return True

    def read_alarm_data(self):
        data = self.read_serial_data_renogy(self.command_cell_alarm_info)
        if data is False:
            return False

        (cell_voltage_alarm, cell_temperature_alarm) = unpack(">II", data)
        self.get_cell_count()

        for i in range(0, self.cell_count):
            self.cells[i].voltage_alarm = Alarm[(cell_voltage_alarm >> i * 2) & 3]
            self.cells[i].temperature_alarm = Alarm[(cell_temperature_alarm >> i * 2) & 3]

        data = self.read_serial_data_renogy(self.command_other_alarm_info)
        if data is False:
            return False

        other_alarm = unpack(">I", data)
        self.other_alarm.discharge_current_alarm = Alarm[(other_alarm >> 18) & 3]
        self.other_alarm.charge_current_alarm = Alarm[(other_alarm >> 20) & 3]
        self.other_alarm.heater_temperature_2_alarm = Alarm[(other_alarm >> 22) & 3]
        self.other_alarm.heater_temperature_1_alarm = Alarm[(other_alarm >> 24) & 3]
        self.other_alarm.environment_temperature_2_alarm = Alarm[(other_alarm >> 26) & 3]
        self.other_alarm.environment_temperature_1_alarm = Alarm[(other_alarm >> 28) & 3]
        self.other_alarm.bms_board_temperature_alarm = Alarm[(other_alarm >> 30) & 3]

        return True

    def read_status_data(self):
        data = self.read_serial_data_renogy(self.command_cycles)
        if data is False:
            return False

        self.history.charge_cycles = unpack(">H", data)[0]

        data = self.read_serial_data_renogy(self.command_status_info)
        if data is False:
            return False

        (status1, status2, status3, chargeStatus) = unpack(">HHHH", data)

        # Status 1 values
        self.status1.short_circuit = ProtectionState[(status1 >> 0) & 1]
        self.status1.charge_MOSFET = State[(status1 >> 1) & 1]
        self.status1.discharge_MOSFET = State[(status1 >> 2) & 1]
        self.status1.using_battery_module_power = UsingState[(status1 >> 3) & 1]
        self.status1.charge_over_current_2 = ProtectionState[(status1 >> 4) & 1]
        self.status1.discharge_over_current_2 = ProtectionState[(status1 >> 5) & 1]
        self.status1.module_over_voltage = ProtectionState[(status1 >> 6) & 1]
        self.status1.cell_under_voltage = ProtectionState[(status1 >> 7) & 1]
        self.status1.cell_over_voltage = ProtectionState[(status1 >> 8) & 1]
        self.status1.charge_over_current_1 = ProtectionState[(status1 >> 9) & 1]
        self.status1.discharge_over_current_1 = ProtectionState[(status1 >> 10) & 1]
        self.status1.discharge_under_temp = ProtectionState[(status1 >> 11) & 1]
        self.status1.discharge_over_temp = ProtectionState[(status1 >> 12) & 1]
        self.status1.charge_under_temp = ProtectionState[(status1 >> 13) & 1]
        self.status1.charge_over_temp = ProtectionState[(status1 >> 14) & 1]
        self.status1.module_under_voltage = ProtectionState[(status1 >> 15) & 1]

        # Status 2 values
        self.status2.cell_low_voltage = WarningState[(status2 >> 0) & 1]
        self.status2.cell_high_voltage = WarningState[(status2 >> 1) & 1]
        self.status2.module_low_voltage = WarningState[(status2 >> 2) & 1]
        self.status2.module_high_voltage = WarningState[(status2 >> 3) & 1]
        self.status2.charge_low_temp = WarningState[(status2 >> 4) & 1]
        self.status2.charge_high_temp = WarningState[(status2 >> 5) & 1]
        self.status2.discharge_low_temp = WarningState[(status2 >> 6) & 1]
        self.status2.discharge_high_temp = WarningState[(status2 >> 7) & 1]
        self.status2.buzzer = State[(status2 >> 8) & 1]
        self.status2.fully_charged = ChargedState[(status2 >> 11) & 1]
        self.status2.heater_on = State[(status2 >> 13) & 1]
        self.status2.effective_discharge_current = EffectiveState[(status2 >> 14) & 1]
        self.status2.effective_charge_current = EffectiveState[(status2 >> 15) & 1]

        # Status 3 values
        self.get_cell_count()

        for i in range(0, self.cell_count):
            self.status.cell_voltage_state[i] = ErrorState[(status3 >> i) & 1]

        # Charge status
        self.chargeStatus.full_charge_request = ChargeRequest[(chargeStatus >> 3) & 1]
        self.chargeStatus.charge_immediately_1 = ChargeRequest[(chargeStatus >> 4) & 1]
        self.chargeStatus.charge_immediately_2 = ChargeRequest[(chargeStatus >> 5) & 1]
        self.chargeStatus.discharge_enable_request = DischargeEnableRequest[(chargeStatus >> 6) & 1]
        self.chargeStatus.charge_enable_request = ChargeEnableRequest[(chargeStatus >> 7) & 1]

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

        self.max_battery_charge_current = config_charge_high_current_limit / 10.0
        self.max_battery_discharge_current = config_discharge_high_current_limit / 100.0
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
