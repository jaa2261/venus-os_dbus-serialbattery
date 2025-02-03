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


def checksum(payload):
    crc = calc_crc(payload)
    logger.debug("CRC: " + bytearray_to_string(crc))
    sumpayload = sum(payload)
    result = (0x10000 - sumpayload) % 0x10000
    return result


def calc_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


CRC16_LOW_BYTES = (
    0x00,
    0xC0,
    0xC1,
    0x01,
    0xC3,
    0x03,
    0x02,
    0xC2,
    0xC6,
    0x06,
    0x07,
    0xC7,
    0x05,
    0xC5,
    0xC4,
    0x04,
    0xCC,
    0x0C,
    0x0D,
    0xCD,
    0x0F,
    0xCF,
    0xCE,
    0x0E,
    0x0A,
    0xCA,
    0xCB,
    0x0B,
    0xC9,
    0x09,
    0x08,
    0xC8,
    0xD8,
    0x18,
    0x19,
    0xD9,
    0x1B,
    0xDB,
    0xDA,
    0x1A,
    0x1E,
    0xDE,
    0xDF,
    0x1F,
    0xDD,
    0x1D,
    0x1C,
    0xDC,
    0x14,
    0xD4,
    0xD5,
    0x15,
    0xD7,
    0x17,
    0x16,
    0xD6,
    0xD2,
    0x12,
    0x13,
    0xD3,
    0x11,
    0xD1,
    0xD0,
    0x10,
    0xF0,
    0x30,
    0x31,
    0xF1,
    0x33,
    0xF3,
    0xF2,
    0x32,
    0x36,
    0xF6,
    0xF7,
    0x37,
    0xF5,
    0x35,
    0x34,
    0xF4,
    0x3C,
    0xFC,
    0xFD,
    0x3D,
    0xFF,
    0x3F,
    0x3E,
    0xFE,
    0xFA,
    0x3A,
    0x3B,
    0xFB,
    0x39,
    0xF9,
    0xF8,
    0x38,
    0x28,
    0xE8,
    0xE9,
    0x29,
    0xEB,
    0x2B,
    0x2A,
    0xEA,
    0xEE,
    0x2E,
    0x2F,
    0xEF,
    0x2D,
    0xED,
    0xEC,
    0x2C,
    0xE4,
    0x24,
    0x25,
    0xE5,
    0x27,
    0xE7,
    0xE6,
    0x26,
    0x22,
    0xE2,
    0xE3,
    0x23,
    0xE1,
    0x21,
    0x20,
    0xE0,
    0xA0,
    0x60,
    0x61,
    0xA1,
    0x63,
    0xA3,
    0xA2,
    0x62,
    0x66,
    0xA6,
    0xA7,
    0x67,
    0xA5,
    0x65,
    0x64,
    0xA4,
    0x6C,
    0xAC,
    0xAD,
    0x6D,
    0xAF,
    0x6F,
    0x6E,
    0xAE,
    0xAA,
    0x6A,
    0x6B,
    0xAB,
    0x69,
    0xA9,
    0xA8,
    0x68,
    0x78,
    0xB8,
    0xB9,
    0x79,
    0xBB,
    0x7B,
    0x7A,
    0xBA,
    0xBE,
    0x7E,
    0x7F,
    0xBF,
    0x7D,
    0xBD,
    0xBC,
    0x7C,
    0xB4,
    0x74,
    0x75,
    0xB5,
    0x77,
    0xB7,
    0xB6,
    0x76,
    0x72,
    0xB2,
    0xB3,
    0x73,
    0xB1,
    0x71,
    0x70,
    0xB0,
    0x50,
    0x90,
    0x91,
    0x51,
    0x93,
    0x53,
    0x52,
    0x92,
    0x96,
    0x56,
    0x57,
    0x97,
    0x55,
    0x95,
    0x94,
    0x54,
    0x9C,
    0x5C,
    0x5D,
    0x9D,
    0x5F,
    0x9F,
    0x9E,
    0x5E,
    0x5A,
    0x9A,
    0x9B,
    0x5B,
    0x99,
    0x59,
    0x58,
    0x98,
    0x88,
    0x48,
    0x49,
    0x89,
    0x4B,
    0x8B,
    0x8A,
    0x4A,
    0x4E,
    0x8E,
    0x8F,
    0x4F,
    0x8D,
    0x4D,
    0x4C,
    0x8C,
    0x44,
    0x84,
    0x85,
    0x45,
    0x87,
    0x47,
    0x46,
    0x86,
    0x82,
    0x42,
    0x43,
    0x83,
    0x41,
    0x81,
    0x80,
    0x40,
)

CRC16_HIGH_BYTES = (
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x01,
    0xC0,
    0x80,
    0x41,
    0x00,
    0xC1,
    0x81,
    0x40,
)


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
    command_cell_voltages = b"\x13\x89\x00\x04"  # Registers 5001-5004
    command_cell_temperatures = b"\x13\x9A\x00\x04"  # Registers 5018-5021
    command_total_voltage = b"\x13\xB3\x00\x01"  # Register  5043
    command_bms_temperature_1 = b"\x13\xAD\x00\x01"  # Register  5037
    command_bms_temperature_2 = b"\x13\xB0\x00\x01"  # Register  5040
    command_current = b"\x13\xB2\x00\x01"  # Register  5042 (signed int)
    command_capacity = b"\x13\xB6\x00\x02"  # Registers 5046-5047 (long)
    command_soc = b"\x13\xB2\x00\x04"  # Registers 5042-5045 (amps, volts, soc as long)
    # Battery info
    command_manufacturer = b"\x14\x0C\x00\x08"  # Registers 5132-5139 (8 byte string)
    command_model = b"\x14\x02\x00\x08"  # Registers 5122-5129 (8 byte string)
    command_serial_number = b"\x13\xF6\x00\x08"  # Registers 5110-5117 (8 byte string)
    command_firmware_version = b"\x14\x0A\x00\x02"  # Registers 5130-5131 (2 byte string)
    # BMS warning and protection config

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

        # Set fet status once, because it is not available from the BMS
        self.charge_fet = True
        self.discharge_fet = True
        # self.balance_fet = True  # BMS does not have a balaner?

        return True

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure
        result = self.read_soc_data()
        result = result and self.read_cell_data()
        result = result and self.read_temperature_data()

        return result

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
            manufacturer, _, _ = unpack("16s", manufacturer)[0].decode("utf-8").partition("\0")
            self.hardware_version = f"{manufacturer} {model_num}"

        logger.info(self.hardware_version)

        if self.cell_count is None:
            cc = self.read_serial_data_renogy(self.command_cell_count)
            self.cell_count = struct.unpack(">H", cc)[0]

            for c in range(self.cell_count):
                self.cells.append(Cell(False))

        firmware = self.read_serial_data_renogy(self.command_firmware_version)
        firmware_major, firmware_minor = unpack_from("2s2s", firmware)
        firmware_major = firmware_major.decode("utf-8")
        firmware_minor = firmware_minor.decode("utf-8")
        self.version = float(f"{firmware_major}.{firmware_minor}")

        capacity = self.read_serial_data_renogy(self.command_capacity)
        self.capacity = unpack(">L", capacity)[0] / 1000.0

        try:
            serial_number = self.read_serial_data_renogy(self.command_serial_number)
            self.serial_number = unpack("16s", serial_number)[0].decode("utf-8")
        except Exception:
            logger.debug(f"serial number: {serial_number}")
            self.serial_number = None
            pass

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

    """
    # Did not found who changed this. "command_env_temperature_count" is missing
    def read_temperature_data(self):
        # Check to see how many Enviromental Temp Sensors this battery has, it may have none.
        num_env_temps = self.read_serial_data_renogy(self.command_env_temperature_count)
        logger.info("Number of Enviromental Sensors = %s", num_env_temps)

        if num_env_temps == 0:
            return False

        if num_env_temps == 1:
            temperature_1 = self.read_serial_data_renogy(self.command_env_temperature_1)

        if temperature_1 is False:
            return False
        else:
            self.temperature_1 = unpack(">H", temperature_1)[0] / 10
            logger.info("temperature_1 = %s °C", temperature_1)

        if num_env_temps == 2:
            temperature_2 = self.read_serial_data_renogy(self.command_env_temperature_2)

        if temperature_2 is False:
            return False
        else:
            self.temperature_2 = unpack(">H", temperature_2)[0] / 10
            logger.info("temperature_2 = %s °C", temperature_2)

        return True
    """

    def read_temperature_data(self):
        temperature_1 = self.read_serial_data_renogy(self.command_bms_temperature_1)
        temperature_2 = self.read_serial_data_renogy(self.command_bms_temperature_2)
        if temperature_1 is False:
            return False
        self.temperature_1 = unpack(">H", temperature_1)[0] / 10
        self.temperature_2 = unpack(">H", temperature_2)[0] / 10

        return True

    def read_bms_config(self):
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
        if data is False:
            return False

        start, flag, length = unpack_from("BBB", data)
        logger.info("Start: " + str(start))
        logger.info("Flag: " + str(flag))
        logger.info("Length: " + str(length))

        checksum = unpack_from(">H", data, length + 3)
        logger.info("Checksum: " + str(checksum[0]))

        if flag == 3:
            return data[3 : length + 3]
        else:
            logger.error(">>> ERROR: Incorrect Reply")
            return False
