# -*- coding: utf-8 -*-

# NOTES
# Please see "Add/Request a new BMS" https://mr-manuel.github.io/venus-os_dbus-serialbattery_docs/general/supported-bms#add-by-opening-a-pull-request
# in the documentation for a checklist what you have to do, when adding a new BMS

# avoid importing wildcards, remove unused imports
from bms.renogy import Renogy
from utils import (config, logger)
from bms.RenogyBLEBatteryClient import RenogyBLEBatteryClient
import sys
import bms.RenogyBLEUtils

class Renogy_Ble(Renogy):
    def __init__(self, port, baud, address):
        super(Renogy_Ble, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE
        # Exclude history values from calculation if they are provided from the BMS
        self.history.exclude_values_to_calculate = []

        # The RBT100LFP12SH-G1 uses 0xF7, another battery uses 0x30
        self.address = address


    BATTERYTYPE = "Renogy BLE"

    # the callback func when you receive data
    def on_data_received(client, data):
        filtered_data = RenogyBLEUtils.filter_fields(data, config["data"]["fields"])
        logger.info(f"{client.ble_manager.device.name} => {filtered_data}")
        
    # error callback
    def on_error(client, error):
        logger.error(f"on_error: {error}")

    def test_connection(self):
        """
        call a function that will connect to the battery, send a command and retrieve the result.
        The result or call should be unique to this BMS. Battery name or version, etc.
        Return True if success, False for failure
        """
        result = False
        try:
            battery = RenogyBLEBatteryClient(config, self.on_data_received, self.on_error)
            battery.start()

            # get settings to check if the data is valid and the connection is working
            result = self.get_settings()
            # get the rest of the data to be sure, that all data is valid and the correct battery type is recognized
            # only read next data if the first one was successful, this saves time when checking multiple battery types
            ##    result = result and self.read_gen_data()
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
