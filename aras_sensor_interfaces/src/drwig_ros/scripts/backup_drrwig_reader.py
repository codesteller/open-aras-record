#!/usr/bin/env python
"""
 # @ Author: Pallab Maji
 # @ Create Time: 2024-03-26 13:02:20
 # @ Modified time: 2024-03-26 13:05:07
 # @ Description: This is a ros node which reads the data comming in from DRWIG Radar and 
 #               publishes the data in the form of ROS1 Point Cloud messages.
 """

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

import sys
from dotted_dict import DottedDict
import time
import struct

import os
import sys
import serial
import numpy as np
import time
import json
import time 


class DrwigReaderBase:
    def __init__(self, args):

        if args.data_port is None:
            print("Error: Serial Data Port is Required")
            sys.exit(1)
        else:
            # If unix then serial port name is /dev/ttyACMx
            # If windows then serial port name is COMx
            if os.name == "posix":
                self.serial_port_name = "/dev/ttyACM" + str(args.data_port)
            elif os.name == "nt":
                self.serial_port_name = "COM" + str(args.data_port)
            else:
                print("Error: Unsupported OS")
                sys.exit(1)

        self.maxRange = args.max_range
        self.maxAzimuth = args.max_azimuth
        self.maxElevation = args.max_elevation
        self.maxDopperVel = args.max_doppler_vel
        self.out_filename = args.out_filename

        self.debug = False
        self.debug_warning = False

        # Parameters
        self.serial_handler = None
        if self.out_filename is None:
            self.recordData = False
        else:
            self.recordData = True
            self.openFile()

        self.sleep_time = 0.01  # 10ms read frequency

    def openSerialPort(self):
        try:
            self.serial_handler = serial.Serial(
                self.serial_port_name, 926100, timeout=0.1
            )
            print("Serial Port Opened Successfully")
        except Exception as e:
            print("Error: ", e)
            sys.exit(1)

    def closeSerialPort(self):
        self.serial_handler.close()
        print("Serial Port Closed Successfully")

    def readSerialData(self):
        if self.serial_handler is None:
            print("Serial Port is not Opened")
            sys.exit(1)
        try:
            while True:
                data = self.serial_handler.readline()
                print(data)
        except KeyboardInterrupt:
            self.closeSerialPort()

    def openFile(self):
        check_dir = os.path.dirname(self.out_filename)
        if not os.path.exists(check_dir):
            os.makedirs(check_dir)
        try:
            self.file_handler = open(self.out_filename, "w")
            print("File Opened Successfully")
            self.file_handler.write("[")
        except Exception as e:
            print("Error: ", e)
            sys.exit(1)

    def closeFile(self):
        if self.file_handler is None:
            print("File is not Opened")
            sys.exit(1)
        self.file_handler.write("]")
        self.file_handler.close()
        print("File Closed Successfully")

    def writeToFile(self, data):
        if self.file_handler is None:
            print("File is not Opened")
            sys.exit(1)
        # Convert all NDArray to List
        dumpdata = dict()
        for key in data:
            if isinstance(data[key], np.ndarray):
                dumpdata[key] = data[key].tolist()
            else:
                dumpdata[key] = data[key]

        json.dump(dumpdata, self.file_handler, indent=4)
        self.file_handler.write(",\n")
        # print(data)

    def parseTLVDataStructure(self, data):
        """
        This function needs to be overridden by base class to accomodate
        various Types of Radar Data Formats and Devices
        """
        pass


# Class definition for AWR1843 Radar Data Reader with DRWIG TLV Format
class Drwig1843Reader(DrwigReaderBase):
    def __init__(self, args):
        super().__init__(args)

        # Drwig Predefined Parameters from Drwig Firmware
        self.platformType = int("a1843", 16)  # AWR1843 Radar Platform Type
        self.MMWDEMO_UART_MSG_DETECTED_POINTS = 1
        self.MMWDEMO_UART_MSG_CLUSTERS = 2
        self.MMWDEMO_UART_MSG_TRACKED_OBJ = 3
        self.MMWDEMO_UART_MSG_PARKING_ASSIST = 4
        self.MMWDEMO_UART_MSG_STATS = 6

        self.max_objects = 200
        self.max_clusters = 24
        self.max_tracks = 24

        self.bytesize_obj_structure = 10
        self.bytesize_cluster_structure = 8
        self.bytesize_track_structure = 12
        self.bytesize_stats_structure = 16

        self.bytevec_log = np.zeros((0, 1), dtype=np.float32)
        self.readUartFcnCntr = 0
        self.ELEV_VIEW = 3
        self.EXIT_KEY_PRESSED = 0
        self.BYTES_AVAILABLE_FLAG = 0
        self.BYTES_AVAILABLE_FCN_CNT = 32 * 8
        self.BYTE_VEC_ACC_MAX_SIZE = 2**16
        self.bytevecAcc = np.zeros((self.BYTE_VEC_ACC_MAX_SIZE), dtype=np.uint8)
        self.bytevecAccLen = 0

        self.statsInfo = {
            "interFrameProcessingTime": 0,
            "transmitOutputTime": 0,
            "interFrameProcessingMargin": 0,
            "interChirpProcessingMargin": 0,
            "interFrameCPULoad": 0,
            "activeFrameCPULoad": 0,
        }
        self.activeFrameCPULoad = np.zeros((100, 1), dtype=np.float32)
        self.interFrameCPULoad = np.zeros((100, 1), dtype=np.float32)
        self.guiCPULoad = np.zeros((100, 1), dtype=np.float32)
        self.view_range = 0
        self.guiProcTime = 0

        self.displayUpdateCntr = 0
        self.timeout_ctr = 0
        self.bytevec_cp_max_len = 2**15
        self.bytevec_cp = np.zeros((self.bytevec_cp_max_len), dtype=np.uint8)
        self.bytevec_cp_len = 0
        self.packetNumberPrev = 0
        self.loggingEnable = 0
        self.fidLog = 0
        self.use_perspective_projection = 0
        self.prev_use_perspective_projection = self.use_perspective_projection

        self.magicNotOkCntr = 0

        # Every packet of data coming from the AWR1843 has a barker code as defined in the firmware
        # Defaut barker code is [2 1 4 3 6 5 8 7]
        # TODO: Change the barker code to Dwrig Specific barker code
        # barker_code = char([2 1 4 3 6 5 8 7]);
        self.barker_code = [2, 1, 4, 3, 6, 5, 8, 7]

        self.sleep_time = 0.001

        self.file_handler = None

    def parseTLVDataStructureThreading(self, delay, run_event):

        self.openSerialPort()
        while run_event.is_set():
            # ------------------------------------------
            # Read UART data and Load it to the buffer
            # ------------------------------------------
            _ret = self.readUartData()
            if _ret is None:
                if self.debug_warning:
                    print("[WARNING]: No Data Available")
                continue

            # -------------------------------------------------------------------------
            # Check if the bytes are available, then append new bytes to the bytevec_cp
            # -------------------------------------------------------------------------
            if self.BYTES_AVAILABLE_FLAG == 1:
                self.BYTES_AVAILABLE_FLAG = 0
                if self.bytevec_cp_len + self.bytevecAccLen < self.bytevec_cp_max_len:
                    self.bytevec_cp[
                        self.bytevec_cp_len : self.bytevec_cp_len + self.bytevecAccLen
                    ] = self.bytevecAcc[0 : self.bytevecAccLen]
                    self.bytevec_cp_len = self.bytevec_cp_len + self.bytevecAccLen
                    self.bytevecAccLen = 0
                else:
                    print(
                        "[Error]: Buffer Overflow: bytevec_cp_max_len exceeded. {}, {}".format(
                            self.bytevec_cp_len, self.bytevecAccLen
                        )
                    )
                    self.bytevecAccLen = 0
                    self.bytevec_cp_len = 0

            # -------------------------------------------------------------------
            # Try Catch Block to Parse the TLV Data from Buffer
            # -------------------------------------------------------------------

            try:
                # TODO : Check this part for correctness in byte vs char
                bytevecStr = self.bytevec_cp[0 : self.bytevec_cp_len].tostring()
                magicOk = 0
                # if the bytevecStr is atleast as large as the header, check if it contains the header.
                if len(bytevecStr) > 72:
                    # Check is the start of the header is present in the bytevecStr
                    # if so then set startIdx to the index of the start of the header
                    startIdx = bytevecStr.find(b"\x02\x01\x04\x03\x06\x05\x08\x07")
                else:
                    startIdx = -1

                if startIdx != -1:
                    if startIdx >= 0:
                        # print("startIdx: ", startIdx)
                        magicOk = 1
                        countOk = 0
                        self.bytevecAccLen = self.bytevec_cp_len - startIdx
                        self.bytevec_cp[0 : self.bytevecAccLen] = self.bytevec_cp[
                            startIdx : self.bytevec_cp_len
                        ]
                        self.bytevec_cp_len = self.bytevecAccLen

                    else:
                        print("[Error] startIdx < 0")
                        magicOk = 0
                        countOk = 0
                        self.bytevec_cp_len = 0
                        self.bytevec_cp = np.zeros(
                            self.bytevec_cp_max_len, dtype="uint8"
                        )
                else:
                    print("[Error] Header not found")
                    magicOk = 0

                if magicOk == 1:
                    # Start timer for parsing
                    tStart = time.time()
                    # Parse the TLV Data
                    # parsed_data = self.parseTLVData(bytevecStr)
                    parsed_data = self.parseTLVData()
                    # Stop timer for parsing
                    tStop = time.time()
                    # Calculate the time taken for parsing
                    tParse = tStop - tStart
                    # Print the time taken for parsing
                    if self.debug_warning:
                        print("[INFO] Time Taken for Parsing: ", tParse)

                    if not parsed_data:
                        if self.debug_warning:
                            print("[WARNING]: More than 2 subframes detected")
                        continue

                if ord("q") == 0xFF & 0xFF:
                    self.closeSerialPort()
                    self.closeFile()
                    self.EXIT_KEY_PRESSED = 1
                    print("Exiting the Radar Data Parsing")

            except Exception as e:
                # print("[Error] ", e)
                return

            time.sleep(delay)
        return

    def parseTLVDataStructure(self):
        """
        This function needs to be overridden by base class to accomodate
        various Types of Radar Data Formats and Devices
        """
        # Run a loop to parse the TLV Data until esc key is pressed to exit

        try:
            self.openSerialPort()
            while self.EXIT_KEY_PRESSED == 0:
                self.readRadarDataMRR()

        except KeyboardInterrupt as e:
            self.closeSerialPort()
            self.closeFile()
            self.EXIT_KEY_PRESSED = 1
            print("Exiting the Radar Data Parsing")

        return

    def readRadarDataMRR(self):
        """
        This function is used to read the Radar Data from MRR Radar
        """
        # ------------------------------------------
        # Read UART data and Load it to the buffer
        # ------------------------------------------
        _ret = self.readUartData()
        parsed_data = None
        if _ret is None:
            if self.debug_warning:
                print("[WARNING]: No Data Available")
            return

        # -------------------------------------------------------------------------
        # Check if the bytes are available, then append new bytes to the bytevec_cp
        # -------------------------------------------------------------------------
        if self.BYTES_AVAILABLE_FLAG == 1:
            self.BYTES_AVAILABLE_FLAG = 0
            if self.bytevec_cp_len + self.bytevecAccLen < self.bytevec_cp_max_len:
                self.bytevec_cp[
                    self.bytevec_cp_len : self.bytevec_cp_len + self.bytevecAccLen
                ] = self.bytevecAcc[0 : self.bytevecAccLen]
                self.bytevec_cp_len = self.bytevec_cp_len + self.bytevecAccLen
                self.bytevecAccLen = 0
            else:
                print(
                    "[Error]: Buffer Overflow: bytevec_cp_max_len exceeded. {}, {}".format(
                        self.bytevec_cp_len, self.bytevecAccLen
                    )
                )
                self.bytevecAccLen = 0
                self.bytevec_cp_len = 0

        # -------------------------------------------------------------------
        # Try Catch Block to Parse the TLV Data from Buffer
        # -------------------------------------------------------------------

        try:
            # TODO : Check this part for correctness in byte vs char
            bytevecStr = self.bytevec_cp[0 : self.bytevec_cp_len].tostring()
            magicOk = 0
            # if the bytevecStr is atleast as large as the header, check if it contains the header.
            if len(bytevecStr) > 72:
                # Check is the start of the header is present in the bytevecStr
                # if so then set startIdx to the index of the start of the header
                startIdx = bytevecStr.find(b"\x02\x01\x04\x03\x06\x05\x08\x07")
                # print("Header Found at: ", startIdx)
            else:
                startIdx = -1

            if startIdx != -1:
                if startIdx >= 0:
                    # print("startIdx: ", startIdx)
                    magicOk = 1
                    countOk = 0
                    self.bytevecAccLen = self.bytevec_cp_len - startIdx
                    self.bytevec_cp[0 : self.bytevecAccLen] = self.bytevec_cp[
                        startIdx : self.bytevec_cp_len
                    ]
                    self.bytevec_cp_len = self.bytevecAccLen

                else:
                    print("Error: startIdx < 0")
                    magicOk = 0
                    countOk = 0
                    self.bytevec_cp_len = 0
                    self.bytevec_cp = np.zeros(self.bytevec_cp_max_len, dtype="uint8")
            else:
                print("Error: Header not found")
                magicOk = 0

            if magicOk == 1:
                # Start timer for parsing
                tStart = time.time()
                # Parse the TLV Data
                # parsed_data = self.parseTLVData(bytevecStr)
                parsed_data = self.parseTLVData()
                # Stop timer for parsing
                tStop = time.time()
                # Calculate the time taken for parsing
                tParse = tStop - tStart
                # Print the time taken for parsing
                if self.debug:
                    print("[INFO] Time Taken for Parsing: ", tParse)

                if not parsed_data:
                    if self.debug_warning:
                        print("[WARNING]: More than 2 subframes detected")
                    return

            if ord("q") == 0xFF & 0xFF:
                self.closeSerialPort()
                self.closeFile()
                self.EXIT_KEY_PRESSED = 1
                print("Exiting the Radar Data Parsing")

        except Exception as e:
            print("[Error]: ", e)

        return parsed_data

    # Override Open Serial Port function to add additional functionality
    def openSerialPort(self, baudrate=926100, timeout=10):
        """
        Documentation suggests following parameters for serial port:
        Baud Rate: 926100
        Input Buffer Size: 2^16
        Timeout: 10
        ErrorFunction: To be defined as a callback function
        BytesAvailableFcnMode: 'byte'
        BytesAvailableFcnCount: 2^16 + 1
        BytesAvailableFcn: To be defined as a callback function

        However, pyserial does not support all of these parameters. Hence we will use
        Baud Rate: 926100
        Timeout: 10 sec

        """
        try:
            self.serial_handler = serial.Serial(
                self.serial_port_name, baudrate, timeout=timeout
            )
            print("[INFO] Serial Port Opened Successfully")
        except Exception as e:
            print("[Error] Could not open Serial Port {}".format(self.serial_port_name))
            print("[Error]: ", e)
            sys.exit(1)

        return

    # Read Uart Data and Load to Buffer
    def readUartData(self):
        bytestoread = self.serial_handler.inWaiting()
        if bytestoread > 0:
            ser_data = self.serial_handler.read(bytestoread)
            bytecount = len(ser_data)

            # Precautionary Data check to avoid buffer overflow
            if self.bytevecAccLen + bytecount < self.BYTE_VEC_ACC_MAX_SIZE * 3 / 4:
                self.bytevecAcc[self.bytevecAccLen : self.bytevecAccLen + bytecount] = (
                    np.frombuffer(ser_data, dtype=np.uint8)
                )
                self.bytevecAccLen = self.bytevecAccLen + bytecount
            # if self.bytevec_cp_len + bytecount < self.bytevec_cp_max_len:
            #     self.bytevec_cp[
            #         self.bytevec_cp_len : self.bytevec_cp_len + bytecount
            #     ] = np.frombuffer(ser_data, dtype=np.uint8)
            #     self.bytevec_cp_len = self.bytevec_cp_len + bytecount
            else:
                print("[Error] Buffer Overflow")
                self.bytevecAccLen = 0
                self.bytevecAcc = np.zeros(
                    (self.BYTE_VEC_ACC_MAX_SIZE, 1), dtype=np.uint8
                )
                # self.bytevec_cp_len = 0
                # self.bytevec_cp = np.zeros(
                #     (self.bytevec_cp_max_len), dtype=np.uint8
                # )

            self.readUartFcnCntr = self.readUartFcnCntr + 1
            self.BYTES_AVAILABLE_FLAG = 1
            return True
        else:
            self.BYTES_AVAILABLE_FLAG = 0
            return None

    # Parse TLV Data from AWR1843 Radar
    def parseTLVData(self):
        """
        This function is used to parse the TLV Data from AWR1843 Radar
        """

        parsedData = dict()

        byteVecIndex = 0
        # Forward index by 8 byte due to barker code
        byteVecIndex += 8

        # Parse the TLV Header
        bytevecStr = self.bytevec_cp[0 : self.bytevec_cp_len]
        header, byteVecIndex = self.parseTLVHeader(bytevecStr, byteVecIndex)
        subFrameNumber = header["subFrameNumber"] + 1
        if subFrameNumber > 2:
            return parsedData

        parsedData["header"] = header

        # Parse the TLV Data based on the TLV Type
        detectedObjects = dict()
        clusterObjects = dict()
        trackedObjects = dict()

        detectedObjects["num_objects"] = 0
        clusterObjects["num_objects"] = 0
        trackedObjects["num_objects"] = 0

        for tlvIdx in range(header["numTLVs"]):
            tlvType = int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32"
                )
            )
            byteVecIndex += 4
            tlvLength = int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32"
                )
            )
            byteVecIndex += 4
            if tlvType == self.MMWDEMO_UART_MSG_DETECTED_POINTS:
                # Parse the Detected Objects
                detectedObjects, byteVecIndex = self.parseDetectedObjects(
                    bytevecStr, byteVecIndex, tlvLength, detectedObjects
                )
            elif tlvType == self.MMWDEMO_UART_MSG_CLUSTERS:
                # Parse the Cluster Objects
                clusterObjects, byteVecIndex = self.parseClusterObjects(
                    bytevecStr, byteVecIndex, tlvLength, clusterObjects
                )
            elif tlvType == self.MMWDEMO_UART_MSG_TRACKED_OBJ:
                # Parse the Tracked Objects
                trackedObjects, byteVecIndex = self.parseTrackedObjects(
                    bytevecStr, byteVecIndex, tlvLength, trackedObjects
                )
            else:
                # Skip the TLV Data
                byteVecIndex += tlvLength

        parsedData["detected_objects"] = detectedObjects
        parsedData["cluster_objects"] = clusterObjects
        parsedData["tracked_objects"] = trackedObjects

        if self.debug:
            if detectedObjects["num_objects"] > 0:
                print("*********************************************")

        return parsedData

    def parseDetectedObjects(
        self, bytevecStr, byteVecIndex, tlvLength, detectedObjects
    ):
        """
        This function is used to parse the Detected Objects from AWR1843 Radar
        """
        # Initialize the detected objects dictionary
        byteVecStrLength = len(bytevecStr)
        if byteVecStrLength < byteVecIndex + tlvLength:
            return detectedObjects, byteVecIndex

        if tlvLength > 0:
            """
            The byte vector structure is int16 i.e. 2 bytes
            """
            # Get utc timestamp and convert to string
            detectedObjects["timestamp"] = time.time()
            detectedObjects["num_objects"] = int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 2], dtype="int16"
                )
            )
            byteVecIndex += 2

            xyzqFormat = 2 ** int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 2], dtype="int16"
                )
            )
            byteVecIndex += 2

            inv_xyzqFormat = 1.0 / xyzqFormat

            if (
                byteVecStrLength
                < byteVecIndex
                + self.bytesize_obj_structure * detectedObjects["num_objects"]
            ):
                detectedObjects["num_objects"] = 0
                byteVecIndex = byteVecStrLength
                return detectedObjects, byteVecIndex

            raw_bytes = bytevecStr[
                byteVecIndex : byteVecIndex
                + detectedObjects["num_objects"] * self.bytesize_obj_structure
            ]
            byteVecIndex += detectedObjects["num_objects"] * self.bytesize_obj_structure

            # Reshape the raw bytes to 2D array
            raw_bytes = np.reshape(
                raw_bytes, (detectedObjects["num_objects"], self.bytesize_obj_structure)
            )

            # ------------------------------------------------------------------------------------------
            #       Parse the Detected Objects
            # ------------------------------------------------------------------------------------------
            detectedObjects["dopplerVelocity"] = raw_bytes[:, 0] + raw_bytes[:, 1] * 256
            detectedObjects["peakValue"] = raw_bytes[:, 2] + raw_bytes[:, 3] * 256

            detectedObjects["x"] = np.array(
                raw_bytes[:, 4] + raw_bytes[:, 5] * 256, dtype="int16"
            )
            detectedObjects["y"] = np.array(
                raw_bytes[:, 6] + raw_bytes[:, 7] * 256, dtype="int16"
            )
            detectedObjects["z"] = np.array(
                raw_bytes[:, 8] + raw_bytes[:, 9] * 256, dtype="int16"
            )

            detectedObjects["x"][detectedObjects["x"] > 32767] = (
                detectedObjects["x"][detectedObjects["x"] > 32767] - 65536
            )
            detectedObjects["y"][detectedObjects["y"] > 32767] = (
                detectedObjects["y"][detectedObjects["y"] > 32767] - 65536
            )
            detectedObjects["z"][detectedObjects["z"] > 32767] = (
                detectedObjects["z"][detectedObjects["z"] > 32767] - 65536
            )
            detectedObjects["dopplerVelocity"][
                detectedObjects["dopplerVelocity"] > 32767
            ] = (
                detectedObjects["dopplerVelocity"][
                    detectedObjects["dopplerVelocity"] > 32767
                ]
                - 65536
            )

            detectedObjects["x"] = detectedObjects["x"] * inv_xyzqFormat
            detectedObjects["y"] = detectedObjects["y"] * inv_xyzqFormat
            detectedObjects["z"] = -detectedObjects["z"] * inv_xyzqFormat

            detectedObjects["range"] = np.sqrt(
                detectedObjects["x"] ** 2
                + detectedObjects["y"] ** 2
                + detectedObjects["z"] ** 2
            )

            detectedObjects["azimuth"] = np.arctan2(
                detectedObjects["y"], detectedObjects["x"]
            )
            detectedObjects["elevation"] = np.arcsin(
                detectedObjects["z"] / detectedObjects["range"]
            )

            if self.debug:
                print(
                    "Total Detected Objects [{}]: ".format(
                        detectedObjects["num_objects"]
                    )
                )
                for i in range(detectedObjects["num_objects"]):
                    print(
                        "Detected Object {}: Range: {}, Azimuth: {}, Elevation: {}, Doppler Velocity: {}, Peak Value: {}".format(
                            i,
                            detectedObjects["y"][i],
                            detectedObjects["x"][i],
                            detectedObjects["z"][i],
                            detectedObjects["dopplerVelocity"][i],
                            detectedObjects["peakValue"][i],
                        )
                    )

            if self.recordData:
                self.writeToFile(detectedObjects)

        return detectedObjects, byteVecIndex

    def parseClusterObjects(self, bytevecStr, byteVecIndex, tlvLength, clusterObjects):
        """
        This function is used to parse the Cluster Objects from AWR1843 Radar
        """
        # Initialize the cluster objects dictionary
        byteVecStrLength = len(bytevecStr)
        if byteVecStrLength < byteVecIndex + tlvLength:
            return clusterObjects, byteVecIndex

        if tlvLength > 0:
            """
            The byte vector structure is int16 i.e. 2 bytes
            """

            clusterObjects["num_objects"] = int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 2], dtype="int16"
                )
            )
            byteVecIndex += 2

            xyzqFormat = 2 ** int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 2], dtype="int16"
                )
            )
            byteVecIndex += 2

            inv_xyzqFormat = 1.0 / xyzqFormat

            if (
                byteVecStrLength
                < byteVecIndex
                + self.bytesize_cluster_structure * clusterObjects["num_objects"]
            ):
                clusterObjects["num_objects"] = 0
                byteVecIndex = byteVecStrLength
                return clusterObjects, byteVecIndex

            raw_bytes = bytevecStr[
                byteVecIndex : byteVecIndex
                + clusterObjects["num_objects"] * self.bytesize_cluster_structure
            ]
            byteVecIndex += (
                clusterObjects["num_objects"] * self.bytesize_cluster_structure
            )

            # Reshape the raw bytes to 2D array
            raw_bytes = np.reshape(
                raw_bytes,
                (clusterObjects["num_objects"], self.bytesize_cluster_structure),
            )

            # ------------------------------------------------------------------------------------------
            #  Parse the Cluster Objects X & Y raw to find the custer center and then calculate the area
            #    The byte vector structure is int16 i.e. 2 bytes
            #       - x: 2 bytes
            #       - y: 2 bytes
            #       - clusterwidth: 2 bytes
            #       - clusterlength: 2 bytes
            #       - No Z, Doppler Velocity, Peak Value in Cluster Object
            # ------------------------------------------------------------------------------------------
            x_raw = np.array(raw_bytes[:, 0] + raw_bytes[:, 1] * 256, dtype="int16")
            y_raw = np.array(raw_bytes[:, 2] + raw_bytes[:, 3] * 256, dtype="int16")
            clusterWidth = np.array(
                raw_bytes[:, 4] + raw_bytes[:, 5] * 256, dtype="int16"
            )
            clusterLength = np.array(
                raw_bytes[:, 6] + raw_bytes[:, 7] * 256, dtype="int16"
            )

            x_raw[x_raw > 32767] = x_raw[x_raw > 32767] - 65536
            y_raw[y_raw > 32767] = y_raw[y_raw > 32767] - 65536

            x_center = x_raw * inv_xyzqFormat
            y_center = y_raw * inv_xyzqFormat
            clusterWidth = clusterWidth * inv_xyzqFormat
            clusterLength = clusterLength * inv_xyzqFormat

            cluster_area = clusterWidth * clusterLength * 4

            # Filter clusterWidth based on cluster area & X-center
            x_center[cluster_area > 20] = 99999

            x_position = x_center + clusterWidth * [-1, 1, 1, -1, -1, 99999]
            y_position = y_center + clusterLength * [-1, -1, 1, 1, -1, 99999]

            clusterObjects["x_position"] = x_position
            clusterObjects["y_position"] = y_position
            clusterObjects["x_center"] = x_center
            clusterObjects["y_center"] = y_center
            clusterObjects["area"] = cluster_area

            if self.debug:
                print(
                    "Total Cluster Objects [{}]: ".format(clusterObjects["num_objects"])
                )
                for i in range(clusterObjects["num_objects"]):
                    print(
                        "Cluster Object {}: X: {}, Y: {}, Area: {}".format(
                            i,
                            clusterObjects["x_position"][i],
                            clusterObjects["y_position"][i],
                            clusterObjects["area"][i],
                        )
                    )

            return clusterObjects, byteVecIndex

    def parseTrackedObjects(self, bytevecStr, byteVecIndex, tlvLength, trackedObjects):
        """
        This function is used to parse the Tracked Objects from AWR1843 Radar
        """
        # Initialize the tracked objects dictionary
        byteVecStrLength = len(bytevecStr)
        if byteVecStrLength < byteVecIndex + tlvLength:
            return trackedObjects, byteVecIndex

        if tlvLength > 0:
            """
            The byte vector structure is int16 i.e. 2 bytes
            """

            trackedObjects["num_objects"] = int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 2], dtype="int16"
                )
            )
            byteVecIndex += 2

            xyzqFormat = 2 ** int(
                np.frombuffer(
                    bytevecStr[byteVecIndex : byteVecIndex + 2], dtype="int16"
                )
            )
            byteVecIndex += 2

            inv_xyzqFormat = 1.0 / xyzqFormat

            if (
                byteVecStrLength
                < byteVecIndex
                + self.bytesize_track_structure * trackedObjects["num_objects"]
            ):
                trackedObjects["num_objects"] = 0
                byteVecIndex = byteVecStrLength
                return trackedObjects, byteVecIndex

            raw_bytes = bytevecStr[
                byteVecIndex : byteVecIndex
                + trackedObjects["num_objects"] * self.bytesize_track_structure
            ]
            byteVecIndex += (
                trackedObjects["num_objects"] * self.bytesize_track_structure
            )

            # Reshape the raw bytes to 2D array
            raw_bytes = np.reshape(
                raw_bytes,
                (trackedObjects["num_objects"], self.bytesize_track_structure),
            )

            # ------------------------------------------------------------------------------------------
            #  Parse the Tracked Objects X & Y raw to find the custer center and then calculate the area
            #  , dopper velocity, range, azimuth, elevation
            #    The byte vector structure is int16 i.e. 2 bytes
            #       - x: 2 bytes
            #       - y: 2 bytes
            #       - vx: 2 bytes
            #       - vy: 2 bytes
            #       - objWidth: 2 bytes
            #       - objLength: 2 bytes
            # ------------------------------------------------------------------------------------------

            x_raw = np.array(raw_bytes[:, 0] + raw_bytes[:, 1] * 256, dtype="int16")
            y_raw = np.array(raw_bytes[:, 2] + raw_bytes[:, 3] * 256, dtype="int16")

            vx_raw = np.array(raw_bytes[:, 4] + raw_bytes[:, 5] * 256, dtype="int16")
            vy_raw = np.array(raw_bytes[:, 6] + raw_bytes[:, 7] * 256, dtype="int16")

            objWidth = np.array(raw_bytes[:, 8] + raw_bytes[:, 9] * 256, dtype="int16")
            objLength = np.array(
                raw_bytes[:, 10] + raw_bytes[:, 11] * 256, dtype="int16"
            )

            x_raw[x_raw > 32767] = x_raw[x_raw > 32767] - 65536
            y_raw[y_raw > 32767] = y_raw[y_raw > 32767] - 65536
            vx_raw[vx_raw > 32767] = vx_raw[vx_raw > 32767] - 65536
            vy_raw[vy_raw > 32767] = vy_raw[vy_raw > 32767] - 65536

            x_pos = x_raw * inv_xyzqFormat
            y_pos = y_raw * inv_xyzqFormat
            vx = vx_raw * inv_xyzqFormat
            vy = vy_raw * inv_xyzqFormat
            objWidth = objWidth * inv_xyzqFormat
            objLength = objLength * inv_xyzqFormat

            x_center = np.array(
                x_raw.reshape(-1, 1)
                + objWidth.reshape(-1, 1) * [-1, 1, 1, -1, -1, 99999]
            ).reshape(-1)
            y_center = np.array(
                y_raw.reshape(-1, 1)
                + objLength.reshape(-1, 1) * [-1, -1, 1, 1, -1, 99999]
            ).reshape(-1)

            range_ = np.sqrt(x_pos**2 + y_pos**2)
            azimuth = np.arctan2(y_pos, x_pos)
            elevation = 0
            dopplerVelocity = ((x_pos * vx) + (y_pos * vy)) / range_

            trackedObjects["x_position"] = x_pos
            trackedObjects["y_position"] = y_pos
            trackedObjects["x_center"] = x_center
            trackedObjects["y_center"] = y_center
            trackedObjects["range"] = range_
            trackedObjects["azimuth"] = azimuth
            trackedObjects["elevation"] = elevation
            trackedObjects["dopplerVelocity"] = dopplerVelocity

            if self.debug:
                print(
                    "Total Tracked Objects [{}]: ".format(trackedObjects["num_objects"])
                )
                for i in range(trackedObjects["num_objects"]):
                    print(
                        "Tracked Object {}: X: {}, Y: {}, Range: {}, Azimuth: {}, Elevation: {}, Doppler Velocity: {}".format(
                            i,
                            trackedObjects["x_position"][i],
                            trackedObjects["y_position"][i],
                            trackedObjects["range"][i],
                            trackedObjects["azimuth"][i],
                            trackedObjects["elevation"][i],
                            trackedObjects["dopplerVelocity"][i],
                        )
                    )

            return trackedObjects, byteVecIndex

    def parseTLVHeader(self, bytevecStr, byteVecIndex):
        """
        This function is used to parse the TLV Header Data & start pointer of the data from AWR1843 Radar
        """
        # Initialize the header dictionary
        # multiply by 256^0, 256^1, 256^2, 256^3 and add resulting in unit32
        word = [1, 256, 65536, 16777216]
        header = dict()
        header["version"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["totalPacketLen"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["platform"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["frameNumber"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["timeCpuCycles"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["numDetectedObj"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["numTLVs"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        header["subFrameNumber"] = int(
            np.frombuffer(bytevecStr[byteVecIndex : byteVecIndex + 4], dtype="uint32")
        )
        byteVecIndex = byteVecIndex + 4
        return header, byteVecIndex

    def __del__(self):
        if self.serial_handler is not None:
            self.closeSerialPort()
        if self.recordData:
            self.closeFile()
        print("Drwig1843Reader Object Destroyed")


class DrwigReader:
    def __init__(self):
        rospy.init_node("drwig_reader", anonymous=True)

        # Create publisher for detection data, cluster data and tracker data as PointCloud2 messages
        self.publisher_detection = rospy.Publisher(
            "/drwig_data/detection", PointCloud2, queue_size=10
        )
        self.publisher_cluster = rospy.Publisher(
            "/drwig_data/cluster", PointCloud2, queue_size=10
        )
        self.publisher_tracker = rospy.Publisher(
            "/drwig_data/tracker", PointCloud2, queue_size=10
        )

        # Create Timer
        self.rate = rospy.Rate(30)  # 10hz
        self.frame_idx = 0

        # Create DRWIG Radar object
        args = DottedDict()
        args["data_port"] = 2
        args["max_range"] = 100
        args["max_azimuth"] = 80
        args["max_elevation"] = 50
        args["max_doppler_vel"] = 20
        args["out_filename"] = None
        rospy.loginfo("Initializing DRWIG Radar at port /dev/ttyACM%s" % args.data_port)

        self.dw = Drwig1843Reader(args)
        self.dw.openSerialPort()

    def run(self):

        # Setup detection Messages
        det_msg = PointCloud2()
        det_msg.header.frame_id = "drwig"
        det_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="velocity", offset=16, datatype=PointField.FLOAT32, count=1
            ),
        ]

        det_msg.height = 1
        det_msg.is_bigendian = False
        det_msg.is_dense = True
        det_msg.point_step = len(det_msg.fields) * 4

        points_dtype_det = np.dtype(
            [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("intensity", np.float32),
                ("velocity", np.float32),
            ]
        )

        # Setup cluster Messages
        cluster_msg = PointCloud2()
        cluster_msg.header.frame_id = "drwig"
        cluster_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="area", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cluster_msg.height = 1
        cluster_msg.is_bigendian = False
        cluster_msg.is_dense = True
        cluster_msg.point_step = len(cluster_msg.fields) * 4

        points_dtype_cluster = np.dtype(
            [("x", np.float32), ("y", np.float32), ("area", np.float32)]
        )

        # Setup tracker Messages
        tracker_msg = PointCloud2()
        tracker_msg.header.frame_id = "drwig"
        tracker_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="velocity", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        tracker_msg.height = 1
        tracker_msg.is_bigendian = False
        tracker_msg.is_dense = True
        tracker_msg.point_step = len(tracker_msg.fields) * 4

        points_dtype_tracker = np.dtype(
            [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("velocity", np.float32),
            ]
        )

        # --------------------------------------------------------------------------

        # Run the loop to read the data from the radar
        while True:
            if rospy.is_shutdown():
                break
            # Read the data from the radar
            detectedObjects_prev = None
            parsed_data = self.dw.readRadarDataMRR()
            if parsed_data is None:
                rospy.logerr("No data received from the radar")
                time.sleep(0.1)
                continue

            time_tic = time.time()

            # --------------------------------------------------------------------------
            #               Detection Data
            # --------------------------------------------------------------------------
            detectedObjects_new = parsed_data["detected_objects"]

            if detectedObjects_new["num_objects"] < 1:
                if self.dw.debug:
                    rospy.loginfo("No objects detected")

                if detectedObjects_prev is not None:
                    detectedObjects = detectedObjects_prev
                else:
                    continue
            else:
                detectedObjects = detectedObjects_new

            # print the detected objects
            if self.dw.debug:
                rospy.loginfo("Detected %d objects" % detectedObjects["num_objects"])
                for i in range(detectedObjects["num_objects"]):
                    rospy.loginfo(
                        "Object %d: Range: %f, Azimuth: %f, Doppler: %f"
                        % (
                            i,
                            detectedObjects["range"][i],
                            detectedObjects["azimuth"][i],
                            detectedObjects["doppler"][i],
                        )
                    )
                rospy.loginfo("-----------------------------------")

            ## Package Deetction data
            det_msg.header.stamp = rospy.Time.now()
            num_points = detectedObjects["num_objects"]
            points = np.zeros((num_points, len(det_msg.fields)), dtype=np.float32)
            # Fill points with detected objects x,y,z coordinates
            for i in range(num_points):
                points[i, 0] = detectedObjects["x"][i]
                points[i, 1] = detectedObjects["y"][i]
                points[i, 2] = detectedObjects["z"][i]
                points[i, 3] = detectedObjects["peakValue"][i]
                points[i, 4] = detectedObjects["dopplerVelocity"][i]

            # Pack point cloud data into binary format
            points_array_det = np.empty(len(points), dtype=points_dtype_det)
            points_array_det["x"] = points[:, 0]
            points_array_det["y"] = points[:, 1]
            points_array_det["z"] = points[:, 2]
            points_array_det["intensity"] = points[:, 3]
            points_array_det["velocity"] = points[:, 4]

            points_list = points_array_det.tolist()

            # Pack the data into PointCloud2 message
            det_msg.width = len(points_list)
            det_msg.row_step = det_msg.point_step * len(points_list)
            det_msg.data = struct.pack(
                "<%df" % (len(points_list) * len(det_msg.fields)), *points.ravel()
            )

            # Store previous detected objects
            detectedObjects_prev = detectedObjects_new

            # --------------------------------------------------------------------------
            #               Cluster Data
            # --------------------------------------------------------------------------
            _flag = 1
            clusters = parsed_data["cluster_objects"]
            if clusters["num_objects"] < 1:
                if self.dw.debug:
                    rospy.loginfo("[INFO] No clusters detected")
                    _flag = 0
            else:
                if self.dw.debug:
                    rospy.loginfo(
                        "[INFO] Detected %d clusters" % clusters["num_objects"]
                    )
                    # Print the detected objects X-Position, Y-Position, Area coordinates
                    rospy.loginfo("[INFO] Detected clusters: ")
                    for i in range(clusters["num_objects"]):
                        rospy.loginfo(
                            "Cluster %d: x=%f, y=%f, area=%f"
                            % (
                                i,
                                clusters["x_position"][i],
                                clusters["y_position"][i],
                                clusters["area"][i],
                            )
                        )
                    rospy.loginfo("[INFO] -----------------------------------")

            if _flag:
                ## Package the data
                cluster_msg.header.stamp = det_msg.header.stamp
                num_points = clusters["num_objects"]
                points = np.zeros(
                    (num_points, len(cluster_msg.fields)), dtype=np.float32
                )
                # Fill points with detected objects x,y,z coordinates
                for i in range(num_points):
                    points[i, 0] = clusters["x_position"][i]
                    points[i, 1] = clusters["y_position"][i]
                    points[i, 2] = clusters["area"][i]

                # Pack point cloud data into binary format
                points_array = np.empty(len(points), dtype=points_dtype_cluster)
                points_array["x"] = points[:, 0]
                points_array["y"] = points[:, 1]
                points_array["area"] = points[:, 2]

                points_list = points_array.tolist()

                # Pack the data into PointCloud2 message
                cluster_msg.width = len(points_list)
                cluster_msg.row_step = cluster_msg.point_step * len(points_list)
                cluster_msg.data = struct.pack(
                    "<%df" % (len(points_list) * len(cluster_msg.fields)),
                    *points.ravel()
                )

                _flag = 0

            # --------------------------------------------------------------------------
            #               Tracker Data
            # --------------------------------------------------------------------------
            _flag = 1
            trackedObjects = parsed_data["tracked_objects"]
            if trackedObjects["num_objects"] < 1:
                if self.dw.debug:
                    rospy.loginfo("[INFO] No tracked objects detected")
                    _flag = 0
            else:
                if self.dw.debug:
                    rospy.loginfo(
                        "[INFO] Detected %d tracked objects"
                        % trackedObjects["num_objects"]
                    )
                    # Print the detected objects X-Position, Y-Position, Area coordinates
                    rospy.loginfo("[INFO] Detected tracked objects: ")
                    for i in range(trackedObjects["num_objects"]):
                        rospy.loginfo(
                            "Tracked Object %d: x=%f, y=%f, range=%f, azimuth=%f, elevation=%f, doppler=%f"
                            % (
                                i,
                                trackedObjects["x_position"][i],
                                trackedObjects["y_position"][i],
                                trackedObjects["range"][i],
                                trackedObjects["azimuth"][i],
                                trackedObjects["elevation"][i],
                                trackedObjects["dopplerVelocity"][i],
                            )
                        )
                    rospy.loginfo("[INFO] -----------------------------------")

            if _flag:
                ## Package the data
                tracker_msg.header.stamp = det_msg.header.stamp
                num_points = trackedObjects["num_objects"]
                points = np.zeros(
                    (num_points, len(tracker_msg.fields)), dtype=np.float32
                )
                # Fill points with detected objects x,y,z coordinates
                for i in range(num_points):
                    points[i, 0] = trackedObjects["x_position"][i]
                    points[i, 1] = trackedObjects["y_position"][i]
                    points[i, 2] = trackedObjects["range"][i]
                    points[i, 3] = trackedObjects["dopplerVelocity"][i]

                # Pack point cloud data into binary format
                points_array = np.empty(len(points), dtype=points_dtype_tracker)
                points_array["x"] = points[:, 0]
                points_array["y"] = points[:, 1]
                points_array["z"] = points[:, 2]
                points_array["velocity"] = points[:, 3]

                points_list = points_array.tolist()

                # Pack the data into PointCloud2 message
                tracker_msg.width = len(points_list)
                tracker_msg.row_step = tracker_msg.point_step * len(points_list)
                tracker_msg.data = struct.pack(
                    "<%df" % (len(points_list) * len(tracker_msg.fields)),
                    *points.ravel()
                )

                _flag = 0

            # Publish the data
            self.publisher_detection.publish(det_msg)
            self.publisher_cluster.publish(cluster_msg)
            self.publisher_tracker.publish(tracker_msg)

            self.rate.sleep()

        # Close the serial port
        rospy.loginfo("Exiting the program")
        self.dw.closeSerialPort()
        self.dw.closeFile()


if __name__ == "__main__":
    try:
        point_cloud_publisher = DrwigReader()
        point_cloud_publisher.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Exiting the program")
        sys.exit(0)
