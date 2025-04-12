# /*====================================================================================*/
# /*                                                                                    */
# /*                        Copyright 2021 NXP                                          */
# /*                                                                                    */
# /*   All rights are reserved. Reproduction in whole or in part is prohibited          */
# /*   without the written consent of the copyright owner.                              */
# /*                                                                                    */
# /*   NXP reserves the right to make changes without notice at any time. NXP makes     */
# /*   no warranty, expressed, implied or statutory, including but not limited to any   */
# /*   implied warranty of merchantability or fitness for any particular purpose,       */
# /*   or that the use will not infringe any third party patent, copyright or trademark.*/
# /*   NXP must not be liable for any loss or damage arising from its use.              */
# /*                                                                                    */
# /*====================================================================================*/

from datetime import datetime
from threading import Thread, Condition, Event

import matplotlib.pyplot as plt
import numpy as np
import os
import queue
import serial
import signal
import sys
import zmq
import re

# Arguments: DS-TWR_Unicast.py [i|r] [COM12] [10] [notime] [noplot|nocirplot] [ipc <prefix_file_name> | <bin_path>] [OFFSET=xx]
#   Role of the Rhodes board ("i" for initiator, "r" for responder)
#   Communication Port (e.g. "COM12")
#   Number of valid measurements before stop session (no stop if missing or 0)
#   Don't put date and time in the log
#   "noplot" to not display all the plots or "nocirplot" to display plots of distance and AoA but nor CIR amplitude
#   "ipc" to store all output into file which name is controled by IPC (in this case, timestamp and plots are disabled)
#   or bin_path to store CIR, RFrame and Range Data notifications in binary files (no store if empty)
#   TX Power offset (e.g. "OFFSET=8")


# Default role of the Rhodes board (Initiator|Responder)
rhodes_role = "Responder"

# Default Port value (COMxx)
com_port = "/dev/ttyUSB1"

# Number of valid measurement before stop (0: no stop)
nb_meas = 0

# To add date and time in the log
is_timestamp = True

# To display plot of range data
is_range_plot = True

# To display plot of rframe data
is_cir_plot = True

# To control the name of output log by IPC
is_ipc = False

# Prefixe of output files in IPC mode
prefix_ipc = ""

readOTP = True

data_log = False

channel_ID = [0x09]

# Power offset
power_offset = 0

# To read out calibration values from OTP. 2021.11.30
UWB_EXT_READ_CALIB_DATA_XTAL_CAP = [0x2A, 0x01, 0x00, 0x03, 0x09, 0x01, 0x02]
UWB_EXT_READ_CALIB_DATA_XTAL_CAP_NTF = bytes([0x6A, 0x01, 0x00, 0x05])
UWB_EXT_READ_CALIB_DATA_TX_POWER = [0x2A, 0x01, 0x00, 0x03] + channel_ID + [0x01, 0x01]
UWB_EXT_READ_CALIB_DATA_TX_POWER_NTF = bytes([0x6A, 0x01, 0x00, 0x06])

# Initialize the UWBD for specific platform variant
UWB_SET_BOARD_VARIANT = [0x2E, 0x00, 0x00, 0x02, 0x73, 0x04]

# Reset the UWB device
UWB_RESET_DEVICE = [0x20, 0x00, 0x00, 0x01, 0x00]

# Configure parameters of the UWB device
UWB_CORE_SET_CONFIG = [0x20, 0x04, 0x00, 0x1C,
    0x06,                                             # Number of parameters
    0x01, 0x01, 0x01,                                 # LOW_POWER_MODE
    0xE4, 0x02, 0x01, 0x00,                           # DPD_WAKEUP_SRC
    0xE4, 0x03, 0x01, 0x14,                           # WTX_COUNT_CONFIG
    0xE4, 0x04, 0x02, 0xF4, 0x01,                     # DPD_ENTRY_TIMEOUT
    0xE4, 0x28, 0x04, 0x2F, 0x2F, 0x2F, 0x00,         # TX_PULSE_SHAPE_CONFIG
    0xE4, 0x33, 0x01, 0x01                            # NXP_EXTENDED_NTF_CONFIG
]

UWB_CORE_SET_ANTENNA_TX_IDX_DEFINE = [0x20, 0x04, 0x00, 0x0F,
	0x01, 
	0xE4, 0x61, 0x0B, 0x02,
	0x01, 0x01, 0x00, 0x00, 0x00,   # TX_ANTENNA: 0x01 - MASK selects EF1, EF1 = 0 => Tx-ANT0
	0x02, 0x01, 0x00, 0x01, 0x00    # TX_ANTENNA: 0x02 - MASK selects EF1, EF1 = 1 => NA
]

UWB_CORE_SET_ANTENNA_RX_IDX_DEFINE = [0x20, 0x04, 0x00, 0x1D,
	0x01, 
	0xE4, 0x60, 0x19, 0x04,
	0x01, 0x01, 0x02, 0x00, 0x00, 0x00,   # RX_ANTENNA: 0x01 - Rx1 port – MASK selects EF2, EF2 = 0 => ANT2
	0x02, 0x01, 0x02, 0x00, 0x02, 0x00,   # RX_ANTENNA: 0x02 - Rx1 port – MASK selects EF2, EF2 = 1 => ANT1
	0x03, 0x02, 0x01, 0x00, 0x01, 0x00,   # RX_ANTENNA: 0x03 - Rx2 port – MASK selects EF1, EF1 = 1 => ANT0
	0x04, 0x02, 0x01, 0x00, 0x00, 0x00    # RX_ANTENNA: 0x04 - Rx2 port – MASK selects EF1, EF1 = 0 => NA
]

UWB_CORE_SET_ANTENNAS_RX_PAIR_DEFINE = [0x20, 0x04, 0x00, 0x11,
	0x01, 
	0xE4, 0x62, 0x0D, 0x02,
	0x01, 0x01, 0x03, 0x00, 0x00, 0x00,                 # RX_ANTENNA_PAIR: 0x01 – ANT2 & ANT0 
	0x02, 0x02, 0x03, 0x00, 0x00, 0x00                  # RX_ANTENNA_PAIR: 0x02 – ANT1 & ANT0
]

# Session ID
SESSION_ID = [0x01, 0x00, 0x00, 0x00]

# Create new UWB ranging session
UWB_SESSION_INIT_RANGING = [0x21, 0x00, 0x00, 0x05] + SESSION_ID + [0x00]

# Set Application configurations parameters
# Generic settings
UWB_SESSION_SET_APP_CONFIG = [0x21, 0x03, 0x00, 0x75] + SESSION_ID + [
    0x22,                                             # Number of parameters
#   0x00, 0x01, 0x00,                                 # DEVICE_TYPE
    0x01, 0x01, 0x02,                                 # RANGING_ROUND_USAGE
    0x02, 0x01, 0x00,                                 # STS_CONFIG
    0x03, 0x01, 0x01,                                 # MULTI_NODE_MODE --> multi mode
    0x04, 0x01] + channel_ID + [                      # CHANNEL_NUMBER
    0x05, 0x01, 0x01,                                 # NUMBER_OF_CONTROLEES
#   0x06, 0x02, 0x00, 0x00,                           # DEVICE_MAC_ADDRESS
#   0x07, 0x02, 0x00, 0x00,                           # DST_MAC_ADDRESS
    0x08, 0x02, 0x60, 0x09,                           # SLOT_DURATION (2400 rtsu = 2000us)
    0x09, 0x04, 0xF4, 0x01, 0x00, 0x00,               # RANGING_DURATION (500ms)
    0x0A, 0x04, 0x00, 0x00, 0x00, 0x00,               # STS_INDEX
    0x0B, 0x01, 0x00,                                 # MAC_FCS_TYPE
    0x0C, 0x01, 0x03,                                 # RANGING_ROUND_CONTROL
    0x0D, 0x01, 0x01,                                 # AOA_RESULT_REQ
    0x0E, 0x01, 0x01,                                 # RANGE_DATA_NTF_CONFIG
    0x0F, 0x02, 0x00, 0x00,                           # RANGE_DATA_NTF_PROXIMITY_NEAR
    0x10, 0x02, 0x20, 0x4E,                           # RANGE_DATA_NTF_PROXIMITY_FAR
#   0x11, 0x01, 0x00                                  # DEVICE_ROLE
    0x12, 0x01, 0x03,                                 # RFRAME_CONFIG
    0x13, 0x01, 0x00,                                 # RSSI_REPORTING
    0x14, 0x01, 0x0A,                                 # PREAMBLE_CODE_INDEX
    0x15, 0x01, 0x02,                                 # SFD_ID
    0x16, 0x01, 0x00,                                 # PSDU_DATA_RATE
    0x17, 0x01, 0x01,                                 # PREAMBLE_DURATION
    0x1A, 0x01, 0x01,                                 # RANGING_TIME_STRUCT
    0x1B, 0x01, 0x19,                                 # SLOTS_PER_RR
    0x1C, 0x01, 0x01,                                 # TX_ADAPTIVE_PAYLOAD_POWER #change 03/23 Kato
    0x1E, 0x01, 0x01,                                 # RESPONDER_SLOT_INDEX
    0x1F, 0x01, 0x00,                                 # PRF_MODE
    0x22, 0x01, 0x01,                                 # SCHEDULED_MODE
    0x23, 0x01, 0x00,                                 # KEY_ROTATION
    0x24, 0x01, 0x00,                                 # KEY_ROTATION_RATE
    0x25, 0x01, 0x32,                                 # SESSION_PRIORITY
    0x26, 0x01, 0x00,                                 # MAC_ADDRESS_MODE
####0x27, 0x02, 0x00, 0x00,                           # VENDOR_ID
####0x28, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   # STATIC_STS_IV
    0x29, 0x01, 0x01,                                 # NUMBER_OF_STS_SEGMENTS
    0x2A, 0x02, 0x00, 0x00,                           # MAX_RR_RETRY
####0x2B, 0x04, 0x00, 0x00, 0x00, 0x00,               # UWB_INITIATION_TIME
    0x2C, 0x01, 0x00,                                 # HOPPING_MODE
####0x2D, 0x01, 0x00,                                 # BLOCK_STRIDE_LENGTH
####0x2E, 0x01, 0x00,                                 # RESULT_REPORT_CONFIG
    0x2F, 0x01, 0x00                                  # IN_BAND_TERMINATION_ATTEMPT_COUNT
####0x30, 0x04, 0x00, 0x00, 0x00, 0x00,               # SUB_SESSION_ID
]
    # Proprietary
UWB_SESSION_SET_APP_CONFIG_NXP = [0x21, 0x03, 0x00, 0x31] + SESSION_ID + [
    0x0A,                                             # Number of parameters
    0xE3, 0x01, 0x01, 0x76,                           # CIR_CAPTURE_MODE
####0xE3, 0x02, 0x01, 0x01,                           # MAC_PAYLOAD_ENCRYPTION
####0xE3, 0x03, 0x01, 0x01,                           # RX_ANTENNA_POLARIZATION_OPTION
    0xE3, 0x05, 0x01, 0x03,                           # SESSION_SYNC_ATTEMPTS
    0xE3, 0x06, 0x01, 0x03,                           # SESSION_SHED_ATTEMPTS
    0xE3, 0x07, 0x01, 0x00,                           # SCHED_STATUS_NTF
    0xE3, 0x08, 0x01, 0x00,                           # TX_POWER_DELTA_FCC
    0xE3, 0x09, 0x01, 0x00,                           # TEST_KDF_FEATURE
    0xE3, 0x0B, 0x01, 0x00,                           # TX_POWER_TEMP_COMPENSATION
####0xE3, 0x0C, 0x01, 0x03,                           # WIFI_COEX_MAX_TOLERANCE_COUNT
####0xE3, 0x0D, 0x01, 0x00,                           # ADAPTIVE_HOPPING_THRESHOLD
####0xE3, 0x13, 0x01, 0x00,                           # AUTHENTICITY_TAG
####0xE3, 0x14, 0x02, 0x1E, 0x14,                     # RX_NBIC_CONFIG
    0xE3, 0x15, 0x01, 0x03,                           # MAC_CFG
####0xE3, 0x16, 0x01, 0x00                            # SESSION_INBAND_DATA_TX_BLOCKS
####0xE3, 0x17, 0x01, 0x00                            # SESSION_INBAND_DATA_RX_BLOCKS
####0xE3, 0x18, 0x01, 0x00                            # SUSPEND_RANGING
####0xE3, 0x19, 0x01, 0x00                            # RX_ANTENNA_SELECTION_RFM
####0xE3, 0x1A, 0x01, 0x00                            # DATA_TRANSFER_MODE
    0xE3, 0x1B, 0x02, 0x01, 0x00,                     # ANTENNAS_CONFIGURATION_TX, supported from FW32
    0xE3, 0x1C, 0x04, 0x00, 0x02, 0x01, 0x02          # ANTENNAS_CONFIGURATION_RX, supported from FW32, for 3D AoA
#   0xE3, 0x1C, 0x04, 0x00, 0x02, 0x01, 0x00          # ANTENNAS_CONFIGURATION_RX, supported from FW32, for 2D AoA
]

# Set Application configurations parameters
# Specific settings for Initiator
UWB_SESSION_SET_INITIATOR_CONFIG = [0x21, 0x03, 0x00, 0x13] + SESSION_ID + [
    0x04,                                          # Number of parameters
    0x00, 0x01, 0x01,                              # DEVICE_TYPE: Controller
    0x06, 0x02, 0x11, 0x11,                        # DEVICE_MAC_ADDRESS: 0x1111
    0x07, 0x02, 0x07, 0x10,                        # DST_MAC_ADDRESS: 0x1007
    0x11, 0x01, 0x01                               # DEVICE_ROLE: Initiator
]

# Set Application configurations parameters
# Specific settings for Responder
UWB_SESSION_SET_RESPONDER_CONFIG = [0x21, 0x03, 0x00, 0x13] + SESSION_ID + [ 
    0x04,                                          # Number of parameters
    0x00, 0x01, 0x00,                              # DEVICE_TYPE: Controlee
    0x06, 0x02, 0x00, 0x10,                        # DEVICE_MAC_ADDRESS: 0x1000
    0x07, 0x02, 0x11, 0x11,                        # DST_MAC_ADDRESS: 0x1111
    0x11, 0x01, 0x00                               # DEVICE_ROLE: Responder
]

# Set Debug configurations parameters
UWB_SESSION_SET_DEBUG_CONFIG = [0x21, 0x03, 0x00, 0x3C] + SESSION_ID + [
    0x0C,                                          # Number of parameters
    0xE4, 0x00, 0x02, 0x00, 0x00,                  # THREAD_SECURE
    0xE4, 0x01, 0x02, 0x00, 0x00,                  # THREAD_SECURE_ISR
    0xE4, 0x02, 0x02, 0x00, 0x00,                  # THREAD_NON_SECURE_ISR
    0xE4, 0x03, 0x02, 0x00, 0x00,                  # THREAD_SHELL
    0xE4, 0x04, 0x02, 0x00, 0x00,                  # THREAD_PHY
    0xE4, 0x05, 0x02, 0x00, 0x00,                  # THREAD_RANGING
    0xE4, 0x06, 0x02, 0x00, 0x00,                  # THREAD_SECURE_ELEMENT
    0xE4, 0x10, 0x01, 0x00,                        # DATA_LOGGER_NTF
    0xE4, 0x11, 0x01, 0x00,                        # CIR_LOG_NTF
    0xE4, 0x12, 0x01, 0x00,                        # PSDU_LOG_NTF
    0xE4, 0x13, 0x01, 0x00,                        # RFRAME_LOG_NTF
    0xE4, 0x14, 0x01, 0x00                         # TEST_CONTENTION_RANGING_FEATURE
]

# Start UWB ranging session
UWB_RANGE_START = [0x22, 0x00, 0x00, 0x04] + SESSION_ID

# Stop UWB ranging session
UWB_RANGE_STOP = [0x22, 0x01, 0x00, 0x04] + SESSION_ID

# Deinit UWB session
UWB_SESSION_DEINIT = [0x21, 0x01, 0x00, 0x04] + SESSION_ID

#Set Calibration API
#   0x00: VCO PLL
#   0x01: TX POWER Byte1 (TX_POWER_ID_RMS) , Byte2 (TX_POWER_DELTA_PEAK) 
#   0x02: 38.4 MHz XTAL CAP
#   0x06: MANUAL_TX_POW_CTRL 
#   0x08: AOA_FINE_CALIB_PARAM
#   0x09: TX_TEMPERATURE_COMP
#   0x0C: AOA_ANTENNAS_PDOA_CALIB
#   0x0D: AOA_ANTENNAS_MULTIPOINT_CALIB
#   0x0F: RX_ANT_DELAY_CALIB
#   0x10: PDOA_OFFSET_CALIB
#   0x11: PDOA_MANUFACT_ZERO_OFFSET_CALIB
#   0x12: AOA_THRESHOLD_PDOA
#   0x13: RSSI_CALIB_CONSTANT_HIGH_PWR
#   0x14: RSSI_CALIB_CONSTANT_LOW_PWR
#   0x15: SNR_CALIB_CONSTANT_UNIFIED

UWB_SET_POWER_CALIBRATION = [0x2E, 0x11, 0x00, 0x09] + channel_ID + [ 
    0x17,               # TX_POWER_PER_ANTENNA
	0x02,               # Number of parameters
    0x01, 0x17, 0x00,   # TX_POWER_ID for TX_ANTENNA 0x01
    0x02, 0x00, 0x00    # TX_POWER_ID for TX_ANTENNA 0x02
] 

UWB_SET_CFO_CALIBRATION = [0x2E, 0x11, 0x00, 0x05] + channel_ID + [
    0x02,               # XTAL_CAP
    0x12, 0x12, 0x21
] 
                                      
UWB_SET_CALIBRATION_PDOA_MANUFACT_ZERO_OFFSET_CALIB_CH5 = [0x2E, 0x11, 0x00, 0x09,
    0x05,               # Channel ID
    0x11,               # PDOA_MANUFACT_ZERO_OFFSET_CALIB
	0x02,               # Number of parameters
    #0x01, 0x00, 0x00,   # RX_ANTENNA_PAIR:0x01, in Q9.7 hex 
    #0x02, 0x00, 0x00,   # RX_ANTENNA_PAIR:0x02, in Q9.7 hex 
    0x01, 0xAD, 0x01,   # RX_ANTENNA_PAIR:0x01, in Q9.7 hex # N3 average data come from py#30_31 + 3.35 Murata EVK value
    0x02, 0xB1, 0x0A,   # RX_ANTENNA_PAIR:0x02, in Q9.7 hex # N3 average data come from py#30_31 +21.38 Murata EVK value
]
                                       
UWB_SET_CALIBRATION_PDOA_MANUFACT_ZERO_OFFSET_CALIB_CH9 = [0x2E, 0x11, 0x00, 0x09,
    0x09,               # Channel ID
    0x11,               # PDOA_MANUFACT_ZERO_OFFSET_CALIB
    0x02,               # Number of parameters
    #0x01, 0x00, 0x00,   # RX_ANTENNA_PAIR:0x01, in Q9.7 hex 
    #0x02, 0x00, 0x00,   # RX_ANTENNA_PAIR:0x02, in Q9.7 hex 
    0x01, 0x83, 0xFB,   # RX_ANTENNA_PAIR:0x01, in Q9.7 hex # N3 average data come from py#30_31 -8.98 Murata EVK value
    0x02, 0x50, 0xFB,   # RX_ANTENNA_PAIR:0x02, in Q9.7 hex # N3 average data come from py#30_31 -9.38 Murata EVK value
]

           
UWB_SET_CALIBRATION_PDOA_MULTIPOINT_CALIB_CH5 = [0x2E, 0x11, 0x00, 0x25,
    0x05,                                               # Channel ID
    0x0D,                                               # PDOA_MULTIPOINT_CALIB
	0x02,                                               # Number of parameters
    0x01,                                               # RX_ANTENNA_PAIR : 0x01
    0x80, 0x5C, 0x00, 0x00,                             # azimuth  00°(0x80) elevation -36°(0x5C)
    0x80, 0xA4, 0x00, 0x00,                             # azimuth  00°(0x80) elevation +36°(0xA4) 
    0x5C, 0x80, 0x00, 0x00,                             # azimuth -36°(0x5C) elevation  00°(0x80) 
    0xA4, 0x80, 0x00, 0x00,                             # azimuth +36°(0xA4) elevation  00°(0x80) PDoA +0° (0x0000)
    0x02,                                               # RX_ANTENNA_PAIR : 0x02 
    0x80, 0x5C, 0x00, 0x00,                             # azimuth  00°(0x80) elevation -36°(0x5C)
    0x80, 0xA4, 0x00, 0x00,                             # azimuth  00°(0x80) elevation +36°(0xA4)
    0x5C, 0x80, 0x00, 0x00,                             # azimuth -36°(0x5C) elevation  00°(0x80)
    0xA4, 0x80, 0x00, 0x00                              # azimuth +36°(0xA4) elevation  00°(0x80) PDoA +0° (0x0000)
]
       
UWB_SET_CALIBRATION_PDOA_MULTIPOINT_CALIB_CH9 = [0x2E, 0x11, 0x00, 0x25,
    0x09,                                               # Channel ID
    0x0D,                                               # PDOA_MULTIPOINT_CALIB
	0x02,                                               # Number of parameters
    0x01,                                               # RX_ANTENNA_PAIR : 0x01
    0x80, 0x5C, 0x00, 0x00,                             # azimuth  00°(0x80) elevation -36°(0x5C)
    0x80, 0xA4, 0x00, 0x00,                             # azimuth  00°(0x80) elevation +36°(0xA4) 
    0x5C, 0x80, 0x00, 0x00,                             # azimuth -36°(0x5C) elevation  00°(0x80) 
    0xA4, 0x80, 0x00, 0x00,                             # azimuth +36°(0xA4) elevation  00°(0x80) PDoA +0° (0x0000)
    0x02,                                               # RX_ANTENNA_PAIR : 0x02 
    0x80, 0x5C, 0x00, 0x00,                             # azimuth  00°(0x80) elevation -36°(0x5C)
    0x80, 0xA4, 0x00, 0x00,                             # azimuth  00°(0x80) elevation +36°(0xA4)
    0x5C, 0x80, 0x00, 0x00,                             # azimuth -36°(0x5C) elevation  00°(0x80)
    0xA4, 0x80, 0x00, 0x00                              # azimuth +36°(0xA4) elevation  00°(0x80) PDoA +0° (0x0000)
]

UWB_CORE_SET_RX_ANT_DELAY_CALIB_CH5 = [0x2E, 0x11, 0x00, 0x0F,
	0x05,                                               # Channel ID
	0x0F, 		                                        # RX_ANT_DELAY_CALIB
	0x04,                                               # Number of parameters
	#0x01, 0x05, 0x3B,                                  # RX_ANTENNA:0x01 NXP default value
	#0x02, 0x05, 0x3B,                                  # RX_ANTENNA:0x02 NXP default value
	#0x03, 0x05, 0x3B,                                  # RX_ANTENNA:0x03 NXP default value
	#0x04, 0x05, 0x3B                                   # RX_ANTENNA:0x04 NXP default value
    0x01, 0xDC, 0x3A,                                   # RX_ANTENNA:0x01 Murata EVK value
    0x02, 0xDC, 0x3A,                                   # RX_ANTENNA:0x02 Murata EVK value
    0x03, 0xDC, 0x3A,                                   # RX_ANTENNA:0x03 Murata EVK value
    0x04, 0xDC, 0x3A                                    # RX_ANTENNA:0x04 Murata EVK value
]

UWB_CORE_SET_RX_ANT_DELAY_CALIB_CH9 = [0x2E, 0x11, 0x00, 0x0F,
	0x09,                                               # Channel ID
	0x0F, 		                                        # RX_ANT_DELAY_CALIB
	0x04,                                               # Number of parameters
	#0x01, 0xE9, 0x3A,                                  # RX_ANTENNA:0x01 NXP default value
	#0x02, 0xE9, 0x3A,                                  # RX_ANTENNA:0x02 NXP default value
	#0x03, 0xE9, 0x3A,                                  # RX_ANTENNA:0x03 NXP default value
	#0x04, 0xE9, 0x3A                                   # RX_ANTENNA:0x04 NXP default value
    0x01, 0xC5, 0x3A,                                   # RX_ANTENNA:0x01 Murata EVK value
    0x02, 0xC5, 0x3A,                                   # RX_ANTENNA:0x02 Murata EVK value
    0x03, 0xC5, 0x3A,                                   # RX_ANTENNA:0x03 Murata EVK value
    0x04, 0xC5, 0x3A                                    # RX_ANTENNA:0x04 Murata EVK value
]
          
UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR1_CH5 = [0x2E, 0x11, 0x00, 0xF6,
	0x05,                                               # Channel ID
	0x0C,                                               # AOA_ANTENNAS_PDOA_CALIB
	0x01,                                               # 
	0x01,                                               # RX_ANTENNA_PAIR:0x01 
    # Pan  -60,        -48,        -36,        -24,        -12,          0,        +12,        +24,        +36,        +48,         +60,
    0x2E, 0x32, 0xBB, 0x24, 0xE7, 0x17, 0xD9, 0x0E, 0x3F, 0x02, 0x6C, 0xFB, 0x2E, 0xF1, 0xA0, 0xE9, 0xEA, 0xE4, 0xFB, 0xDB, 0x71, 0xD3, 
    0xEF, 0x2F, 0x9A, 0x25, 0xD5, 0x18, 0xD9, 0x0D, 0x2C, 0x02, 0x69, 0xFA, 0xDA, 0xF0, 0xFA, 0xE7, 0xA9, 0xE1, 0x2C, 0xD9, 0xA3, 0xCF, 
    0x74, 0x2E, 0x3D, 0x24, 0xA2, 0x1A, 0xAC, 0x0D, 0x59, 0x01, 0xEB, 0xF8, 0xBD, 0xEF, 0x3D, 0xE6, 0xD0, 0xDE, 0xDC, 0xD6, 0xD2, 0xCD, 
    0x87, 0x2E, 0x9E, 0x22, 0x64, 0x1A, 0xF8, 0x0F, 0x2C, 0x01, 0x53, 0xF8, 0x84, 0xED, 0x56, 0xE4, 0x35, 0xDD, 0x3A, 0xD6, 0x82, 0xCE, 
    0x2B, 0x2D, 0x24, 0x23, 0x83, 0x18, 0xB1, 0x10, 0x8D, 0x02, 0x79, 0xF9, 0xB8, 0xEC, 0xFE, 0xE3, 0x3C, 0xDC, 0xE7, 0xD5, 0xCD, 0xCF, 
    0x52, 0x2A, 0xEF, 0x22, 0xEA, 0x18, 0xB6, 0x0D, 0x8E, 0x03, 0xF4, 0xF9, 0x95, 0xEE, 0x46, 0xE5, 0xB4, 0xDB, 0x31, 0xD5, 0xE8, 0xCF, 
    0x19, 0x27, 0xF1, 0x1F, 0x28, 0x1A, 0x6A, 0x0E, 0x75, 0x05, 0x3C, 0xFB, 0xC0, 0xEF, 0xA0, 0xE6, 0x71, 0xDD, 0x99, 0xD4, 0xA0, 0xD1, 
    0x5E, 0x25, 0xF7, 0x1E, 0xD7, 0x15, 0xFD, 0x0A, 0xEE, 0x02, 0xB3, 0xFA, 0x77, 0xF0, 0x1E, 0xE7, 0xE3, 0xDE, 0xEB, 0xD6, 0xB9, 0xD3, 
    0xF6, 0x23, 0xD0, 0x1E, 0x10, 0x12, 0xB5, 0x00, 0xA4, 0xFB, 0xCB, 0xF5, 0x6A, 0xF1, 0x45, 0xEA, 0xAE, 0xDF, 0xAB, 0xD6, 0x0D, 0xD4, 
    0x3D, 0x22, 0xE4, 0x19, 0xAA, 0x11, 0x1B, 0x04, 0x36, 0xFB, 0x71, 0xF2, 0x07, 0xEE, 0xBF, 0xE6, 0x3C, 0xDE, 0xBB, 0xD4, 0x12, 0xD3, 
    0x94, 0x1F, 0xB5, 0x10, 0xC7, 0x0C, 0x5B, 0x0A, 0x92, 0x00, 0x5D, 0xEF, 0x6B, 0xE7, 0xB5, 0xDE, 0x6A, 0xDC, 0xE6, 0xD3, 0x04, 0xD3
]

UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR2_CH5 = [0x2E, 0x11, 0x00, 0xF6,
	0x05,                                               # Channel ID
	0x0C,                                               # AOA_ANTENNAS_PDOA_CALIB
	0x01,                                               # 
	0x02,                                               # RX_ANTENNA_PAIR:0x02 
    # Pan  -60,        -48,        -36,        -24,        -12,          0,        +12,        +24,        +36,        +48,         +60,
    0x83, 0xF7, 0x3E, 0xF8, 0xF0, 0xFB, 0xA5, 0x00, 0x74, 0x04, 0x64, 0x07, 0x20, 0x0A, 0x8D, 0x0D, 0x08, 0x10, 0xE7, 0x0F, 0xEC, 0x0E, 
    0xC7, 0xE6, 0xDE, 0xE9, 0x66, 0xF0, 0x43, 0xF7, 0x32, 0xFF, 0xE6, 0x06, 0xD7, 0x0B, 0xB1, 0x11, 0xF5, 0x18, 0x5D, 0x1A, 0xC2, 0x14, 
    0x62, 0xDF, 0xAD, 0xE3, 0x86, 0xEB, 0xCE, 0xF4, 0x82, 0xFD, 0x15, 0x07, 0x28, 0x11, 0x8D, 0x17, 0x9F, 0x1E, 0xA5, 0x26, 0xDF, 0x22, 
    0x13, 0xDA, 0x38, 0xDF, 0xDC, 0xE5, 0x2E, 0xF1, 0x28, 0xFE, 0x18, 0x07, 0x65, 0x12, 0x4E, 0x20, 0x7D, 0x27, 0x1F, 0x2F, 0x3B, 0x39, 
    0x89, 0xD1, 0x84, 0xD9, 0xF0, 0xDF, 0xD0, 0xE9, 0xA4, 0xF8, 0xFA, 0x05, 0x05, 0x14, 0xB3, 0x23, 0x06, 0x2E, 0xB3, 0x32, 0x48, 0x39, 
    0x6A, 0xCE, 0xAA, 0xD6, 0xC7, 0xDD, 0xA8, 0xE7, 0xF1, 0xF6, 0x0E, 0x05, 0x49, 0x12, 0x7E, 0x24, 0xBF, 0x2F, 0xAF, 0x33, 0xF6, 0x36, 
    0xB8, 0xC6, 0xA3, 0xD1, 0x3D, 0xDB, 0xA3, 0xE5, 0x61, 0xF3, 0x47, 0x02, 0x89, 0x10, 0x9D, 0x20, 0x05, 0x2E, 0x90, 0x34, 0x54, 0x34, 
    0xE1, 0xC8, 0x91, 0xD1, 0xFB, 0xDB, 0xD8, 0xE6, 0x7A, 0xF4, 0x4C, 0x00, 0x94, 0x0D, 0x80, 0x1B, 0x0C, 0x27, 0x83, 0x2D, 0x09, 0x2E, 
    0x63, 0xCC, 0x8A, 0xD1, 0x76, 0xDA, 0xD4, 0xE6, 0xFD, 0xF2, 0x0E, 0xFF, 0x68, 0x0B, 0xFD, 0x13, 0x84, 0x1B, 0x5D, 0x24, 0xE9, 0x28, 
    0x76, 0xCB, 0x15, 0xD2, 0x98, 0xDC, 0x3A, 0xE9, 0x6F, 0xF5, 0x52, 0xFF, 0x35, 0x07, 0xE7, 0x0D, 0x05, 0x12, 0xC4, 0x14, 0x29, 0x17, 
    0xB0, 0xD6, 0xB0, 0xDB, 0x5F, 0xE3, 0x30, 0xED, 0x60, 0xF6, 0x84, 0xFD, 0xAA, 0x04, 0xC7, 0x0A, 0x00, 0x0E, 0x24, 0x0F, 0x43, 0x11
]

UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR1_CH9 = [0x2E, 0x11, 0x00, 0xF6,
	0x09,                                               # Channel ID
	0x0C,                                               # AOA_ANTENNAS_PDOA_CALIB
	0x01,                                               # 
	0x01,                                               # RX_ANTENNA_PAIR:0x01 
    # Pan  -60,        -48,        -36,        -24,        -12,          0,        +12,        +24,        +36,        +48,        +60,
    0x2C, 0x3F, 0xEE, 0x33, 0xD5, 0x24, 0x98, 0x19, 0xDC, 0x0C, 0xE5, 0xFB, 0x3B, 0xEA, 0x0C, 0xDB, 0xA9, 0xD2, 0x79, 0xC7, 0x29, 0xBB, 
    0x61, 0x3C, 0xD9, 0x33, 0x56, 0x24, 0x8B, 0x19, 0x0C, 0x0A, 0x53, 0xFC, 0x45, 0xED, 0x75, 0xDE, 0x7D, 0xD2, 0x75, 0xC7, 0x59, 0xBA, 
    0x3F, 0x3A, 0xAD, 0x2F, 0xF3, 0x26, 0xD5, 0x1A, 0x1C, 0x0A, 0x71, 0xFA, 0x58, 0xEC, 0xE1, 0xDD, 0x11, 0xCF, 0xFE, 0xC5, 0x77, 0xBA, 
    0x75, 0x3B, 0x9A, 0x2C, 0xDB, 0x24, 0xC7, 0x17, 0x63, 0x09, 0x16, 0xF7, 0xF3, 0xE7, 0x84, 0xDB, 0xA1, 0xCC, 0xD4, 0xC4, 0xC4, 0xBB, 
    0x9B, 0x3F, 0x9D, 0x30, 0x16, 0x26, 0x8E, 0x16, 0x1C, 0x09, 0x80, 0xFA, 0xDD, 0xE9, 0x0D, 0xDD, 0x1A, 0xCE, 0x97, 0xC5, 0xCD, 0xBE, 
    0x49, 0x40, 0xD1, 0x35, 0x17, 0x2B, 0xF2, 0x1B, 0xCF, 0x0A, 0xB2, 0xFC, 0xA2, 0xEB, 0x98, 0xDD, 0xDB, 0xD1, 0x11, 0xC8, 0xF9, 0xC1, 
    0x5A, 0x3C, 0x64, 0x32, 0x0D, 0x26, 0xA1, 0x1A, 0x7E, 0x0A, 0x82, 0xFC, 0x09, 0xEC, 0xB8, 0xDD, 0x8F, 0xD2, 0x2B, 0xC7, 0xC6, 0xC2, 
    0xA4, 0x39, 0x4E, 0x33, 0x2D, 0x28, 0x90, 0x1D, 0x9F, 0x11, 0x9E, 0x02, 0x05, 0xF2, 0x8E, 0xE1, 0x3E, 0xD4, 0x4A, 0xC6, 0xBA, 0xBF, 
    0x5A, 0x3E, 0xC8, 0x3C, 0x3F, 0x2D, 0xFC, 0x22, 0x9F, 0x10, 0x91, 0xFE, 0xAB, 0xEF, 0xE6, 0xE1, 0x1B, 0xD5, 0x7A, 0xC8, 0x89, 0xBC, 
    0xA7, 0x45, 0x6B, 0x3C, 0x52, 0x2D, 0x8C, 0x20, 0x4D, 0x10, 0x22, 0xFC, 0xC4, 0xE8, 0x35, 0xDC, 0xD7, 0xD4, 0x7F, 0xC8, 0xAC, 0xBA, 
    0xCF, 0x44, 0x89, 0x35, 0x22, 0x30, 0x47, 0x25, 0xF2, 0x11, 0x85, 0xFB, 0xC0, 0xEB, 0x1B, 0xD9, 0x52, 0xCE, 0x49, 0xC7, 0xF1, 0xBB
]

UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR2_CH9 = [0x2E, 0x11, 0x00, 0xF6,
	0x09,                                               # Channel ID
	0x0C,                                               # AOA_ANTENNAS_PDOA_CALIB
	0x01,                                               # 
	0x02,                                               # RX_ANTENNA_PAIR:0x02 
    # Pan  -60,        -48,        -36,        -24,        -12,          0,        +12,        +24,        +36,        +48,        +60,
    0x37, 0xE2, 0x02, 0xE3, 0x21, 0xE3, 0xAB, 0xEB, 0xF3, 0xF9, 0x1D, 0x06, 0xA0, 0x0B, 0x31, 0x0B, 0x6D, 0x0F, 0xE9, 0x1A, 0x2E, 0x24, 
    0x96, 0xD7, 0xA6, 0xDF, 0x81, 0xDF, 0x14, 0xE1, 0x9F, 0xEF, 0x12, 0x05, 0xC5, 0x0E, 0x52, 0x10, 0xC8, 0x1B, 0x27, 0x27, 0xE4, 0x2B, 
    0x58, 0xC8, 0x72, 0xD2, 0x6E, 0xE3, 0x29, 0xE9, 0xC5, 0xED, 0x62, 0x01, 0xDF, 0x0E, 0x97, 0x14, 0x03, 0x21, 0xDB, 0x2D, 0x89, 0x36, 
    0xFC, 0xC6, 0x99, 0xCC, 0xA3, 0xDA, 0x5C, 0xE9, 0x95, 0xEB, 0xE7, 0xFA, 0x14, 0x0F, 0x41, 0x18, 0xF4, 0x27, 0xB0, 0x35, 0x75, 0x3E, 
    0x38, 0xC9, 0x99, 0xCA, 0x50, 0xD3, 0x2D, 0xE5, 0x11, 0xEC, 0x34, 0xF8, 0xF3, 0x0C, 0xD4, 0x19, 0x80, 0x29, 0x51, 0x37, 0x77, 0x40, 
    0xC8, 0xC8, 0x3C, 0xCC, 0x9C, 0xD0, 0x75, 0xDF, 0xE8, 0xEA, 0x4F, 0xF7, 0x0F, 0x0B, 0xBA, 0x1B, 0xF1, 0x28, 0x03, 0x37, 0x12, 0x41, 
    0x35, 0xC3, 0xCB, 0xCA, 0x04, 0xD1, 0xBC, 0xDD, 0x67, 0xE8, 0x94, 0xF5, 0x5F, 0x07, 0x19, 0x1A, 0xE2, 0x28, 0x53, 0x33, 0x55, 0x3E, 
    0xEE, 0xBE, 0xBC, 0xC7, 0x4A, 0xD2, 0x80, 0xDC, 0x16, 0xE6, 0x90, 0xF5, 0xD2, 0x03, 0x16, 0x15, 0xA0, 0x25, 0xBB, 0x2D, 0xD6, 0x36, 
    0x31, 0xC8, 0xFD, 0xCD, 0x4B, 0xD3, 0x1B, 0xD9, 0xD4, 0xE3, 0x4F, 0xF5, 0xF0, 0x00, 0x75, 0x0D, 0x96, 0x1A, 0xF3, 0x25, 0x39, 0x2E, 
    0xB6, 0xCD, 0x84, 0xD5, 0x9E, 0xDA, 0x88, 0xDE, 0x4E, 0xE8, 0x28, 0xF6, 0xE1, 0xFE, 0xF1, 0x05, 0x02, 0x13, 0x4E, 0x1C, 0x1A, 0x20, 
    0xF5, 0xD5, 0x12, 0xD7, 0x80, 0xD9, 0x34, 0xE0, 0xF1, 0xEB, 0x18, 0xF8, 0x6B, 0x00, 0x93, 0x03, 0x64, 0x07, 0x38, 0x0C, 0xF7, 0x11
]

UWB_CORE_SET_PDOA_OFFSET_CALIB_CH5 = [0x2E, 0x11, 0x00, 0x09,
	0x05,                                                   # Channel ID
	0x10,                                                   # PDOA_OFFSET_CALIB
	0x02,                                                   # Number of parameters
	0x01, 0xD6, 0x07,                                       # RX_ANTENNA_PAIR:0x01
    0x02, 0xF4, 0x05                                        # RX_ANTENNA_PAIR:0x02
]

UWB_CORE_SET_PDOA_OFFSET_CALIB_CH9 = [0x2E, 0x11, 0x00, 0x09,
	0x09,                                                   # Channel ID
	0x10,                                                   # PDOA_OFFSET_CALIB
	0x02,                                                   # Number of parameters
	0x01, 0x0F, 0xFF,                                       # RX_ANTENNA_PAIR:0x01
    0x02, 0x97, 0x04                                        # RX_ANTENNA_PAIR:0x02
]

UWB_CORE_SET_AOA_THRESHOLD_PDOA_CH5 = [0x2E, 0x11, 0x00, 0x09,
	0x05,                                                   # Channel ID
	0x12,                                                   # AOA_THRESHOLD_PDOA
	0x02,                                                   # Number of parameters
	0x01, 0xD7, 0xAD,                                       # RX_ANTENNA_PAIR:0x01
    0x02, 0xF5, 0xAB                                        # RX_ANTENNA_PAIR:0x02
]

UWB_CORE_SET_AOA_THRESHOLD_PDOA_CH9 = [0x2E, 0x11, 0x00, 0x09,
	0x09,                                                   # Channel ID
	0x12,                                                   # AOA_THRESHOLD_PDOA
	0x02,                                                   # Number of parameters
	0x01, 0x0E, 0x59,                                       # RX_ANTENNA_PAIR:0x01
    0x02, 0x98, 0xAA                                        # RX_ANTENNA_PAIR:0x02
]

###########################################################
class SIGINThandler():
    def __init__(self):
        self.sigint = False
    
    def signal_handler(self, signal, frame):
        print("You pressed Ctrl+C!")
        self.sigint = True


class SessionStates():
    def __init__(self):
        self.allow_config = Event()
        self.allow_start = Event()
        self.allow_stop = Event()
        self.allow_end = Event()
    
    def set(self, status):
        if (status == 0x00):
            # SESSION_STATE_INIT
            self.allow_config.set()
            self.allow_start.clear()
            self.allow_stop.clear()
            self.allow_end.clear()
        
        if (status == 0x01):
            # SESSION_STATE_DEINIT
            self.allow_config.clear()
            self.allow_start.clear()
            self.allow_stop.clear()
            self.allow_end.set()
        
        if (status == 0x02):
            # SESSION_STATE_ACTIVE
            self.allow_config.set()
            self.allow_start.set()
            self.allow_stop.set()
            self.allow_end.clear()
        
        if (status == 0x03):
            # SESSION_STATE_IDLE
            self.allow_config.set()
            self.allow_start.set()
            self.allow_stop.clear()
            self.allow_end.clear()
        
        if (status == 0xFF):
            # SESSION_ERROR
            self.allow_config.clear()
            self.allow_start.clear()
            self.allow_stop.clear()
            self.allow_end.clear()
    
    def set_all(self):
        self.allow_config.set()
        self.allow_start.set()
        self.allow_stop.set()
        self.allow_end.set()


###########################################################
serial_port = serial.Serial()
command_queue = queue.Queue(maxsize=100)
session_status = SessionStates()
write_wait = Condition()
go_stop = Event()
stop_write_thread = False
stop_read_thread = False
stop_ipc_thread = False
retry_cmd = False
meas_idx = 1
bin_store = False
cir0_file = ""
cir1_file = ""
rframe_session = ""
rframe_nb = 0
rframe_meas = []
file_ipc = None
socket = None
file_data_log = None

# Not draw when index is negative
range_plot = {"index": -1, "valid": False, "nlos": 0, "distance": 0,
              "azimuth": 0, "elevation": 0, "avg_azimuth": 0, "avg_elevation": 0}

# Not draw when number of measurement is zero
cir_plot = {"nb_meas": 0, "mappings": [], "cir_samples": []}


# Output string on STDOUT or store into file depending of IPC mode
# Return True is success to write string into file
def output(string):
    global is_ipc
    global file_ipc
    
    if (is_ipc):
        if ((file_ipc is not None) and (not file_ipc.closed) and (file_ipc.writable())):
            # File available for write
            file_ipc.write(string + "\n")
        
            return True
    else:
        # Output string on STDOUT
        print(string)
        
    return False


def deg_to_rad(angle_deg):
    return (angle_deg * np.pi / 180)


def init_plots(title):
    global is_cir_plot
    
    plt.ion()  # Interactive mode
    plt.rcParams["toolbar"] = "None"  # Remove toolbar
    
    # Define figure of all plots
    fig = plt.figure()
    fig.canvas.set_window_title(title)
    fig.subplots_adjust(wspace=0.4, hspace=0.6)
    
    if (is_cir_plot):
        nb_row = 3
    else:
        nb_row = 2
    
    # Define distance plot
    plot_dist = fig.add_subplot(nb_row, 2, (1, 2))
    plot_dist.clear()
    plot_dist.set_title("Distance")
    plot_dist.set_xlabel("Sample #")
    plot_dist.set_xlim(1, 100)
    plot_dist.set_ylabel("cm")
    plot_dist.set_ylim(0, 1000)
    
    # Define Azimuth plot
    plot_azimuth = fig.add_subplot(nb_row, 2, 3, polar=True)
    plot_azimuth.clear()
    plot_azimuth.set_xlabel("Azimuth")
    plot_azimuth.set_theta_zero_location("N")
    plot_azimuth.set_theta_direction("clockwise")
    plot_azimuth.set_thetalim(deg_to_rad(-90), deg_to_rad(90))
    plot_azimuth.set_xticks([deg_to_rad(-90), deg_to_rad(-60), deg_to_rad(-30), 0,
                             deg_to_rad(30), deg_to_rad(60), deg_to_rad(90)])
    plot_azimuth.set_ylim(0, 1)
    plot_azimuth.set_yticks([])
    
    # Define Elevation plot
    plot_elevation = fig.add_subplot(nb_row, 2, 4, polar=True)
    plot_elevation.clear()
    plot_elevation.set_xlabel("Elevation")
    plot_elevation.set_theta_zero_location("E")
    plot_elevation.set_theta_direction("counterclockwise")
    plot_elevation.set_thetalim(deg_to_rad(-90), deg_to_rad(90))
    plot_elevation.set_xticks([deg_to_rad(-90), deg_to_rad(-60), deg_to_rad(-30), 0,
                               deg_to_rad(30), deg_to_rad(60), deg_to_rad(90)])
    plot_elevation.set_ylim(0, 1)
    plot_elevation.set_yticks([])
    
    if (nb_row == 3):
        # Define CIR plot
        plot_cir = fig.add_subplot(nb_row, 2, (5, 6))
        plot_cir.clear()
        plot_cir.set_title("Amplitude")
        plot_cir.set_xlabel("First Path Index")
        plot_cir.set_xlim(-8, 7)
        plot_cir.set_xticks(range(-8, 8, 2))
        plot_cir.set_ylim(0, 3000)
    else:
        plot_cir = None
    
    plt.show()  # Not blocking as Interactive mode
    plt.pause(0.01)  # Run the GUI event loop 10ms to draw the plots
    
    # Return plots for reference when drawing
    return (plot_dist, plot_azimuth, plot_elevation, plot_cir)


def draw_distance(plot_dist, data):
    if (data["index"] == 0):
        # Clear plot and restore X axis
        plot_dist.lines = []
        plot_dist.set_xlim(1, 100)
    
    # Slide X axis when reach the right border
    xmin, xmax = plot_dist.get_xlim()
    
    if (data["index"] > xmax):
        plot_dist.set_xlim(data["index"] - 100, data["index"])
    
    # Increase Y axis if distance is greater than the max
    ymin, ymax = plot_dist.get_ylim()
    
    if (data["distance"] > ymax):
        plot_dist.set_ylim(0, data["distance"] + 50)
    
    # Set the color of the point
    if (data["nlos"] == 0):
        # LoS => point marker, blue
        coloredPoint = ".b"
    else:
        # NLoS or invalid => point marker, red
        coloredPoint = ".r"
    
    plot_dist.plot(data["index"], data["distance"], coloredPoint)


def draw_aoa(plot_aoa, angle, avg):
    # Remove previous arrow and cicle
    plot_aoa.lines = []
    
    # Draw new arrow (solid line style, blue)
    plot_aoa.plot([0, deg_to_rad(angle), deg_to_rad(angle - 10), deg_to_rad(angle),
                   deg_to_rad(angle + 10), deg_to_rad(angle)],
                  [0, 0.95, 0.9, 1, 0.9, 0.95], "-b")
    
    # Draw new cicle marker, blue
    plot_aoa.plot(deg_to_rad(avg), 1, "ob")


def draw_cir(plot_cir, data):
    # Remove previous lines
    plot_cir.lines = []
    
    coloredLine = ""
    legend = ""
    
    # Store slot index of first measurement
    first_slot = data["mappings"][0] & 0x3F
    
    for data_idx in range(0, data["nb_meas"]):
        slot = data["mappings"][data_idx] & 0x3F
        
        # Set the style of the line
        if (slot == first_slot):
            # Solid line
            coloredLine = "-"
        else:
            # Dashed line
            coloredLine = "--"
        
        legend = "Slot "
        legend += format(slot)
        
        # Set the color of the line
        if (data["mappings"][data_idx] >= 128):
            # RX2 => Red
            coloredLine += "r"
            legend += " RX2"
        else:
            # RX1 => Green
            coloredLine += "g"
            legend += " RX1"
        
        plot_cir.plot(range(-8, 8), data["cir_samples"][data_idx], coloredLine, label=legend)
        plot_cir.legend(loc="upper left")


def extract_seq_cnt(byte_array):
    return int((byte_array[3] << 24) + (byte_array[2] << 16) + (byte_array[1] << 8) + byte_array[0])


def extract_nlos(byte_array):
    return int(byte_array[28])


def extract_distance(byte_array):
    return int((byte_array[30] << 8) + byte_array[29])


def extract_azimuth(byte_array):
    return int((byte_array[32] << 8) + byte_array[31])


def extract_azimuth_fom(byte_array):
    return int(byte_array[33])


def extract_elevation(byte_array):
    return int((byte_array[35] << 8) + byte_array[34])


def extract_elevation_fom(byte_array):
    return int(byte_array[36])


def extract_cir(byte_array):
    cir_raw = []
    
    for idx in range(0, len(byte_array), 4):
        cir_sample = byte_array[idx:idx + 4]
        
        real = twos_comp(int((cir_sample[1] << 8) + cir_sample[0]), 16)
        imaginary = twos_comp(int((cir_sample[3] << 8) + cir_sample[2]), 16)
        
        cir_raw.append(real + 1j * imaginary)
    
    return np.abs(cir_raw)

def extract_pdoa1(byte_array):
    return int((byte_array[67] << 8) + byte_array[66])
    
def extract_pdoa2(byte_array):
    return int((byte_array[71] << 8) + byte_array[70])

def twos_comp(val, bits):
    # Compute the 2's complement of integer val with the width of bits
    if (val & (1 << (bits - 1))) != 0:  # If sign bit is set
        val = val - (1 << bits)  # Compute negative value
    return val


def convert_qformat_to_float(q_in, n_ints, n_fracs, round_of=2):
    bits = n_ints + n_fracs
    
    # Compute the 2's complement of integer q_in with the width of n_ints + n_fracs
    if (q_in & (1 << (bits - 1))) != 0:  # If sign bit is set
        q_in = q_in - (1 << bits)  # Compute negative value
    
    # Divide by 2^n_fracs
    frac = q_in / (1 << n_fracs)
    
    # Return rounded value
    return round(frac, round_of)


def write_to_serial_port():
    global stop_write_thread
    global command_queue
    global session_status
    global go_stop
    global write_wait
    global serial_port
    global retry_cmd
    global is_timestamp
    global readOTP
    
    output("Write to serial port started")
    while (not stop_write_thread):
        if (retry_cmd):
            retry_cmd = False
        else:
            uci_command = command_queue.get()
        
        if (uci_command[0] == 0xFF and uci_command[1] == 0xFF):
            break
        
        usb_out_packet = bytearray()
        usb_out_packet.append(0x01)
        usb_out_packet.append(0x00)
        usb_out_packet.append(len(uci_command))
        usb_out_packet.extend(uci_command)
        
        if (uci_command[0] == 0x21 and uci_command[1] == 0x03):
            # Wait Session State Initialized to send APP Configs
            session_status.allow_config.wait()
        if (uci_command[0] == 0x22 and uci_command[1] == 0x00):
            # Wait Session State Idle to start ranging
            session_status.allow_start.wait()
        if (uci_command[0] == 0x22 and uci_command[1] == 0x01):
            # Wait Session State Activated
            session_status.allow_stop.wait()
            # Wait reach limit of measurements to stop ranging
            go_stop.wait()
            
        write_wait.acquire()  # Acquire Lock to avoid mixing in output
        if serial_port.isOpen():
            if (is_timestamp):
                output(datetime.now().isoformat(sep=" ", timespec="milliseconds") + "NXPUCIX => " + \
                      "".join("{:02x} ".format(x) for x in uci_command))
            else:
                output("NXPUCIX => " + "".join("{:02x} ".format(x) for x in uci_command))
            
            serial_port.write(serial.to_bytes(usb_out_packet))
            # Wait the reception of RSP or timeout of 0.25s before allowing send of new CMD
            notified = write_wait.wait(0.25)  
            if (not (notified)): retry_cmd = True  # Repeat command if timeout
        write_wait.release()
    
    output("Write to serial port exited")


def read_from_serial_port():
    global stop_read_thread
    global serial_port
    global write_wait
    global retry_cmd
    global session_status
    global go_stop
    global nb_meas
    global is_timestamp
    global meas_idx
    global bin_store
    global cir0_file
    global cir1_file
    global rframe_session
    global rframe_nb
    global rframe_meas
    global range_plot
    global is_ipc
    global socket
    global UWB_SET_POWER_CALIBRATION
    global UWB_SET_CFO_CALIBRATION
    
    meas_nlos = 0
    meas_distance = 0
    meas_azimuth = 0
    meas_azimuth_fom = 0
    meas_elevation = 0
    meas_elevation_fom = 0
    meas_pdoa1 = 0
    meas_pdoa2 = 0
    avg_window_size = 10
    hist_distance = []
    hist_azimuth = []
    hist_elevation = []
    hist_pdoa1 = []
    hist_pdoa2 = []
    is_stored = False
    
    output("Read from serial port started")
    while (not stop_read_thread):
        if serial_port.isOpen():
            if serial_port.isOpen():
                uci_hdr = serial_port.read(4)  # Read header of UCI frame
                write_wait.acquire()  # Acquire Lock to avoid mixing in output
                if len(uci_hdr) == 4:
                    count = uci_hdr[3]
                    if (uci_hdr[1] & 0x80) == 0x80:
                        # Extended length
                        count = int((uci_hdr[3] << 8) + uci_hdr[2])
                    
                    if count > 0:
                        if serial_port.isOpen():
                            uci_payload = serial_port.read(count)  # Read payload of UCI frame
                            
                            if (is_timestamp):
                                is_stored = output(datetime.now().isoformat(sep=" ", timespec="milliseconds") + \
                                      "NXPUCIR <= " + "".join("{:02x} ".format(h) for h in uci_hdr) + \
                                      "".join("{:02x} ".format(p) for p in uci_payload))
                            else:
                                is_stored = output("NXPUCIR <= " + "".join("{:02x} ".format(h) for h in uci_hdr) + \
                                      "".join("{:02x} ".format(p) for p in uci_payload))
                            
                            if len(uci_payload) == count:
                                if (uci_hdr[0] & 0xF0) == 0x40: write_wait.notify()  # Notify the reception of RSP
                                
                                if (uci_hdr[0] == 0x60 and uci_hdr[1] == 0x07 and uci_hdr[3] == 0x01 and \
                                        uci_payload[0] == 0x0A):
                                    # Command retry without wait response
                                    retry_cmd = True
                                    write_wait.notify()
                                
                                if (uci_hdr == UWB_EXT_READ_CALIB_DATA_XTAL_CAP_NTF):
                                    #print("XTAL_CAP in OTP 0x%02X 0x%02X 0x%02X" % (uci_payload[2], uci_payload[3], uci_payload[4]))
                                    #CALIB_DATA_XTAL_CAP = uci_payload
                                    UWB_SET_CFO_CALIBRATION[6] = uci_payload[2]
                                    UWB_SET_CFO_CALIBRATION[7] = uci_payload[3]
                                    UWB_SET_CFO_CALIBRATION[8] = uci_payload[4]

                                if (uci_hdr == UWB_EXT_READ_CALIB_DATA_TX_POWER_NTF):
                                    #print("TX_POWER in OTP ", end="")
                                    #CALIB_DATA_TX_POWER = uci_payload
                                    UWB_SET_POWER_CALIBRATION[8] = uci_payload[2] + power_offset + int((2.1-0.6+0.5)*4) # Murata EVK case: ANT+2.1dBi, trace loss 0.6dB 
                                    UWB_SET_POWER_CALIBRATION[9] = uci_payload[3]
                                    #print(" 0x%02X 0x%02X" % (uci_payload[2], uci_payload[3]))

                                if (uci_hdr[0] == 0x61 and uci_hdr[1] == 0x02 and uci_hdr[3] == 0x06):
                                    # Change Session state
                                    session_status.set(uci_payload[4])
                                    
                                    if (uci_payload[5] == 0x01):
                                        # Session termination on max RR Retry
                                        go_stop.set()
                                        
                                    if ((is_ipc) and (uci_payload[4] == 0x02)):
                                        # Ranging is active
                                        file_ipc.close()
                                        
                                        # indicate to server the start of ranging
                                        if(socket is not None):
                                            try:
                                                socket.send_string("started")
                                            except:
                                                print("Fail to send started on socket")
                                
                                if (bin_store and uci_hdr[0] == 0x7E and uci_hdr[1] == 0x84):
                                    # DBG_CIR0_LOG_NTF with PBF = 1 and extended payload length
                                    
                                    if (cir0_file == ""):
                                        # First segment => create file and write payload without Session ID
                                        
                                        cir0_file = "uwb_data_session_"
                                        cir0_file += format((uci_payload[3] << 24) + (uci_payload[2] << 16) + \
                                                            (uci_payload[1] << 8) + uci_payload[0])
                                        cir0_file += "_"
                                        cir0_file += format(meas_idx)
                                        cir0_file += "_CIR0.log"
                                        
                                        with open(cir0_file, "wb") as data_file:
                                            data_file.write(uci_payload[4:])
                                    
                                    else:
                                        # Second segment => append file
                                        with open(cir0_file, "ab") as data_file:
                                            data_file.write(uci_payload)
                                
                                if (bin_store and uci_hdr[0] == 0x6E and uci_hdr[1] == 0x04):
                                    # DBG_CIR0_LOG_NTF with PBF = 0 and standard payload length
                                    
                                    if (cir0_file != ""):
                                        # Last segment => append file
                                        with open(cir0_file, "ab") as data_file:
                                            data_file.write(uci_payload)
                                    
                                    cir0_file = ""
                                
                                if (bin_store and uci_hdr[0] == 0x7E and uci_hdr[1] == 0x85):
                                    # DBG_CIR1_LOG_NTF with PBF = 1 and extended payload length
                                    
                                    if (cir1_file == ""):
                                        # First segment => create file and write payload without Session ID
                                        
                                        cir1_file = "uwb_data_session_"
                                        cir1_file += format((uci_payload[3] << 24) + (uci_payload[2] << 16) + \
                                                            (uci_payload[1] << 8) + uci_payload[0])
                                        cir1_file += "_"
                                        cir1_file += format(meas_idx)
                                        cir1_file += "_CIR1.log"
                                        
                                        with open(cir1_file, "wb") as data_file:
                                            data_file.write(uci_payload[4:])
                                    
                                    else:
                                        # Second segment => append file
                                        with open(cir1_file, "ab") as data_file:
                                            data_file.write(uci_payload)
                                
                                if (bin_store and uci_hdr[0] == 0x6E and uci_hdr[1] == 0x05):
                                    # DBG_CIR1_LOG_NTF with PBF = 0 and standard payload length
                                    
                                    if (cir1_file != ""):
                                        # Last segment => append file
                                        with open(cir1_file, "ab") as data_file:
                                            data_file.write(uci_payload)
                                    
                                    cir1_file = ""
                                
                                if (uci_hdr[0] == 0x7E and uci_hdr[1] == 0x0B):
                                    # DBG_RFRAME_LOG_NTF with PBF = 1
                                    
                                    if (rframe_session == ""):
                                        # First segment
                                        rframe_session = format((uci_payload[3] << 24) + (uci_payload[2] << 16) + \
                                                                (uci_payload[1] << 8) + uci_payload[0])
                                        rframe_nb = uci_payload[4]
                                        rframe_meas = uci_payload[5:]
                                    
                                    else:
                                        # Second segment => append measurements
                                        rframe_meas += uci_payload
                                
                                if (uci_hdr[0] == 0x6E and uci_hdr[1] == 0x0B):
                                    # DBG_RFRAME_LOG_NTF with PBF = 0
                                    
                                    if (rframe_session == ""):
                                        # No segment
                                        rframe_session = format((uci_payload[3] << 24) + (uci_payload[2] << 16) + \
                                                                (uci_payload[1] << 8) + uci_payload[0])
                                        rframe_nb = uci_payload[4]
                                        rframe_meas = uci_payload[5:]
                                    
                                    else:
                                        # Last segment => append measurements
                                        rframe_meas += uci_payload
                                    
                                    if (bin_store):
                                        file_name = "uwb_data_session_"
                                        file_name += rframe_session
                                        file_name += "_"
                                        file_name += format(meas_idx)
                                        file_name += "_rframe.log"
                                        
                                        with open(file_name, "wb") as data_file:
                                            data_file.write(rframe_nb)
                                            data_file.write(rframe_meas)
                                    
                                    # Number of Rframe measurements
                                    cir_plot["nb_meas"] = rframe_nb
                                    
                                    # Delete previeous Rframe measurements
                                    cir_plot["mappings"] = []
                                    cir_plot["cir_samples"] = []
                                    
                                    idx = 0
                                    for rframe_meas_idx in range(0, rframe_nb):
                                        cir_plot["mappings"].append(rframe_meas[idx])
                                        idx += 27
                                        cir_plot["cir_samples"].append(extract_cir(rframe_meas[idx:idx + 64]))
                                        idx += 64
                                    
                                    rframe_session = ""
                                    rframe_nb = 0
                                    rframe_meas = []
                                
                                if (uci_hdr[0] == 0x62 and uci_hdr[1] == 0x00):
                                    # RANGE_DATA_NTF
                                    seq_cnt = extract_seq_cnt(uci_payload)
                                    
                                    # Check Status
                                    if (uci_payload[27] != 0x00 and uci_payload[27] != 0x1b):
                                        output("***** Ranging Error Detected ****")
                                        
                                        # Store range data for plot
                                        range_plot["index"] = seq_cnt
                                        range_plot["valid"] = False
                                        range_plot["nlos"] = 0
                                        range_plot["distance"] = 0
                                        range_plot["aoa1"] = 0
                                        range_plot["aoa2"] = 0
                                        range_plot["avg_aoa1"] = 0
                                        range_plot["avg_aoa2"] = 0
                                    else:
                                        if (bin_store):
                                            file_name = "uwb_data_session_"
                                            file_name += format((uci_payload[7] << 24) + (uci_payload[6] << 16) + \
                                                                (uci_payload[5] << 8) + uci_payload[4])
                                            file_name += "_"
                                            file_name += format(meas_idx)
                                            file_name += "_ntf.log"
                                            
                                            with open(file_name, "wb") as data_file:
                                                data_file.write(uci_payload)
                                        
                                        meas_nlos = (extract_nlos(uci_payload))
                                        meas_distance = (extract_distance(uci_payload))
                                        meas_azimuth = convert_qformat_to_float(extract_azimuth(uci_payload), 9, 7, 1)
                                        meas_azimuth_fom = (extract_azimuth_fom(uci_payload))
                                        meas_elevation = convert_qformat_to_float(extract_elevation(uci_payload), 9, 7, 1)
                                        meas_elevation_fom = (extract_elevation_fom(uci_payload))
                                        
                                        # added by maya, 20210618
                                        if (len(uci_payload) > 71):
                                            meas_pdoa1 = convert_qformat_to_float(extract_pdoa1(uci_payload), 9, 7, 7)
                                            meas_pdoa2 = convert_qformat_to_float(extract_pdoa2(uci_payload), 9, 7, 7)
                                        
                                        if (uci_payload[27] == 0x1b):   # negative distance
                                            meas_distance = -1 * meas_distance

                                        hist_distance.append(meas_distance)
                                        hist_azimuth.append(meas_azimuth)
                                        hist_elevation.append(meas_elevation)
                                        hist_pdoa1.append(meas_pdoa1)
                                        hist_pdoa2.append(meas_pdoa2)
                                        
                                        if (len(hist_distance) > avg_window_size): hist_distance.pop(0)
                                        if (len(hist_azimuth) > avg_window_size): hist_azimuth.pop(0)
                                        if (len(hist_elevation) > avg_window_size): hist_elevation.pop(0)
                                        if (len(hist_pdoa1) > avg_window_size): hist_pdoa1.pop(0)
                                        if (len(hist_pdoa2) > avg_window_size): hist_pdoa2.pop(0)
                                        
                                        avg_distance = sum(hist_distance) / len(hist_distance)
                                        avg_azimuth = sum(hist_azimuth) / len(hist_azimuth)
                                        avg_elevation = sum(hist_elevation) / len(hist_elevation)
                                        avg_pdoa1 = sum(hist_pdoa1) / len(hist_pdoa1)
                                        avg_pdoa2 = sum(hist_pdoa2) / len(hist_pdoa2)
                                        
                                        output("***(%d) NLos:%d   Dist:%d   Azimuth:%f (FOM:%d)   Elevation:%f (FOM:%d)  PDoA1:%f   PDoA2:%f" \
                                              % (seq_cnt, meas_nlos, meas_distance, meas_azimuth, meas_azimuth_fom, meas_elevation, meas_elevation_fom, meas_pdoa1, meas_pdoa2))
                                        output("*** Avg Dist:%d   Avg Azimuth:%f   Avg Elevation:%f   Avg_PDoA1:%f   Avg_PDoA2:%f" \
                                              % (avg_distance, avg_azimuth, avg_elevation, avg_pdoa1, avg_pdoa2))
                                        
                                        if((file_data_log is not None) and (not file_data_log.closed) and (file_data_log.writable())):
                                            string = datetime.now().isoformat(sep=" ", timespec="milliseconds")+",%d,%d,%d,%.1f,%d,%.1f,%d,%.1f,%.1f" % (seq_cnt, meas_nlos, meas_distance, meas_azimuth, meas_azimuth_fom, meas_elevation, meas_elevation_fom, meas_pdoa1, meas_pdoa2)
                                            file_data_log.write(string + "\n")
                                        
                                        # Store range data for plot
                                        range_plot["index"] = seq_cnt
                                        range_plot["valid"] = True
                                        range_plot["nlos"] = meas_nlos
                                        range_plot["distance"] = meas_distance
                                        range_plot["azimuth"] = meas_azimuth
                                        range_plot["elevation"] = meas_elevation
                                        range_plot["avg_azimuth"] = avg_azimuth
                                        range_plot["avg_elevation"] = avg_elevation
                                        
                                        if ((not is_ipc) or (is_stored)):
                                            # Increment the number of valid measurements
                                            meas_idx += 1
                                        
                                        if (nb_meas > 0 and meas_idx > nb_meas):
                                            if (is_ipc):
                                                file_ipc.close()
                                                
                                                # indicate to server the end of measurement set
                                                if(socket is not None):
                                                    try:
                                                        socket.send_string("ok")
                                                    except:
                                                        print("Fail to send OK on socket")
                                                
                                                # Restart new set of measures
                                                meas_idx = 1
                                            else:
                                                go_stop.set()
                            else:
                                output("\nExpected Payload bytes is " + str(count) + \
                                      ", Actual Paylod bytes received is " + str(len(uci_payload)))
                        else:
                            output("Port is not opened")
                    else:
                        output("\nUCI Payload Size is Zero")
                else:
                    output("\nUCI Header is not valid")
                
                write_wait.release()
            else:
                output("Port is not opened (2)")
        else:
            output("Port is not opened (1)")
    
    if serial_port.isOpen(): serial_port.close()
    
    output("Read from serial port exited")


def ipc_file_name():
    global stop_ipc_thread
    global file_ipc
    global socket
    global session_status
    global prefix_ipc
    
    output("IPC for output file name started")
    
    socket = zmq.Context().socket(zmq.PAIR)
    socket.connect("tcp://localhost:9999")

    while (not stop_ipc_thread):
        new_file_name = ""
        
        # Check if new file name is received
        # with a timeout of 1s to avoid endless blocking
        event = socket.poll(timeout=1000, flags=zmq.POLLIN)
        if event == zmq.POLLIN:
            new_file_name = socket.recv_string()
        
        if (new_file_name != ""):
            if (new_file_name == "STOP"):
                # Stop the processing loop
                session_status.set_all()
            else:
                # Close current output file if still open
                if ((file_ipc is not None) and (not file_ipc.closed)):
                    file_ipc.close()
                
                print("New file name: " + prefix_ipc + new_file_name)
                file_ipc = open(prefix_ipc + new_file_name, "w")
            
    # Close output file if still open
    if ((file_ipc is not None) and (not file_ipc.closed)):
        file_ipc.close()
    
    try:
        socket.send_string("closed")
    except:
        print("Fail to send closed on socket")
                                    
    output("IPC for output file name exited")


def serial_port_configure():
    global serial_port
    
    serial_port.baudrate = 3000000
    serial_port.timeout = 1  # To avoid endless blocking read
    serial_port.port = com_port
    
    if serial_port.isOpen(): serial_port.close()
    
    try:
        serial_port.open()
    except:
        output("#=> Fail to open " + com_port)
        sys.exit(1)


def start_processing():
    global stop_ipc_thread
    global stop_read_thread
    global stop_write_thread
    global session_status
    global is_range_plot
    global is_cir_plot
    global is_ipc
    global rhodes_role
    global range_plot
    global cir_plot
    global command_queue
    global go_stop
    
    # Initialize plots
    if (is_range_plot):
        plot_dist, plot_azimuth, plot_elevation, plot_cir = init_plots(rhodes_role)
    
    if (is_ipc):
        stop_ipc_thread = False
        read_thread = Thread(target=ipc_file_name, args = ())
        read_thread.start()
    
    stop_read_thread = False
    read_thread = Thread(target=read_from_serial_port, args=())
    read_thread.start()
    
    stop_write_thread = False
    write_thread = Thread(target=write_to_serial_port, args=())
    write_thread.start()
    
    handler = SIGINThandler()
    signal.signal(signal.SIGINT, handler.signal_handler)
    
    while (session_status.allow_end.is_set() == False):
        if handler.sigint:
            break
        
        if ((is_range_plot) and (plt.get_fignums())):  # Check if figure is still open
            if (range_plot["index"] >= 0):
                # Plot new range data
                draw_distance(plot_dist, range_plot)
                draw_aoa(plot_azimuth, range_plot["azimuth"], range_plot["avg_azimuth"])
                draw_aoa(plot_elevation, range_plot["elevation"], range_plot["avg_elevation"])
                
                # Disable range data
                range_plot["index"] = -1
            
            if ((is_cir_plot) and (cir_plot["nb_meas"] > 0)):
                # Plot new rframe data
                draw_cir(plot_cir, cir_plot)
                
                # Disable rframe data
                cir_plot["nb_meas"] = 0
            
            # Update figure
            plt.draw()
            plt.pause(0.001)
    
    # To restore output on STDOUT
    is_ipc = False
    
    if(data_log):
        file_data_log.close()
        
    # End of processing
    stop_write_thread = True
    stop_read_thread = True
    stop_ipc_thread = True
    
    # Unblock the waiting in the write thread
    command_queue.put([0xFF, 0xFF])  # End of write
    session_status.set_all()
    go_stop.set()
    
    # Close figure
    if (is_range_plot):
        plt.close('all')
        range_plot["index"] = -1


def main():
    global nb_meas
    global rhodes_role
    global com_port
    global is_timestamp
    global is_range_plot
    global is_cir_plot
    global bin_store
    global is_ipc
    global prefix_ipc
    global file_ipc
    global file_data_log
    global command_queue
    global channel_ID
    global readOTP
    global power_offset
    
    path = ""
    
    for arg in sys.argv[1:]:
        if (arg.isdecimal()):
            nb_meas = int(arg)
        elif (arg == "i"):
            rhodes_role = "Initiator"
        elif (arg == "r"):
            rhodes_role = "Responder"
        elif (arg.startswith("COM")):
            com_port = arg
        elif (arg == "notime"):
            is_timestamp = False
        elif (arg == "noplot"):
            is_range_plot = False
            is_cir_plot = False
        elif (arg == "nocirplot"):
            is_range_plot = True
            is_cir_plot = False
        elif (arg == "ipc"):
            is_ipc = True
        elif (arg.startswith("OFFSET=")):
            power_offset = int(re.sub(r"\D", "", arg))
        else:
            path = arg
    
    is_range_plot = False
    is_cir_plot = False
    
    if (is_ipc):
        # Disable timestamp, plots and binary storage
        is_timestamp = False
        is_range_plot = False
        is_cir_plot = False
        bin_store = False
        
        # Fix default number of measurements if not set in argument
        if (nb_meas == 0):
            nb_meas = 10
        
        if (path == ""):
            data_path = os.getcwd()
            prefix_ipc = "ipc_"
            
        else:
            slash_idx = path.rfind("/")
            backslash_idx = path.rfind("\\")

            if ((slash_idx >= 0) or (backslash_idx >= 0)):
                # Split path and prefix
                prefix_ipc = path[max(slash_idx, backslash_idx) + 1:]
                data_path = os.getcwd() + "/" + path[:max(slash_idx, backslash_idx)]
            else:
                prefix_ipc = path
                data_path = os.getcwd()

            if (prefix_ipc[-1] != "_"):
                # Add underscore at end of prefix
                prefix_ipc = prefix_ipc + "_"
            
            try:
                os.stat(data_path)
            except:
                os.makedirs(data_path)
        
        # Open default output file
        os.chdir(data_path)
        print("Working directory: " + data_path)
        
        new_file_name = "_init.txt"
        print("New file name: " + prefix_ipc + new_file_name)
        file_ipc = open(prefix_ipc + new_file_name, "w")
    
    else:
        if (path != ""):
            bin_store = True
            
            if (is_timestamp):
                data_path = os.getcwd() + "/" + rhodes_role + "_" + datetime.now().strftime("%y%m%d_%H%M%S") + \
                            "/" + path
            else:
                data_path = os.getcwd() + "/" + rhodes_role + "/" + path
            
            try:
                os.stat(data_path)
                output("Data directory" + data_path + "already exist")
            except:
                os.makedirs(data_path)
                output("Data directory" + data_path + "created")
            
            os.chdir(data_path)
        else:
            bin_store = False
    
    if(data_log):
        # Open default output file
        data_path = os.getcwd()
        os.chdir(data_path)
        print("Working directory: " + data_path)
        
        new_file_name = "log_" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
        print("New file name: " + new_file_name)
        file_data_log = open(new_file_name, "w")
        file_data_log.write("time, seq_cnt, nlos, distance, azimuth, azimuth_fom, elevation, elevation_fom, pdoa1, pdoa2\n")
    
    
    output("Role:" + rhodes_role + "   Port:" + com_port + "   Nb Meas:" + str(nb_meas) + "   Timestamp:" + str(is_timestamp) + \
          "   Range Plot:" + str(is_range_plot) + "   CIR Plot:" + str(is_cir_plot) + "   IPC:" + str(is_ipc))
    
    output("Configure serial port...")
    serial_port_configure()
    output("Serial port configured")
    
    # Add the UCI Commands to sent
    output("Start adding commands to the queue...")
    
    command_queue.put(UWB_SET_BOARD_VARIANT)
    command_queue.put(UWB_RESET_DEVICE)
    
    command_queue.put([0x20, 0x02, 0x00, 0x00])  # Get Device Information
    command_queue.put([0x20, 0x03, 0x00, 0x00])  # Get Device Capability
    
    if(readOTP == True):
        command_queue.put(UWB_EXT_READ_CALIB_DATA_XTAL_CAP)
        command_queue.put(UWB_EXT_READ_CALIB_DATA_TX_POWER)
    
    command_queue.put(UWB_CORE_SET_CONFIG)
    command_queue.put(UWB_CORE_SET_ANTENNA_RX_IDX_DEFINE)
    command_queue.put(UWB_CORE_SET_ANTENNA_TX_IDX_DEFINE)
    command_queue.put(UWB_CORE_SET_ANTENNAS_RX_PAIR_DEFINE)
    
    command_queue.put(UWB_SET_CFO_CALIBRATION)
    command_queue.put(UWB_SET_POWER_CALIBRATION)
    
    if (channel_ID[0] == 0x05):
        command_queue.put(UWB_CORE_SET_RX_ANT_DELAY_CALIB_CH5)
        command_queue.put(UWB_CORE_SET_PDOA_OFFSET_CALIB_CH5)
        command_queue.put(UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR1_CH5)
        command_queue.put(UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR2_CH5)
        command_queue.put(UWB_CORE_SET_AOA_THRESHOLD_PDOA_CH5)
        command_queue.put(UWB_SET_CALIBRATION_PDOA_MANUFACT_ZERO_OFFSET_CALIB_CH5)
        #command_queue.put(UWB_SET_CALIBRATION_PDOA_MULTIPOINT_CALIB_CH5)
    if (channel_ID[0] == 0x09):
        command_queue.put(UWB_CORE_SET_RX_ANT_DELAY_CALIB_CH9)
        command_queue.put(UWB_CORE_SET_PDOA_OFFSET_CALIB_CH9)
        command_queue.put(UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR1_CH9)
        command_queue.put(UWB_CORE_SET_AOA_ANTENNAS_PDOA_CALIB_PAIR2_CH9)
        command_queue.put(UWB_CORE_SET_AOA_THRESHOLD_PDOA_CH9)
        command_queue.put(UWB_SET_CALIBRATION_PDOA_MANUFACT_ZERO_OFFSET_CALIB_CH9)
        #command_queue.put(UWB_SET_CALIBRATION_PDOA_MULTIPOINT_CALIB_CH9)
        
    command_queue.put(UWB_SESSION_INIT_RANGING)
    command_queue.put(UWB_SESSION_SET_APP_CONFIG)
    command_queue.put(UWB_SESSION_SET_APP_CONFIG_NXP)
    if (rhodes_role == "Initiator"): command_queue.put(UWB_SESSION_SET_INITIATOR_CONFIG)
    if (rhodes_role == "Responder"): command_queue.put(UWB_SESSION_SET_RESPONDER_CONFIG)
    #command_queue.put(UWB_SESSION_SET_DEBUG_CONFIG)
    
    command_queue.put(UWB_RANGE_START)
    if (nb_meas > 0):
        command_queue.put(UWB_RANGE_STOP)
        command_queue.put(UWB_SESSION_DEINIT)
    output("adding commands to the queue completed")
    
    output("Start processing...")
    start_processing()
    output("Processing finished")


if __name__ == "__main__":
    main()
