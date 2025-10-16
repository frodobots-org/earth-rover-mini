from ctypes import Structure, c_uint8, c_uint16, c_int16, c_uint32


# =========================================================================
# Command Identifiers
# =========================================================================
UCP_KEEP_ALIVE           = 0x1
UCP_MOTOR_CTL            = 0x2
UCP_IMU_CORRECTION_START = 0x3
UCP_IMU_CORRECTION_END   = 0x4
UCP_RPM_REPORT           = 0x5
UCP_IMU_WRITE            = 0x6
UCP_MAG_WRITE            = 0x7
UCP_IMUMAG_READ          = 0x8
UCP_OTA                  = 0x9
UCP_STATE                = 0xA


# =========================================================================
# Error Codes
# =========================================================================
class UcpErr:
    UCP_ERR_OK = 0
    UCP_ERR_UNKNOWN = 1
    UCP_ERR_OTA_FORBIDDEN = 2
    UCP_ERR_MAX = 3


# =========================================================================
# IMU Correction Types
# =========================================================================
class UcpImuCorrectionType:
    UICT_MAG = 1
    UICT_IMU = 2


# =========================================================================
# Message Structures
# =========================================================================

class UcpHd(Structure):
    _pack_ = 1
    _fields_ = [
        ("len",   c_uint16),
        ("id",    c_uint8),
        ("index", c_uint8),
    ]


class UcpAlivePing(Structure):
    _pack_ = 1
    _fields_ = [("hd", UcpHd)]


class UcpAlivePong(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",  UcpHd),
        ("err", c_uint8),
    ]


class UcpCtlCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",        UcpHd),
        ("speed",     c_int16),
        ("angular",   c_int16),
        ("front_led", c_int16),
        ("back_led",  c_int16),
        ("version",   c_uint16),
        ("reserve1",  c_uint16),
        ("reserve2",  c_uint32),
    ]


class UcpImuCorrect(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",   UcpHd),
        ("type", c_uint8),
    ]


class UcpImuCorrectAck(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",   UcpHd),
        ("type", c_uint8),
        ("err",  c_uint8),
    ]


class UcpRep(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",          UcpHd),
        ("voltage",     c_uint16),
        ("rpm",         c_int16 * 4),
        ("acc",         c_int16 * 3),
        ("gyros",       c_int16 * 3),
        ("mag",         c_int16 * 3),
        ("heading",     c_int16),
        ("stop_switch", c_uint8),
        ("error_code",  c_uint8),
        ("reserve",     c_uint16),
        ("version",     c_uint16),
    ]


class UcpMagW(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",         UcpHd),
        ("mag_bias_x", c_uint16),
        ("mag_bias_y", c_uint16),
        ("mag_bias_z", c_uint16),
    ]


class UcpMagWAck(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",  UcpHd),
        ("err", c_uint8),
    ]


class UcpImuW(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",          UcpHd),
        ("acc_bias_x",  c_uint16),
        ("acc_bias_y",  c_uint16),
        ("acc_bias_z",  c_uint16),
        ("gyro_bias_x", c_uint16),
        ("gyro_bias_y", c_uint16),
        ("gyro_bias_z", c_uint16),
    ]


class UcpImuWAck(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",  UcpHd),
        ("err", c_uint8),
    ]


class UcpImuR(Structure):
    _pack_ = 1
    _fields_ = [("hd", UcpHd)]


class UcpImuRAck(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",        UcpHd),
        ("err",       c_uint8),
        ("acc_bias_x",  c_uint16),
        ("acc_bias_y",  c_uint16),
        ("acc_bias_z",  c_uint16),
        ("gyro_bias_x", c_uint16),
        ("gyro_bias_y", c_uint16),
        ("gyro_bias_z", c_uint16),
        ("mag_bias_x",  c_uint16),
        ("mag_bias_y",  c_uint16),
        ("mag_bias_z",  c_uint16),
    ]


class UcpOta(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",      UcpHd),
        ("version", c_int16),
    ]


class UcpOtaAck(Structure):
    _pack_ = 1
    _fields_ = [
        ("hd",  UcpHd),
        ("err", c_uint8),
    ]


class UcpState:
    UCP_STATE_UNKNOWN = 0
    UCP_STATE_SIMABSENT = 1
    UCP_NETWORK_DISCONNECTED = 2
    UCP_NETWORK_CONNECTED = 3
    UCP_OTA_ING = 4
