from ctypes import *


VCI_USBCAN2 = 4
STATUS_OK = 1

# 定义VCI_INIT_CONFIG结构体
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte),
    ]

# 定义VCI_CAN_OBJ结构体
class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]

# 定义VCI_CAN_OBJ_ARRAY结构体
class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [ ("SIZE", c_uint16),("STRUCT_ARRAY", POINTER(VCI_CAN_OBJ)),]
    def __init__(self, num_of_structs):
        self.STRUCT_ARRAY = cast((VCI_CAN_OBJ * num_of_structs)(), POINTER(VCI_CAN_OBJ))
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]

# 加载CAN库文件
def load_can_library(library_path):
    return cdll.LoadLibrary(library_path)

CanDLLName = "/home/shansu/catkin_ws/src/wheels_controller/libcontrolcan.so"
canDLL = load_can_library(CanDLLName)
vci_initconfig1 = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x1C, 0)
vci_initconfig2 = VCI_INIT_CONFIG(0x00000000, 0xFFFFFFFF, 0, 0, 0x00, 0x1C, 0)

# 打开设备
def open_device(device_index, reserved):
    res = canDLL.VCI_OpenDevice(VCI_USBCAN2, device_index, reserved)
    return res

# 初始化CAN通道
def init_can(device_index, can_index, config):
    res = canDLL.VCI_InitCAN(VCI_USBCAN2, device_index, can_index, byref(config))
    return res

# 启动CAN通道
def start_can(device_index, can_index):
    res = canDLL.VCI_StartCAN(VCI_USBCAN2, device_index, can_index)
    return res

# 发送数据
# def transmit_data(device_index, can_index, can_obj):
#     res = canDLL.VCI_Transmit(VCI_USBCAN2, device_index, can_index, byref(can_obj), 1)
#     return res

# 发送数据
def transmit_data(device_index, can_index, node_id, data):
    b = (c_ubyte * 3)(0, 0, 0)
    vci_can_obj = VCI_CAN_OBJ(node_id, 0, 0, 1, 0, 0, 8, data, b)
    res = canDLL.VCI_Transmit(VCI_USBCAN2, device_index, can_index, byref(vci_can_obj), 1)
    return res


rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2500)
# 接收数据
def receive_data(device_index, can_index, obj_array, size, wait_time):
    res = canDLL.VCI_Receive(VCI_USBCAN2, device_index, can_index, byref(obj_array.ADDR), size, wait_time)
    if res > 0:
        for i in range(res):
            print(f"CAN{can_index + 1} ", end=" ")
            print("ID:", end="")
            print(hex(obj_array.STRUCT_ARRAY[i].ID), end=" ")
            print("DataLen:", end="")
            print(hex(obj_array.STRUCT_ARRAY[i].DataLen), end=" ")
            print("Data:", end="")
            print(list(obj_array.STRUCT_ARRAY[i].Data), end=" ")
            print("\\r")
    return res

# 关闭设备
def close_device(device_index):
    res = canDLL.VCI_CloseDevice(VCI_USBCAN2, device_index)
    return res
