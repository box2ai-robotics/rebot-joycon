"""Select the arm serial port without local calibration files."""
import os

from serial.tools import list_ports


def select_port():
    if os.environ.get('REBOT_PORT'):
        return os.environ['REBOT_PORT']
    ports = [p.device for p in list_ports.comports()
             if p.device.startswith(('/dev/ttyACM', '/dev/ttyUSB'))]
    if len(ports) != 1:
        raise RuntimeError('未找到唯一机械臂串口；请检查连接，或用 '
                           'REBOT_PORT=/dev/ttyACM0 指定（不需要本地校准文件）')
    return ports[0]
