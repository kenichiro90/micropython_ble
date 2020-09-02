import gc
from ubluetooth import BLE, UUID, FLAG_NOTIFY, FLAG_READ, FLAG_WRITE
from micropython import const
import utime

from umqtt.simple import MQTTClient

from mpu6886 import MPU6886
from neopixel import NeoPixel
from machine import Pin, I2C

# Set MQTT Broker Settings
SERVER = "mqtt.thingspeak.com"
CHANNEL_ID = "1131265"
WRITE_API_KEY = "S251B8I21QXCD7UB"

# Set MQTT Client Settings
client = MQTTClient("umqtt_client", SERVER)
topic = "channels/" + CHANNEL_ID + "/publish/" + WRITE_API_KEY


class ScanBle():
    def __init__(self):
        # Initialize BLE Device
        self.bt = BLE()
        self.bt.active(True)
        self.bt.irq(handler=self.bt_irq)
        self._IRQ_SCAN_RESULT = const(1 << 4)
        self._IRQ_SCAN_COMPLETE = const(1 << 5)
        
        # Initialize List
        self._li_addr = []

        # Set Const. Values
        self._UTC_OFFSET = 9
        self.ble_num = 0

    def bt_irq(self, event, data):
        if event == self._IRQ_SCAN_RESULT:
            # A single scan result.
            # addr_type, addr, connectable, rssi, adv_data = data
            _, addr, _, _, _ = data
            self._li_addr.append(':'.join(['{:02X}'.format(addr[_]) for _ in range(len(addr))]))

    def scan(self, duration_ms, interval_ms, window_ms):
        # Scan BLE Devices
        self._li_addr = []
        self.bt.gap_scan(duration_ms, interval_ms * 1000, window_ms * 1000)
        utime.sleep_ms(duration_ms)
        self._li_addr = list(set(self._li_addr))

        # Parse Scanned Data
        for i in range(len(self._li_addr)):
            _date = utime.localtime(utime.mktime(utime.localtime()) + self._UTC_OFFSET * 3600)
            _date = '{:04d}'.format(_date[0]) + '-' + '{:02d}'.format(_date[1]) + '-' + '{:02d}'.format(_date[2]) + \
                        ' ' + '{:02d}'.format(_date[3]) + ':' + '{:02d}'.format(_date[4]) + ':' + '{:02d}'.format(_date[5])
            print("Mode:ble, date:{} address:{}".format(_date, self._li_addr[i]))
        
        # Get Num of BLE Devices
        self.ble_num = len(self._li_addr)
        
        # Delete Class Variables
        print("ble_num: {}\n".format(self.ble_num))
        return self.ble_num
        # del self._li_addr
        # del self.ble_num


class AcclData():
    def __init__(self):
        # Definitions for the ATOM Matrix
        self.MPU6886_SCL = const(21)
        self.MPU6886_SDA = const(25)
        self.LED_GPIO = const(27)
        self.PUSH_BTN = const(39)
        self.matrix_size_x = const(5)
        self.matrix_size_y = const(5)
        self.color = (0, 0, 20) # Initial color: Blue
        self.np = NeoPixel(Pin(self.LED_GPIO), self.matrix_size_x * self.matrix_size_y)
        self.avg_gx, self.avg_gy, self.avg_gz = 0, 0, 0
        self.accl_diff = 0

        self.initialize_device()

    def calibrateGyro(self, num):
        for _ in range(0, num):
            gx, gy, gz = self.imu.getGyroData()
            self.avg_gx += gx
            self.avg_gy += gy
            self.avg_gz += gz
            utime.sleep_ms(50)
        self.avg_gx /= num
        self.avg_gy /= num
        self.avg_gz /= num

    def initialize_device(self):
        # I2C bus init for ATOM Matrix MPU6886
        i2c = I2C(scl=Pin(self.MPU6886_SCL), sda=Pin(self.MPU6886_SDA))

        # Values you can use to initialize the accelerometer. AFS_16G, means +-8G sensitivity, and so on
        # Larger scale means less precision
        AFS_2G      = const(0x00)
        AFS_4G      = const(0x01)
        AFS_8G      = const(0x02)
        AFS_16G     = const(0x03)

        # Values you can use to initialize the gyroscope. GFS_2000DPS means 2000 degrees per second sensitivity, and so on
        # Larger scale means less precision
        GFS_250DPS  = const(0x00)
        GFS_500DPS  = const(0x01)
        GFS_1000DPS = const(0x02)
        GFS_2000DPS = const(0x03)  

        # by default, if you initialize MPU6886 with  imu = MPU6886(i2c), GFS_2000DPS and AFS_8G are used
        # if you want to initialize with other values you have too use :
        # imu = MPU6886(i2c,mpu6886.GFS_250DPS,mpu6886.AFS_4G )
        # imu = MPU6886(i2c) #=> use default 8G / 2000DPS
        self.imu = MPU6886(i2c, GFS_500DPS, AFS_4G)

        # in order to calibrate Gyroscope you have to put the device on a flat surface
        # preferably level with the floor and not touch it during the procedure. (1s for 20 cycles)
        self.calibrateGyro(20)

    def computeAngles(self, ax,ay,az):
        pitch = 180 * atan (ax / sqrt(ay ** 2 + az ** 2)) / pi
        roll = 180 * atan (ay / sqrt(ax ** 2 + az ** 2)) / pi
        yaw = 180 * atan (az / sqrt(ax ** 2 + ay ** 2)) / pi
        return pitch, roll, yaw

    def get_accl_diff(self):
        # Get Acceleration Data
        old_ax, old_ay, old_az = self.imu.getAccelData()
        utime.sleep_ms(500)
        ax, ay, az = self.imu.getAccelData()

        # gx, gy, gz = ad.imu.getGyroData()
        # pitch, roll, yaw = ad.computeAngles(ax, ay, az)

        # Calc Differential of Acceleration
        self.accl_diff = ((ax - old_ax) ** 2) + ((ay - old_ay) ** 2) + ((az - old_az) ** 2)
        self.accl_diff = self.accl_diff ** (1 / 2)

        print("old_ax:{}, old_ay:{}, old_az:{}".format(old_ax, old_ay, old_az))
        print("ax:{}, ay:{}, az:{}".format(ax, ay, az))

        # Send Data to MQTT Broker
        # payload = "field2=" + str(self.accl_diff)
        # self.client.connect()
        # self.client.publish(self.topic, payload)
        # self.client.disconnect()
        print("accl_diff:{}\n".format(self.accl_diff))
        return self.accl_diff

        # del self.accl_diff
        # print("gx:{}, gy:{}, gz:{}".format(gx, gy, gz))
        # print("pitch:{}, roll:{}, yaw:{}".format(pitch, roll, yaw))


def main():
    # Initialize Class
    sb = ScanBle()
    ad = AcclData()
    while True:
        try:
            # Get Accl_Diff Data
            accl_diff = ad.get_accl_diff()
            utime.sleep_ms(9500)

            # # Scan BLE Devices
            ble_num = sb.scan(duration_ms=5000, interval_ms=1000, window_ms=1000)
            utime.sleep_ms(10000)

            # Send Data to MQTT Broker
            payload = "field1=" + str(ble_num) + "&field2=" + str(accl_diff)
            client.connect()
            client.publish(topic, payload)
            client.disconnect()
        
            # Collect Garbages
            gc.collect()
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    main()
