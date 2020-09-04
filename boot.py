from neopixel import NeoPixel
from machine import Pin, I2C
import network
# import webrepl
import utime
import ntptime

# Set Local TimeZone
UTC_OFFSET = 9

LED_GPIO = const(27)
matrix_size_x = const(5)
matrix_size_y = const(5)
np = NeoPixel(Pin(LED_GPIO), matrix_size_x * matrix_size_y)

def turn_on_led(color_R, color_G, color_B):
    for i in range(25):
        np[i] = (color_R, color_G, color_B)
    np.write()

def ntpset_main():
    # Connect to NTP Server
    utime.sleep_ms(1000)
    ntptime.settime()

    _date = utime.localtime(utime.mktime(utime.localtime()) + UTC_OFFSET * 3600)
    _date = '{:04d}'.format(_date[0]) + '-' + '{:02d}'.format(_date[1]) + '-' + '{:02d}'.format(_date[2]) + \
                            ' ' + '{:02d}'.format(_date[3]) + ':' + '{:02d}'.format(_date[4]) + ':' + '{:02d}'.format(_date[5])
    print(_date + '\n')

turn_on_led(0, 0, 0)
sta = network.WLAN(network.STA_IF)
sta.active(True)

# sta.connect("HUMAX-E7D7F", "XWN5dTNJMWNJN")
sta.connect("SH-03J_AP", "dc1895cbcb06")
start_time = utime.time()

while utime.time() - start_time < 10:
    if sta.isconnected():
        turn_on_led(0, 0, 20)
        print("Connected!")
        # webrepl.start()
        ntpset_main()
        break
else:
    turn_on_led(20, 0, 0)
    print("Timeout")
    sta.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    # webrepl.start()

utime.sleep_ms(3000)
turn_on_led(0, 0, 0)
exec(open('micropython_ble.py').read())
# exec(open('main.py').read())