import network
# import webrepl
import utime
import ntptime

# Set Local TimeZone
UTC_OFFSET = 9

def ntpset_main():
    # Connect to NTP Server
    utime.sleep_ms(1000)
    ntptime.settime()

    _date = utime.localtime(utime.mktime(utime.localtime()) + UTC_OFFSET * 3600)
    _date = '{:04d}'.format(_date[0]) + '-' + '{:02d}'.format(_date[1]) + '-' + '{:02d}'.format(_date[2]) + \
                            ' ' + '{:02d}'.format(_date[3]) + ':' + '{:02d}'.format(_date[4]) + ':' + '{:02d}'.format(_date[5])
    print(_date + '\n')

sta = network.WLAN(network.STA_IF)
sta.active(True)

sta.connect("HUMAX-E7D7F", "XWN5dTNJMWNJN")
start_time = utime.time()

while utime.time() - start_time < 10:
    if sta.isconnected():
        print("Connected!")
        # webrepl.start()
        ntpset_main()
        break
else:
    print("Timeout")
    sta.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    # webrepl.start()

utime.sleep_ms(5000)
exec(open('micropython_ble.py').read())
# exec(open('main.py').read())