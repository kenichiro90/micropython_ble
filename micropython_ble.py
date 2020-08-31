from ubluetooth import BLE, UUID, FLAG_NOTIFY, FLAG_READ, FLAG_WRITE
from micropython import const
import utime


class ScanBle():
    def __init__(self):
        self.bt = BLE()
        self.bt.active(True)
        self.bt.irq(handler=self.bt_irq)
        self._IRQ_SCAN_RESULT = const(1 << 4)
        self._IRQ_SCAN_COMPLETE = const(1 << 5)
        self._li_addr = []
        self._UTC_OFFSET = 9
        self.num_ble = 0

    def bt_irq(self, event, data):
        if event == self._IRQ_SCAN_RESULT:
            # A single scan result.
            # addr_type, addr, connectable, rssi, adv_data = data
            _, addr, _, _, _ = data
            self._li_addr.append(':'.join(['{:02X}'.format(addr[_]) for _ in range(len(addr))]))

    def scan(self, duration_ms, interval_ms, window_ms):
        self._li_addr = []
        self.bt.gap_scan(duration_ms, interval_ms * 1000, window_ms * 1000)
        utime.sleep_ms(duration_ms)
        self._li_addr = list(set(self._li_addr))

        for i in range(len(self._li_addr)):
            _date = utime.localtime(utime.mktime(utime.localtime()) + self._UTC_OFFSET * 3600)
            _date = '{:04d}'.format(_date[0]) + '-' + '{:02d}'.format(_date[1]) + '-' + '{:02d}'.format(_date[2]) + \
                        ' ' + '{:02d}'.format(_date[3]) + ':' + '{:02d}'.format(_date[4]) + ':' + '{:02d}'.format(_date[5])
            print("Mode:ble, date:{} address:{}".format(_date, self._li_addr[i]))
        self.ble_num = len(self._li_addr)
        print("ble_num: {}\n".format(self.ble_num))
        del self._li_addr
        del self.ble_num


def main():
    # Get BLE Data
    sb = ScanBle()

    while True:
        try:
            sb.scan(duration_ms=5000, interval_ms=1000, window_ms=1000)
            utime.sleep_ms(5000)
            gc.collect()
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    main()
