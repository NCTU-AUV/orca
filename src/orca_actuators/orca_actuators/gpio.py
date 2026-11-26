"""Jetson 40-pin 的一支 GPIO 輸出：Linux GPIO character device（/dev/gpiochipN，v1 uAPI）。

不 import ROS、不需要 libgpiod 或 Jetson.GPIO，主機上直接跑（tools/hw_tests/magnet.py）
或在容器裡跑（ball_dropper 節點）都一樣。要能讀寫 /dev/gpiochip*（主機上是 gpio 群組）。

- 腳位用 40-pin 的實體腳號（HEADER），換算成 tegra234 的 GPIO 名稱（例如 29 → PQ.05），
  再到 label 為 tegra234-gpio 的 gpiochip 用名稱找 line（不寫死 line 編號）。
- 開啟（request）之後由這個 process 佔住這支腳：其他程式（gpioset、另一個節點）再開會失敗
  （EBUSY）。close() 或 process 結束時放掉；放掉之後腳位維持什麼電位由核心和 pinmux 決定，
  沒有保證，要實測（tools/hw_tests/magnet.py 的 exit 指令）。
- active_high：邏輯「on」對應的電位。True = on 輸出 HIGH（3.3 V）。
- 輸出電位要真的出現在腳上，pinmux 要把那支腳設成 GPIO（JetPack 6 有些腳預設不是）。
  這裡讀回來的值是「要求輸出的值」，不是腳上量到的電壓；第一次一定要用電錶確認。
"""
import fcntl
import glob
import os
import struct


def _ioc(direction, nr, size):
    return (direction << 30) | (size << 16) | (0xB4 << 8) | nr


_R, _RW = 2, 3
_CHIPINFO = struct.Struct('=32s32sI')                  # struct gpiochip_info
_LINEINFO = struct.Struct('=II32s32s')                 # struct gpioline_info
_HANDLE_REQ = struct.Struct('=64II64B32sIi')           # struct gpiohandle_request
_HANDLE_DATA = struct.Struct('=64B')                   # struct gpiohandle_data
GPIO_GET_CHIPINFO_IOCTL = _ioc(_R, 0x01, _CHIPINFO.size)
GPIO_GET_LINEINFO_IOCTL = _ioc(_RW, 0x02, _LINEINFO.size)
GPIO_GET_LINEHANDLE_IOCTL = _ioc(_RW, 0x03, _HANDLE_REQ.size)
GPIOHANDLE_GET_LINE_VALUES_IOCTL = _ioc(_RW, 0x08, _HANDLE_DATA.size)
GPIOHANDLE_SET_LINE_VALUES_IOCTL = _ioc(_RW, 0x09, _HANDLE_DATA.size)

GPIOLINE_FLAG_KERNEL = 1 << 0
GPIOLINE_FLAG_IS_OUT = 1 << 1
GPIOLINE_FLAG_ACTIVE_LOW = 1 << 2
GPIOHANDLE_REQUEST_OUTPUT = 1 << 1

CHIP_LABEL = 'tegra234-gpio'

# Jetson Orin NX／Orin Nano 40-pin（reComputer J401 照開發套件的腳位）：實體腳號 → GPIO 名稱、別的功能。
# 來源：Jetson.GPIO 2.1.7 的 JETSON_ORIN_NX 對照表（2026-11-26 在 orca 上讀出來的）。
HEADER = {
    7: ('PAC.06', 'AUD_MCLK'),
    11: ('PR.04', 'UART1_RTS'),
    12: ('PH.07', 'I2S0_SCLK'),
    13: ('PY.00', 'SPI1_SCK'),
    15: ('PN.01', 'GPIO12／PWM'),
    16: ('PY.04', 'SPI1_CS1'),
    18: ('PY.03', 'SPI1_CS0'),
    19: ('PZ.05', 'SPI0_MOSI'),
    21: ('PZ.04', 'SPI0_MISO'),
    22: ('PY.01', 'SPI1_MISO'),
    23: ('PZ.03', 'SPI0_SCK'),
    24: ('PZ.06', 'SPI0_CS0'),
    26: ('PZ.07', 'SPI0_CS1'),
    29: ('PQ.05', 'GPIO01'),
    31: ('PQ.06', 'GPIO11'),
    32: ('PG.06', 'GPIO07／PWM'),
    33: ('PH.00', 'GPIO13／PWM'),
    35: ('PI.02', 'I2S0_FS'),
    36: ('PR.05', 'UART1_CTS'),
    37: ('PY.02', 'SPI1_MOSI'),
    38: ('PI.01', 'I2S0_DIN'),
    40: ('PI.00', 'I2S0_DOUT'),
}


class GpioError(RuntimeError):
    pass


def _cstr(b):
    return b.split(b'\0', 1)[0].decode(errors='replace')


def find_chip(label=CHIP_LABEL):
    """label 相符的 /dev/gpiochipN 的路徑。"""
    found = []
    for path in sorted(glob.glob('/dev/gpiochip*')):
        try:
            fd = os.open(path, os.O_RDONLY)
        except OSError as e:
            raise GpioError(f'開不了 {path}：{e}（主機上要在 gpio 群組；容器要掛 /dev/gpiochip*）') from e
        try:
            buf = bytearray(_CHIPINFO.size)
            fcntl.ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, buf, True)
            _, chip_label, _ = _CHIPINFO.unpack(buf)
        finally:
            os.close(fd)
        found.append(_cstr(chip_label))
        if _cstr(chip_label) == label:
            return path
    raise GpioError(f'找不到 label 為 {label} 的 gpiochip（有：{found or "沒有 /dev/gpiochip*"}）')


def line_info(chip_fd, offset):
    """(名稱, 使用者, 是否輸出, 是否被佔用)。"""
    buf = bytearray(_LINEINFO.pack(offset, 0, b'', b''))
    fcntl.ioctl(chip_fd, GPIO_GET_LINEINFO_IOCTL, buf, True)
    _, flags, name, consumer = _LINEINFO.unpack(buf)
    return _cstr(name), _cstr(consumer), bool(flags & GPIOLINE_FLAG_IS_OUT), bool(flags & GPIOLINE_FLAG_KERNEL)


def find_line(chip_fd, name):
    buf = bytearray(_CHIPINFO.size)
    fcntl.ioctl(chip_fd, GPIO_GET_CHIPINFO_IOCTL, buf, True)
    lines = _CHIPINFO.unpack(buf)[2]
    for offset in range(lines):
        if line_info(chip_fd, offset)[0] == name:
            return offset
    raise GpioError(f'gpiochip 裡沒有叫 {name} 的 line')


def header_pin(pin):
    if pin not in HEADER:
        raise GpioError(f'40-pin 的第 {pin} 腳不能當 GPIO（可用：{sorted(HEADER)}）')
    return HEADER[pin][0]


def describe(pin):
    """這支腳目前的狀態（不佔用），給人看的一行字。"""
    name = header_pin(pin)
    fd = os.open(find_chip(), os.O_RDONLY)
    try:
        offset = find_line(fd, name)
        _, consumer, is_out, used = line_info(fd, offset)
    finally:
        os.close(fd)
    who = f'被「{consumer or "?"}」佔用' if used else '沒人佔用'
    return f'pin {pin} = {name}（line {offset}）：{"輸出" if is_out else "輸入"}，{who}'


class OutputPin:
    """佔住 40-pin 的一支腳當輸出。on(True／False) 設邏輯值；active_high 決定邏輯 on 是 HIGH 還是 LOW。"""

    def __init__(self, pin, active_high=True, initial=False, consumer='orca'):
        self.pin, self.name, self.active_high = pin, header_pin(pin), active_high
        chip = os.open(find_chip(), os.O_RDONLY)
        try:
            self.offset = find_line(chip, self.name)
            req = bytearray(_HANDLE_REQ.pack(
                *([self.offset] + [0] * 63), GPIOHANDLE_REQUEST_OUTPUT,
                *([self._level(initial)] + [0] * 63), consumer.encode()[:31], 1, 0))
            try:
                fcntl.ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, req, True)
            except OSError as e:
                _, who, _, _ = line_info(chip, self.offset)
                hint = f'（被「{who}」佔用：先停掉它）' if who else ''
                raise GpioError(f'拿不到 pin {pin}（{self.name}）：{e}{hint}') from e
            self.fd = _HANDLE_REQ.unpack(req)[-1]
        finally:
            os.close(chip)
        self.value = initial

    def _level(self, on):
        return 1 if bool(on) == self.active_high else 0

    def set(self, on):
        if self.fd is None:
            raise GpioError('腳位已經放掉了')
        data = bytearray(_HANDLE_DATA.pack(self._level(on), *([0] * 63)))
        fcntl.ioctl(self.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, data, True)
        self.value = bool(on)

    def level(self):
        """核心回報的電位（0／1）。輸出腳讀到的是要求輸出的值，不等於量到的電壓。"""
        data = bytearray(_HANDLE_DATA.size)
        fcntl.ioctl(self.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, data, True)
        return data[0]

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
