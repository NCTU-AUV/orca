#!/usr/bin/env python3
"""電磁鐵吸放的實機測試（岸上）：直接在 Jetson 主機上跑，不需要 ROS、Docker、sudo。

    cd ~/workspace/orca
    python3 tools/hw_tests/magnet.py pins                  # 列出 40-pin 可用的 GPIO 腳和目前狀態（不動腳位）
    python3 tools/hw_tests/magnet.py --pin 29              # 互動測試（預設 HIGH = 通電吸住）
    python3 tools/hw_tests/magnet.py --pin 29 --active-low # 驅動模組是 LOW 觸發時
    python3 tools/hw_tests/magnet.py --pin 29 cycle -n 20 --on 3 --off 1   # 自動吸放 20 次

互動指令：
    h  吸住（電磁鐵通電）        r  放開（斷電）           t  切換
    p  放開 0.5 秒再吸回來（量放開的反應）
    s  目前的狀態                exit  放掉腳位後結束（看放掉之後電磁鐵會怎樣）
    q  結束（先放開、斷電，再放掉腳位）

一開始腳位設成「放開」（斷電）。要裝球時按 h。

第一次接線的檢查順序（每一步都要確認了才往下）：
  1. 不接電磁鐵、不接驅動模組，電錶量 40-pin 那一腳對 GND：h／r 要在 3.3 V 和 0 V 之間切換。
     沒變的話是 pinmux 沒把這支腳設成 GPIO：在 Jetson 螢幕前跑 sudo /opt/nvidia/jetson-io/jetson-io.py
     （要 sudo，請有密碼的人做）。
  2. 接上驅動模組（還不接電磁鐵）：模組的指示燈跟著 h／r 亮滅，確認是 HIGH 還是 LOW 觸發。
     反過來的話加 --active-low。
  3. 接電磁鐵、拿鋼球試：h 吸得住、r 掉得下來；斷電後有沒有剩磁（球黏著不掉）；
     cycle 跑幾十次看會不會發燙。
  4. exit：放掉腳位之後電磁鐵會不會自己放開或吸住（ball_dropper 節點掛掉、重開時就是這個狀態）。
  5. Jetson 重開機的過程中電磁鐵是什麼狀態（開機時腳位還沒人控制）。
結果記到 docs/test-logs/，確定的腳位與極性填進 src/orca_actuators/config/ball_dropper.yaml。

同一支腳一次只能一個程式用：ball_dropper 節點在跑時這裡會打不開（EBUSY），先停掉它。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'orca_actuators'))
from orca_actuators.gpio import HEADER, GpioError, OutputPin, describe  # noqa: E402


def stamp():
    return time.strftime('%H:%M:%S')


def show(pin):
    state = '吸住（通電）' if pin.value else '放開（斷電）'
    print(f'[{stamp()}] {state}：腳位輸出 {"HIGH" if pin.level() else "LOW"}', flush=True)


def interactive(pin):
    print(__doc__.split('互動指令：')[1].split('一開始')[0].rstrip())
    show(pin)
    while True:
        try:
            cmd = input('> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 'q'
        if cmd in ('h', 'r', 't'):
            pin.set(True if cmd == 'h' else False if cmd == 'r' else not pin.value)
            show(pin)
        elif cmd == 'p':
            pin.set(False)
            t0 = time.monotonic()
            show(pin)
            time.sleep(0.5)
            pin.set(True)
            print(f'  {time.monotonic() - t0:.2f} 秒後吸回來', flush=True)
            show(pin)
        elif cmd == 's':
            show(pin)
        elif cmd in ('q', 'exit'):
            return cmd
        elif cmd:
            print('指令：h r t p s q exit')


def cycle(pin, n, on_s, off_s):
    print(f'吸 {on_s:g} 秒、放 {off_s:g} 秒，共 {n} 次（Ctrl-C 停）', flush=True)
    t0 = time.monotonic()
    for i in range(1, n + 1):
        pin.set(True)
        print(f'[{stamp()}] {i}/{n} 吸住', flush=True)
        time.sleep(on_s)
        pin.set(False)
        print(f'[{stamp()}] {i}/{n} 放開', flush=True)
        time.sleep(off_s)
    print(f'完成，{time.monotonic() - t0:.0f} 秒', flush=True)


def main():
    ap = argparse.ArgumentParser(description='電磁鐵吸放的實機測試（Jetson 40-pin GPIO）',
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('action', nargs='?', default='interactive', choices=['interactive', 'cycle', 'pins'])
    ap.add_argument('--pin', type=int, help='40-pin 的實體腳號（建議 29 或 31）')
    ap.add_argument('--active-low', action='store_true', help='驅動模組是 LOW 觸發（腳位 LOW = 通電吸住）')
    ap.add_argument('-n', type=int, default=10, help='cycle 的次數')
    ap.add_argument('--on', type=float, default=2.0, help='cycle 每次吸住幾秒')
    ap.add_argument('--off', type=float, default=1.0, help='cycle 每次放開幾秒')
    args = ap.parse_args()

    try:
        if args.action == 'pins':
            for p, (name, alt) in sorted(HEADER.items()):
                print(f'{describe(p)}　別的功能：{alt}')
            return 0
        if args.pin is None:
            ap.error('要給 --pin（40-pin 的實體腳號）')
        print(describe(args.pin))
        print(f'極性：{"LOW" if args.active_low else "HIGH"} = 通電吸住', flush=True)
        pin = OutputPin(args.pin, active_high=not args.active_low, initial=False, consumer='magnet-test')
    except GpioError as e:
        print(f'錯誤：{e}', file=sys.stderr)
        return 1

    how = 'q'
    try:
        if args.action == 'cycle':
            cycle(pin, args.n, args.on, args.off)
        else:
            how = interactive(pin)
    except KeyboardInterrupt:
        print()
    finally:
        if how == 'exit':
            print(f'不改腳位就放掉（最後是「{"吸住" if pin.value else "放開"}」），看電磁鐵之後的狀態', flush=True)
        else:
            pin.set(False)
            print('已放開（斷電）', flush=True)
        pin.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
