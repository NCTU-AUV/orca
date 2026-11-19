"""fc_params：把飛控目前的全部參數存成 .parm。要在有 mavros 的容器（fc_adapter）裡跑。

    ros2 run orca_session fc_params /sessions/<場次>/info/fc_params_start.parm

- 格式跟 firmware/ardusub/real.parm 一樣（一行「名稱 值」，照名稱排序，# 開頭是註解），可以直接 diff。
- mavros 把飛控參數存成 /mavros/param 這個節點的 ROS 參數。這裡先呼叫 /mavros/param/pull
  強制從飛控重拉一次（拿到的是當下的值），等拉完再全部讀出來，數量對不上就算失敗。
- 飛控存的是 float32，換成 double 會多出尾數（0.18 → 0.18000000715255737），寫檔時取 7 位有效數字。
"""
import argparse
import os
import re
import sys
import time

import rclpy
from mavros_msgs.srv import ParamPull
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters

PARAM_NODE = '/mavros/param'
CHUNK = 200     # GetParameters 一次問幾個
FC_NAME = re.compile(r'[A-Z0-9_]+')


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def fmt(value):
    if value.type == ParameterType.PARAMETER_INTEGER:
        return str(value.integer_value)
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return f'{value.double_value:.7g}'
    return None


def pull(node, timeout):
    """叫 mavros 從飛控重新拉一次全部參數，等拉完。回傳拉到幾個。

    - 一定要 force_pull：不強制的話，mavros 的快取裡只要有東西就馬上回傳成功
      （實測剛連上時只有 1 個也算成功），而且快取不一定是飛控當下的值。
    - mavros 連上飛控約 10 秒後會自己拉一次，正在拉的時候呼叫會回傳失敗，所以隔 2 秒重試到 timeout。
    """
    client = node.create_client(ParamPull, f'{PARAM_NODE}/pull')
    deadline = time.monotonic() + timeout
    while True:
        res = call(node, client, ParamPull.Request(force_pull=True), max(deadline - time.monotonic(), 1.0))
        if res is not None and res.success:
            return res.param_received
        if time.monotonic() > deadline:
            raise RuntimeError(f'{PARAM_NODE}/pull 沒成功（飛控沒連上，或參數一直沒拉完）')
        time.sleep(2.0)


def read_params(node, timeout):
    """回傳 {名稱: 值的字串}；失敗時丟 RuntimeError。"""
    received = pull(node, timeout)
    listing = call(node, node.create_client(ListParameters, f'{PARAM_NODE}/list_parameters'),
                   ListParameters.Request(), timeout)
    if listing is None:
        raise RuntimeError(f'{PARAM_NODE}/list_parameters 沒回應')
    # 飛控參數都是大寫英數字加底線；use_sim_time 這類 ROS 自己的參數不要
    names = sorted(n for n in listing.result.names if FC_NAME.fullmatch(n))
    getter = node.create_client(GetParameters, f'{PARAM_NODE}/get_parameters')
    out = {}
    for i in range(0, len(names), CHUNK):
        part = names[i:i + CHUNK]
        res = call(node, getter, GetParameters.Request(names=part), timeout)
        if res is None:
            raise RuntimeError(f'{PARAM_NODE}/get_parameters 沒回應')
        for name, value in zip(part, res.values):
            text = fmt(value)
            if text is not None:
                out[name] = text
    if len(out) != received:
        raise RuntimeError(f'mavros 說拉到 {received} 個，讀出來是 {len(out)} 個')
    return out


def main():
    ap = argparse.ArgumentParser(description='把飛控的全部參數存成 .parm')
    ap.add_argument('out', help='輸出的 .parm 路徑（資料夾不存在會建立）')
    ap.add_argument('--note', default='', help='寫在檔頭註解的說明')
    ap.add_argument('--timeout', type=float, default=60.0, help='每個服務呼叫最多等幾秒')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node(f'fc_params_{os.getpid()}')
    try:
        params = read_params(node, args.timeout)
    except RuntimeError as e:
        print(f'讀不到飛控參數：{e}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    tmp = args.out + '.tmp'
    with open(tmp, 'w') as f:
        f.write(f'# 飛控參數快照，{time.strftime("%Y-%m-%d %H:%M:%S")} 經 mavros 讀取，共 {len(params)} 個。'
                f'{args.note}\n')
        for name, text in params.items():
            f.write(f'{name} {text}\n')
    os.replace(tmp, args.out)
    print(f'飛控參數 {len(params)} 個 → {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
