"""場次（session）紀錄的共用部分：目前是哪個場次、紀錄檔放哪裡、發標記。

場次 = 一次下水，從 deploy/<跑法>/up.sh 開始到 down.sh 結束，紀錄都在 logs/session/<場次>/。
容器把整個 logs/session/ 掛在 /sessions，目前場次的名稱寫在 /sessions/current。
容器的 volume 在建立時就固定了，fc_adapter 這種常駐的容器會跨好幾次 up.sh，
所以程式每次要寫檔時才讀 current，不靠建立容器時的環境變數。
"""
import os
import time

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SESSIONS_DIR = os.environ.get('ORCA_SESSIONS_DIR', '/sessions')
MARKER_TOPIC = '/session/marker'
# 可靠傳送、不保留歷史（volatile）。保留歷史的話，recorder 每次重開都會再收到一次舊標記
MARKER_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
# 訂閱標記的有兩個：recorder（錄進 rosbag）、marker_log（寫 markers.csv）
MARKER_SUBSCRIBERS = 2


def current():
    """目前場次的名稱。沒有場次，或容器沒掛 /sessions（例如模擬）時回傳 None。"""
    try:
        with open(os.path.join(SESSIONS_DIR, 'current')) as f:
            return f.read().strip() or None
    except OSError:
        return None


def log_dir(kind, fallback='/logs'):
    """紀錄檔（CSV）要放的資料夾，依序：環境變數 ORCA_LOG_DIR → 目前場次的 <kind>/ → fallback。

    模擬沒有場次，容器把 deploy/sim/logs/<服務>/ 掛在 /logs，所以落到 fallback。
    """
    env = os.environ.get('ORCA_LOG_DIR')
    if env:
        return env
    name = current()
    return os.path.join(SESSIONS_DIR, name, kind) if name else fallback


def git_version():
    """程式版本：主機上的腳本（mission.sh、pool_test.sh）算好傳進來的 ORCA_GIT。

    格式是 commit 的前 7 碼，有未提交的修改時加上「+dirty」。容器裡沒有 .git，自己算不出來。
    """
    return os.environ.get('ORCA_GIT') or 'git 版本未知'


class Marker:
    """發標記到 /session/marker，內容是「<來源>: <文字>」，同時寫進 node 的 log（/rosout）。

    recorder 把它錄進 rosbag，marker_log 把它寫進場次的 markers.csv。
    """

    def __init__(self, node, source):
        self.node = node
        self.source = source
        self.pub = node.create_publisher(String, MARKER_TOPIC, MARKER_QOS)

    def subscribers(self):
        return self.pub.get_subscription_count()

    def wait_subscribers(self, timeout):
        """等 recorder、marker_log 都訂閱上，最多 timeout 秒。回傳訂閱者數量。

        剛建立 publisher 時，recorder 還沒發現這個 topic，馬上發的標記會漏掉。
        DDS 的探索在背景執行緒跑，這裡不用 spin。
        """
        end = time.monotonic() + timeout
        while self.subscribers() < MARKER_SUBSCRIBERS and time.monotonic() < end:
            time.sleep(0.05)
        return self.subscribers()

    def __call__(self, text, wait=0.0):
        """發一筆標記。wait > 0 時先等訂閱者最多 wait 秒（只在剛建立、不趕時間的時候用）。"""
        if wait > 0:
            self.wait_subscribers(wait)
        line = f'{self.source}: {text}'
        self.pub.publish(String(data=line))
        self.node.get_logger().info(f'標記 {line}')
