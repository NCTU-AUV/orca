"""gz service 的請求，放在另一個行程裡送。

gz-transport 的 Python 綁定：同一個行程裡如果有訂閱（尤其是 /world/<名稱>/clock、影像、dynamic_pose
這種高頻的 topic），request() 常常等不到回應而逾時，即使 server 早就回了（用 gz service 命令列送同一個
請求是好的）。看起來是收回應的執行緒卡在等 GIL 的訂閱 callback 上。所以請求交給一個沒有任何訂閱的
子行程送，這邊只透過 pipe 傳序列化後的訊息。

    from gzreq import Requester
    req = Requester()                       # 在建立任何 gz Node 之前建立最保險
    ok, rep = req.request('/world/x/set_pose', pose_msg, Boolean, timeout=5000)

用到它的：sim/tests/referee_check.py、tools/sim_detect/capture.py。
"""
import importlib
import multiprocessing as mp


def _worker(conn):
    from gz.transport13 import Node
    node = Node()
    while True:
        item = conn.recv()
        if item is None:
            return
        service, req_t, rep_t, payload, timeout = item
        ReqT = getattr(importlib.import_module(req_t[0]), req_t[1])
        RepT = getattr(importlib.import_module(rep_t[0]), rep_t[1])
        req = ReqT()
        req.ParseFromString(payload)
        try:
            ok, rep = node.request(service, req, ReqT, RepT, timeout)
            conn.send((bool(ok), rep.SerializeToString() if ok else b''))
        except Exception:                   # noqa: BLE001  服務不存在之類的，當成失敗
            conn.send((False, b''))


class Requester:
    def __init__(self):
        ctx = mp.get_context('spawn')       # 全新的直譯器，不帶父行程的 gz 狀態與執行緒
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_worker, args=(child,), daemon=True)
        self.proc.start()

    def request(self, service, req, rep_type, timeout=5000):
        t = type(req)
        self.conn.send((service, (t.__module__, t.__name__), (rep_type.__module__, rep_type.__name__),
                        req.SerializeToString(), timeout))
        ok, data = self.conn.recv()
        rep = rep_type()
        if ok:
            rep.ParseFromString(data)
        return ok, rep

    def close(self):
        try:
            self.conn.send(None)
        except (BrokenPipeError, OSError):
            pass
        self.proc.join(2)
