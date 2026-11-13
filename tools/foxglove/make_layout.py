#!/usr/bin/env python3
"""產生 Foxglove layout：orca_mission.json（看 deploy/sim/mission.sh 錄的 rosbag）。

    python3 tools/foxglove/make_layout.py      # 改了這個檔或 *.ts 之後重新產生

Foxglove：右上角 layout 選單 → Import from file… → 選 tools/foxglove/orca_mission.json，
再開 deploy/sim/logs/bag/<樹>_<時間>/ 裡的 .mcap。

兩個分頁：
- 過門：前視、底部畫好框的影像，行為樹節點／飛控模式／解鎖／有沒有看到門，
  門的偏離與框寬、深度、推力
- 飛控、log：航向、/studio_script/mission 整筆、/mavros/state、/rosout

/mission/status 是 JSON 字串，Plot 不能直接用，所以 layout 裡帶兩個 User Script
（mission.ts、gate.ts）把它和 /detections 拆成欄位，輸出 /studio_script/mission、/studio_script/gate。
"""
import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / 'orca_mission.json'

MISSION = '/studio_script/mission'
GATE = '/studio_script/gate'


def image(topic):
    return {'imageMode': {'imageTopic': topic}, 'followMode': 'follow-pose', 'scene': {}, 'transforms': {},
            'topics': {}, 'layers': {}, 'synchronize': False}


def plot(*paths):
    return {'paths': [{'value': v, 'enabled': True, 'timestampMethod': 'receiveTime', 'label': label, 'color': color}
                      for v, label, color in paths],
            'showXAxisLabels': True, 'showYAxisLabels': True, 'showLegend': True, 'legendDisplay': 'floating',
            'isSynced': True, 'xAxisVal': 'timestamp'}


def states(*paths):
    return {'paths': [{'value': v, 'timestampMethod': 'receiveTime', 'label': label} for v, label in paths],
            'isSynced': True}


def split(first, second, direction, percent):
    return {'first': first, 'second': second, 'direction': direction, 'splitPercentage': percent}


config = {
    'Image!front': image('/detections/image/compressed'),
    'Image!bottom': image('/detections/bottom_image/compressed'),
    'StateTransitions!mission': states(
        (f'{MISSION}.node', '行為樹節點'),
        (f'{MISSION}.fc_mode', '飛控模式'),
        (f'{MISSION}.armed', '解鎖'),
        (f'{GATE}.seen', '前視看到門'),
    ),
    'Plot!gate': plot(
        (f'{GATE}.lateral', '偏離門中線（門寬）：CenterGate 在 ±0.1 內算對準', '#4e9a06'),
        (f'{GATE}.width', '門框寬／畫面寬：ApproachGate 到 0.7 開始盲走', '#c4a000'),
    ),
    'Plot!depth': plot(
        (f'{MISSION}.depth', '深度 [m]（歸零後，2 Hz）', '#3465a4'),
        ('/auv_control.target_depth', '目標深度 [m]', '#ef2929'),
    ),
    'Plot!thrust': plot(
        ('/auv_control.surge', 'surge（前 +）', '#75507b'),
        ('/auv_control.sway', 'sway（右 +）', '#f57900'),
    ),
    'Plot!heading': plot(
        (f'{MISSION}.heading', '航向 [deg]（歸零後，2 Hz）', '#3465a4'),
        ('/auv_control.target_yaw', '目標航向 [deg]', '#ef2929'),
    ),
    'RawMessages!mission': {'topicPath': MISSION, 'fontSize': 12, 'defaultExpanded': True},
    'RawMessages!mavros_state': {'topicPath': '/mavros/state', 'fontSize': 12, 'defaultExpanded': True},
    'RosOut!log': {'searchTerms': [], 'minLogLevel': 1, 'topicToRender': '/rosout'},
    'Tab!orca_mission': {
        'activeTabIdx': 0,
        'tabs': [
            {'title': '過門',
             'layout': split(
                 split('Image!front', 'Image!bottom', 'column', 62),
                 split('StateTransitions!mission',
                       split('Plot!gate', split('Plot!depth', 'Plot!thrust', 'column', 50), 'column', 34),
                       'column', 28),
                 'row', 50)},
            {'title': '飛控、log',
             'layout': split(
                 split('Plot!heading', 'RawMessages!mission', 'column', 45),
                 split('RawMessages!mavros_state', 'RosOut!log', 'column', 35),
                 'row', 50)},
        ],
    },
}

layout = {
    'configById': config,
    'globalVariables': {},
    'userNodes': {
        'orca-mission-status': {'name': 'mission（/mission/status → 欄位）',
                                'sourceCode': (HERE / 'mission.ts').read_text()},
        'orca-gate': {'name': 'gate（/detections → 門的偏離、框寬）',
                      'sourceCode': (HERE / 'gate.ts').read_text()},
    },
    'playbackConfig': {'speed': 1},
    'layout': 'Tab!orca_mission',
}

OUT.write_text(json.dumps(layout, ensure_ascii=False, indent=2) + '\n')
print(f'寫入 {OUT}')
