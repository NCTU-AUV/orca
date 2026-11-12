# 上層給 auv_adapter.py 的控制指令。
# adapter 以固定頻率把「最後收到的那筆」轉成 ArduSub 的搖桿指令（MANUAL_CONTROL）。
# 上層要持續發送（20 Hz）：超過 0.5 秒沒收到，adapter 改送中位（不切模式、不上鎖）。

# ===== 控制來源 =====
# 只標示是誰發的，adapter 不依它做任何處理
uint8 SW_MANUAL=0     # 鍵盤
uint8 SW_AUTO=1       # 自主程式
uint8 control_source

# ===== ArduSub 飛行模式 =====
# 每一筆都要填。跟飛控目前的模式不同時，adapter 會切換
uint8 MODE_MANUAL=0
uint8 MODE_STABILIZE=1
uint8 MODE_DEPTH_HOLD=2   # ArduSub 的 ALT_HOLD
uint8 mode

# ===== 解鎖／上鎖 =====
# 只在要改變時發 ARM 或 DISARM，之後改回 NO_CHANGE。
# 一直發 ARM 的話，飛控因 failsafe 自己上鎖後會被 adapter 重新解鎖
uint8 ARM_NO_CHANGE=0
uint8 ARM=1
uint8 DISARM=2
uint8 arm_command

# ===== 6 自由度 =====
# 範圍 [-1.0, 1.0]，1.0 = 推進器全力（飛控的 JS_GAIN_DEFAULT 要設 1.0）。
# MODE_MANUAL 下全部忽略，adapter 送中位，推進器不出力。
# STABILIZE／DEPTH_HOLD 下由飛控解讀：
#   surge、sway：推力
#   heave：STABILIZE 是推力；DEPTH_HOLD 是升降速度，0 = 維持深度
#   yaw：轉速，0 = 鎖住航向
#   roll、pitch：目標傾角，1.0 約 30°（ATC_ANGLE_MAX），0 = 保持水平
# 正負號照 ArduSub
float32 surge    # 前 (+) / 後 (-)
float32 sway     # 右 (+) / 左 (-)
float32 heave    # 上 (+) / 下 (-)
float32 roll     # 右側往下 (+)
float32 pitch    # 船頭往上 (+)
float32 yaw      # 從上往下看順時針，也就是右轉 (+)

# ===== 定深 =====
# 目標深度 [m]，往下為正。0 是呼叫 /auv_adapter/zero 當下的深度（剛下水、浮在水面時呼叫）
float32 target_depth
# 只在 DEPTH_HOLD 且已解鎖時有效。為 true 時 heave 一律當 0
bool use_depth_target

# ===== 航向 =====
# 目標航向 [deg]，相對於呼叫 /auv_adapter/zero 當下的航向，範圍 [-180, 180]，順時針為正。
# 超出範圍會自動換算回來
float32 target_yaw
# 只在 DEPTH_HOLD 且已解鎖時有效。為 true 時 roll、pitch 保持水平，
# roll、pitch、yaw 三個 effort 都忽略；surge、sway、定深照常。
# 改回 false 之後（ArduSub 的限制）：
#   5 秒內飛控仍鎖在最後的目標航向，yaw 輸入沒有反應；
#   5 秒後如果 yaw 是 0，飛控會轉回啟用 use_yaw_target 之前鎖住的航向
bool use_yaw_target
