# 任务执行状态
uint8 STATE_IDLE=0
uint8 STATE_NAVIGATING=1
uint8 STATE_PTZ_MOVING=2
uint8 STATE_CAPTURING=3
uint8 STATE_COMPLETED=4
uint8 STATE_FAILED=5
uint8 STATE_CANCELED=6

uint8 state            # 当前状态
uint8 current_index    # 当前执行到第几个点 (0-based)
uint8 total_count      # 总点数
string message         # 状态描述
