byte RARMID=0
byte LARMID=1

# task: push, pick, open, slide, MoveNeck, MoveBase
string task_name

# selection: button names
# string selection_name

# arm: :rarm, :lsrm
int64 arm_id

# state of touch: touch, pick, pinch, sweep
# for debugging
string state
# value of state
float64 state_value

# direction: up, down, left, right
string direction
# value of direction, degree
float64 direction_value

int64 touch_x
int64 touch_y

# Example
#  push: touches[0].x, touches[0].y, task_name, arm_id
#  open: touches[0 1 2].x, touches[0 1 2].y, task_name, arm_id
#  slide: touches[0 1 2].x, touches[0 1 2].y, task_name, arm_id
#  move_neck: task_name, direction, direction_value
#  move_base: task_name, direction, direction_value