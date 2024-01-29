uint8 ADD = 0
uint8 DELETE = 1

# constants for the horizontal and vertical alignment
uint8 LEFT = 0
uint8 RIGHT = 1
uint8 CENTER = 2
uint8 TOP = 3
uint8 BOTTOM = 4

uint8 action

int32 width
int32 height
# Position: Positive values move the overlay towards the center of the window,
#  for center alignment positive values move the overlay towards the bottom right
int32 horizontal_distance # Horizontal distance from left/right border or center, depending on alignment
int32 vertical_distance # Vertical distance between from top/bottom border or center, depending on alignment

# Alignment of the overlay withing RVIZ
uint8 horizontal_alignment # one of LEFT, CENTER, RIGHT
uint8 vertical_alignment # one of TOP, CENTER, BOTTOM

std_msgs/ColorRGBA bg_color

int32 line_width
float32 text_size
string font
std_msgs/ColorRGBA fg_color

string text
