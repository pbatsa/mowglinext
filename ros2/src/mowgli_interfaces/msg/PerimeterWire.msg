# Raw perimeter-wire coil correlations reported by the STM32 firmware.
#
# Correlation values are unitless signed scores from the firmware matched
# filter. They are not distances or magnetic-field values. Higher absolute
# magnitude means a stronger detected signal; steering and wire-found
# thresholds should be owned by ROS2 navigation/docking logic.
uint8 SIGNAL_OFF=0
uint8 SIGNAL_1=1
uint8 SIGNAL_2=2

std_msgs/Header header
bool listening
uint8 signal_code
float32 left_correlation
float32 center_correlation
float32 right_correlation
