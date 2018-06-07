# Has the gripper been activated?
bool ACT

# In what mode is the gripper?
int32 BASIC_MODE = 0
int32 WIDE_MODE = 1
int32 PINCH_MODE = 2
int32 SCISSOR_MODE = 3
int32 MOD

# Should gripper go to requested position?
bool GTO

# Is the gripper activating or changing mode?
int32 IN_RESET = 0
int32 DOING_ACT = 1
int32 DOING_MC = 2			
int32 ACT_MC_DONE = 3
int32 IMC

# Has the gripper been stopped?
int32 IN_MOTION = 0
int32 SOME_FINGERS_STOPPED = 1
int32 ALL_FINGERS_STOPPED = 2
int32 AT_REQUESTED_POSITION = 3
int32 STA

# Note that the scissors motion is also considered a finger !
FingerMsg[4] Fingers

bool[4] Fault


