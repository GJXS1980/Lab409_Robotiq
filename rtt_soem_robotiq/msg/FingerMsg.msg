# Has the finger been stopped?
int32 IN_MOTION = 0
int32 STOPPED_OPENING = 1
int32 STOPPED_CLOSING = 2
int32 AT_REQUESTED_POSITION = 3
int32 DTi

# Echo of the requested position
uint8 ReqPosition
# Echo of the actual position
uint8 ActPosition
# Echo of the actual current
uint8 Current
