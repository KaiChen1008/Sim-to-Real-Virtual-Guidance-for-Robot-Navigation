import enum

PROTOCOLS   = enum.Enum('PROTOCOLS',    'SOCKET ROS')

class ThreeDims():
    x = 0.0
    y = 0.0
    z = 0.0

class Position():
    x = 0.0
    y = 0.0
    z = 0.0

class Orientation():
    x = 0.0
    y = 0.0
    z = 0.0

class Pose():
    Position    = Position()
    Orientation = Orientation()


#class Twist():
#    linear = ThreeDims()
#    angular= ThreeDims()
