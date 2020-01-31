import pygame
from ..utils.communication import sender
import rospy
import roslib
from std_msgs.msg import Bool


# Global variables
SPEED_STEP = 0.05
NOW_SPEED = 0.3

pygame.init()
# Loop until the user clicks the close button.
done = False
# Used to manage how fast the screen updates.
clock = pygame.time.Clock()
# Initialize the joysticks.
pygame.joystick.init()
# socker 
sendman = sender()

# ROS
rospy.init_node('JoystickController')
pub = rospy.Publisher('Emergency', Bool, queue_size=1)
# -------- Main Program Loop -----------
while not rospy.is_shutdown():
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    # create message
    j = {
        'linear_x' : 0.0,
        'linear_y' : 0.0,
        'linear_z' : 0.0,
        'angular_x': 0.0,
        'angular_y': 0.0,
        'angular_z': 0.0,
    }
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    buttons = joystick.get_numbuttons()
    
    # for i in range(4):
    #     button = joystick.get_button(i)
    #     print("Button {:>2} value: {}".format(i, button))
    if joystick.get_button(3) == 1: # button Y
        NOW_SPEED = NOW_SPEED + SPEED_STEP
        if NOW_SPEED > 1.0:
            NOW_SPEED = 1.0
        print('speed up')
    if joystick.get_button(1) == 1: # button A
        NOW_SPEED = NOW_SPEED - SPEED_STEP
        if NOW_SPEED < 0.0:
            NOW_SPEED = 0.0
        print('speed down')
    if joystick.get_button(0) == 1: # button X
        NOW_SPEED = 0.5
        print('reset speed')

    # Hat position. All or nothing for direction, not a float like
    # get_axis(). Position is a tuple of int values (x, y).
    hat = joystick.get_hat(0)
    if hat[1] == 1:
        j['linear_x'] = NOW_SPEED
    if hat[1] == -1:
        j['linear_x'] = - NOW_SPEED
    if hat[0] == 1:
        j['angular_z'] = -0.3
    if hat[0] == -1:
        j['angular_z'] = 0.3

    if joystick.get_button(2) == 1: # button B : shutdowwn
        print("shutdown!!!!!!!!")
        j = {
            'linear_x' : 0.0,
            'linear_y' : 0.0,
            'linear_z' : 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
        }
        b = Bool()
        b.data = True
        pub.publish(b)

    # only send msgs when the input is not none
    if j['linear_x'] != 0.0 or j['angular_z'] != 0.0:
        sendman.send(j)
        print(j)
    clock.tick(20)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
