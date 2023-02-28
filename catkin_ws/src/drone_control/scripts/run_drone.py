import rospy
from drone_fsm import DroneFSM

# This file will control the behaviour of the drone (transition between states)

def main():
    # code to run drone
    
    ## MILESTONE 2 ##
    drone = DroneFSM()
    drone.arm()
    drone.takeoff(1.5) # m
    drone.hover(10.0) # s
    drone.land()

    return


if __name__ == '__main__':
    try:
        rospy.init_node('drone_control', log_level=rospy.DEBUG)
        main()
    except rospy.ROSInterruptException:
        pass