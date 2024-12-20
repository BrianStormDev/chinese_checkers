import rospy
from intera_interface import gripper as robot_gripper

def calibrate_gripper():
    """
    calibrates the gripper and returns the right_gripper object through which the gripper can be controlled
    """
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper', calibrate=True)
    return right_gripper

def control_gripper(right_gripper, open):
    """
    Control the gripper
    """
    # MAX_POSITION = 0.041667 
    # MIN_POSITION = 0.0

    # Higher values close it up
    # Lower values open it up

    # Open the right gripper
    open_val, close_val = [0.022, 0.033]
    if open:
        while input("Try opening the gripper: ") == "y":
            print('Opening gripper.')
            right_gripper.set_position(open_val)
            rospy.sleep(1)

    # Close the right gripper
    else:
        while input("Try closing the gripper: ") == "y":
            print('Closing gripper.')
            right_gripper.set_position(close_val)
            rospy.sleep(1)
    
    # Print out the gripper values
    print([open_val, close_val])

if __name__ == "__main__":
    rospy.init_node('gripper_node')
    right_gripper = calibrate_gripper()
    while True:
        control_gripper(right_gripper, True)
        control_gripper(right_gripper, False)