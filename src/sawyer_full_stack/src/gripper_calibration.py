import rospy
from intera_interface import gripper as robot_gripper
# Used to calibrate the gripper so that it can pick up the balls on the board consistently while not opening
# so far that the sides of the gripper hit other sides of the board

# How long to sleep in between actions
SLEEP = 1.0

def calibrate_gripper():
    """
    calibrates the gripper and returns the right_gripper object through which the gripper can be controlled
    """
    return robot_gripper.Gripper('right_gripper', calibrate=True)

def control_gripper(right_gripper, open):
    """
    Control the gripper
    right_gripper: Gripper object
    open: bool determining whether to close or open the gripper
    """
    # MAX_POSITION = 0.041667 
    # MIN_POSITION = 0.0
    # Higher values close it up
    # Lower values open it up

    # The open and close values for the gripper
    open_val, close_val = [0.028, 0.034]

    # Open the right gripper
    if open:
        if input("Try opening the gripper: ") == "y":
            print('Opening gripper.')
            right_gripper.set_position(open_val)
            rospy.sleep(SLEEP)

    # Close the right gripper
    else:
        if input("Try closing the gripper: ") == "y":
            print('Closing gripper.')
            right_gripper.set_position(close_val)
            rospy.sleep(SLEEP)

    # Print out the gripper values for easier plugin into the main fuile
    print([open_val, close_val])

if __name__ == "__main__":
    rospy.init_node('gripper_node') # Create the ros node 
    right_gripper = calibrate_gripper() # Calibrate the gripper
    while True: # Loop through both opening and closing until the user ends the program
        control_gripper(right_gripper, True)
        control_gripper(right_gripper, False)