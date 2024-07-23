import time
from rtde_receive import RTDEReceiveInterface

def get_robot_pose():
    rtde_r = RTDEReceiveInterface("192.168.0.11")  # replace with your robot's IP
    try:
        # time frequency
        t0 = time.time()
        for i in range(100000):
            pose = rtde_r.getActualTCPPose()
            print(pose[:3])
            #time.sleep(0.1)  # Adjust the sleep time as needed
        t1 = time.time()
        frequency = 100000 / (t1 - t0)
        print(f"Frequency: {frequency} Hz")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rtde_r.disconnect()

if __name__ == "__main__":
    get_robot_pose()