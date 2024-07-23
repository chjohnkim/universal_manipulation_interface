#!/usr/bin/env python
# -*- coding: utf-8 -*-
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

import dynamixel_sdk as dynamixel
import sys, tty, termios
import numpy as np
import scipy 
import time, threading
import socket
import click

class GripperDriver(object):
    def __init__(self):
        # Control table address
        self.ADDR_OPERATING_MODE         = 11
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_PWM               = 100
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_PWM            = 124
        self.ADDR_PRESENT_LOAD           = 126
        self.ADDR_PRESENT_VELOCITY       = 128
        self.ADDR_PRESENT_POSITION       = 132
        self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        self.DXL_PWM_LIMIT               = 885       # Refer to the Maximum PWM Limit of product eManual
        self.BAUDRATE                    = 57600

        # Operating Modes
        # https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode
        self.OPERATING_MODE = {
            'velocity': 1,
            'position': 3, # Default
            'extended_position': 4,
            'pwm': 16
        }
        self.operating_mode = self.OPERATING_MODE['position']

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.PROTOCOL_VERSION            = 2.0

        # Factory default ID of all DYNAMIXEL is 1
        self.DXL_ID                      = 1

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME                  = '/dev/ttyUSB0'

        self.TORQUE_ENABLE               = 1     # Value for enabling the torque
        self.TORQUE_DISABLE              = 0     # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

        # Unit conversion
        self.degrees_per_pulse = 360.0/4096
        self.rev_per_min = 0.229
        self.load_percentage_per_unit = 0.1

        self._init_port_handler()
        self._init_pakcet_handler()
        self.calibrated = False
        self.torque_overload = False
        self.torque_protection_threshold = 60
        self.retries = 10
        self.offset = 5 # hacky offset to make sure gripper grasps the object

    ############################ Initialization ############################
    def _init_port_handler(self):
        # Initialize PortHandler instance
        self.portHandler = dynamixel.PortHandler(self.DEVICENAME)
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()

    def _init_pakcet_handler(self):
        # Initialize PacketHandler instance
        self.packetHandler = dynamixel.PacketHandler(self.PROTOCOL_VERSION)
        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    ############################ Utility ############################
    @staticmethod
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    @staticmethod
    def convert_to_signed_16bit(value):
        if value > 32767:
            value = value - 65536
        return value

    @staticmethod
    def convert_to_signed_32bit(value):
        if value > 2147483647:
            value = value - 4294967296
        return value

    def _check_results(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != dynamixel.COMM_SUCCESS:
            print("Error 1: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Error 2: %s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return True # No error
        return False # Error
        
    def enable_torque(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to enable torque. Closing gripper control.")

    def disable_torque(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        res = self._check_results(dxl_comm_result, dxl_error)        
        for _ in range(self.retries):
            if res:
                break
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            res = self._check_results(dxl_comm_result, dxl_error)        
        if not res:
            self.portHandler.closePort()
            print("Failed to disable torque. Closing gripper control.")

    def close(self):
        self.disable_torque()
        self.portHandler.closePort()
        print("Gripper control closed")

    ############################ Setters ############################
    def set_operating_mode(self, operating_mode):
        self.disable_torque()
        self.operating_mode = self.OPERATING_MODE[operating_mode]
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.operating_mode)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.operating_mode)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to set operating mode. Closing gripper control.")
        self.enable_torque()
        print(f"Operating mode set to {operating_mode}")

    def set_profile_velocity(self, profile_velocity):
        # profile_velocity: percentage of maximum velocity
        profile_velocity = int(np.clip(profile_velocity, 0, 1) * 32767)        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PROFILE_VELOCITY, profile_velocity)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PROFILE_VELOCITY, profile_velocity)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to set profile velocity. Closing gripper control.")
        print(f"Profile velocity set to {profile_velocity*self.rev_per_min:.3f} rev/min")

    def set_target_pwm(self, target_pwm):
        safety_coeff = 0.5 # TEMPORARY
        limit = int(self.DXL_PWM_LIMIT*safety_coeff)
        target_pwm = np.clip(target_pwm, -limit, limit)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, target_pwm)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, target_pwm)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to set target pwm. Closing gripper control.")


    def set_target_position(self, target_position):
        # Clip the position value to avoid exceeding the maximum or minimum position value
        if self.operating_mode == self.OPERATING_MODE['position']:
            target_position = np.clip(target_position, self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE)
        target_position = self.convert_to_signed_32bit(target_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, target_position)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, target_position)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to set target position. Closing gripper control.")

    def set_position_absolute(self, target_position):
        self.set_target_position(target_position)
        while 1:
            current_position = self.get_current_position()
            current_torque = self.get_current_load()
            print(f"GoalPos: {target_position:.1f}  PresPos: {current_position:.1f} PresLoad: {current_torque:.1f}%" )
            if not abs(target_position - current_position) > self.DXL_MOVING_STATUS_THRESHOLD:
                break

    ############################ Getters ############################
    def get_current_position(self): 
        # Units: Servo Ticks
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to get current position. Closing gripper control.")
        return self.convert_to_signed_32bit(dxl_present_position)

    def get_current_load(self):
        # Return the current load in percentage
        dxl_current_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_current_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to get current load. Closing gripper control.")
        percentage_load = self.convert_to_signed_16bit(dxl_current_load)*self.load_percentage_per_unit
        if abs(percentage_load) > self.torque_protection_threshold: 
            self.torque_overload = True
        return percentage_load
    
    # NOTE: Not sure what unit the velocity is in
    def get_current_velocity(self):
        dxl_current_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        res = self._check_results(dxl_comm_result, dxl_error)
        for _ in range(self.retries):
            if res:
                break
            dxl_current_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
            res = self._check_results(dxl_comm_result, dxl_error)
        if not res:
            self.portHandler.closePort()
            print("Failed to get current velocity. Closing gripper control.")
        return dxl_current_velocity

    ############################ UMI ############################

    def calibrate_gripper(self):
        
        self.wheel_radius = 10 # mm
        self.gripper_width = 80 # mm

        # Calibrate the gripper
        self.set_operating_mode('pwm')
        self.set_target_pwm(300)
        current_load = 0
        while abs(current_load) < 30:
            current_load = self.get_current_load()
            #print(f"Calibrating... Current Load: {current_load:.1f}%")
            sys.stdout.write(f"Calibrating... Current Load: {current_load:.1f}%   \r")
            sys.stdout.flush()

        # Record the closed and open positions
        self.set_operating_mode('extended_position')
        self.set_profile_velocity(0)
        closed_position = self.get_current_position()

        unwind_distance = self.gripper_width / 2.0
        unwind_ratio = unwind_distance / (2*np.pi*self.wheel_radius)
        unwind_angle = unwind_ratio * 360.0
        open_position = closed_position - int(unwind_angle/self.degrees_per_pulse)

        # Set the interpolation functions, clip if the position is out of bounds
        self.interp_mm2tick = scipy.interpolate.interp1d([self.gripper_width, 0], 
                                                 [open_position, closed_position], 
                                                 fill_value=(open_position, closed_position), 
                                                 bounds_error=False)
        self.interp_tick2mm = scipy.interpolate.interp1d([open_position, closed_position], 
                                                     [self.gripper_width, 0], 
                                                     fill_value=(self.gripper_width, 0), 
                                                     bounds_error=False)
        self.calibrated = True
        self.set_gripper_width(self.gripper_width)
        print("Succeeded to calibrate the gripper")

    def set_gripper_width(self, width_mm, wait=True):
        width_mm = max(0, width_mm - self.offset)
        self.target_width_mm = width_mm
        if not self.calibrated:
            print("Gripper not calibrated. Please calibrate the gripper first.")
            return
        target_position_tick = int(self.interp_mm2tick(width_mm))
        self.set_target_position(target_position_tick)
        if wait:
            while width_mm==self.target_width_mm:
                current_position_tick = self.get_current_position()
                current_opening_mm = self.interp_tick2mm(current_position_tick)
                current_torque = self.get_current_load()
                sys.stdout.write(f"Target Width: {width_mm:>6.1f}mm  |  Current Width: {current_opening_mm:>6.1f}mm  |  Current Load: {current_torque:>6.1f}% \r")
                sys.stdout.flush()
                if not abs(target_position_tick - current_position_tick) > self.DXL_MOVING_STATUS_THRESHOLD:
                    break
                if self.torque_overload:
                    break
        return self.get_state()
        
    def get_gripper_width(self):
        # Get the current gripper width in mm
        if not self.calibrated:
            print("Gripper not calibrated. Please calibrate the gripper first.")
            return
        current_position_tick = self.get_current_position()
        current_width_mm = self.interp_tick2mm(current_position_tick)
        return current_width_mm

    def get_state(self):
        while True:
            try:
                state = {
                    'position': self.get_gripper_width() + self.offset, # mm
                    'velocity': self.get_current_velocity(),
                    'force': self.get_current_load(), # percentage
                }
                break
            except:
                pass
        return state
        
    ############################ Client-Server ############################
    def server(self, server_address):
        # server_address = ('localhost', 65432)
        self.server_running = True
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind(server_address)
            server_socket.listen()
            print(f"Server listening on {server_address}")

            conn, addr = server_socket.accept()
            with conn:
                print(f"Connected by {addr}")
                input_thread = threading.Thread(target=self._non_blocking_client, args=(conn,))
                input_thread.start()
                self.calibrate_gripper()
                self.target_width_mm = current_width = self.get_gripper_width()
                try:
                    while self.server_running:
                        if self.target_width_mm != current_width and not self.torque_overload:
                            self.set_gripper_width(self.target_width_mm)
                        current_width = self.get_gripper_width()
                        current_load = self.get_current_load()
                        status = f"Target Width: {self.target_width_mm:>6.1f}mm  |  Current Width: {current_width:>6.1f}mm  |  Current Load: {current_load:>6.1f}%"
                        print(status)
                        conn.sendall(status.encode())
                        if self.torque_overload:
                            self.set_position_absolute(int(self.interp_mm2tick(self.gripper_width)))
                            self.server_running = False
                            print("WARNING: Overload protection triggered. Server closing.")
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("Server interrupted by user.")
                finally:
                    self.server_running = False
                    input_thread.join()
                    self.close()
                    print("Server closed.")

    def _non_blocking_client(self, conn):
        while self.server_running:
            cmd = conn.recv(1024).decode()
            if cmd == 'q':
                self.server_running = False
                break
            try:
                width_mm = int(cmd)
                if 0 <= width_mm <= 80:
                    self.target_width_mm = width_mm
                else:
                    conn.sendall(b"Invalid width. Please enter a value between 0 and 80.")
            except ValueError:
                conn.sendall(f"Invalid input: {cmd}. Please enter a valid integer.".encode())

def client(server_address):
    # server_address = ('localhost', 65432)    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect(server_address)
        
        while True:
            cmd = input("Enter target width (mm) (0-80) or 'q' to quit: ")
            client_socket.sendall(cmd.encode())
            if cmd == 'q':
                break
            response = client_socket.recv(1024)
            print(f"Server response: {response.decode()}")


@click.command()
@click.option('--mode', default='gui', help="Run as 'server' or 'client' or 'gui'") 
def main(mode):
    if mode == 'client':
        client(server_address=('localhost', 65432))
    elif mode == 'server':
        gripper = GripperDriver()
        gripper.server(server_address=('localhost', 65432))
    else: 
        gripper = GripperDriver()
        gripper.calibrate_gripper()
        #gripper.set_operating_mode('extended_position')
        #gripper.set_profile_velocity(0.001)
        #gripper.set_position_absolute(0)
        t = time.time()
        for i in range(200):
            gripper.get_state()
        t_total = time.time()-t
        gripper.close()
        print(f"Total time: {t_total:.3f}sec")
        print(f"RTT: {t_total/200:.3f}sec")


if __name__ == '__main__':
    main()
