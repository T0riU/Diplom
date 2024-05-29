import threading
import time
import webview
from jinja2 import Environment, FileSystemLoader, select_autoescape
from pymavlink import mavutil
import tkinter as tk
from tkinter import ttk
import queue

class DroneController:
    def __init__(self, master):
        self.master = master
        self.armed = False
        self.flying = False
        self.mode = None
        self.start = True
        
    def initialize(self):
       self.change_mode("GUIDED")
       self.land()
       self.change_mode("GUIDED")
       self.start = False
       
    def change_mode(self, mode):
        print(f"Changing mode to {mode}")
        if mode not in self.master.mode_mapping():
            print(f"Mode {mode} not supported")
            return False
        
        mode_id = self.master.mode_mapping()[mode]
        self.master.set_mode(mode_id)
        
        while True:
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.mode = mode
                    print(f"Mode changed to {mode}")
                    return True
                else:
                    print(f"Failed to change mode to {mode} with result: {ack.result}")
                    return False
            time.sleep(1)

    def arm(self):
        print("Arming the drone")
        # Check if the mode is armable
        if self.mode != "LAND":
            print("\tMode is not LAND, arming the drone.")
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            self.armed = True
            print("Drone is armed")
        else:
            print("LAND mode is not armable. Cannot arm the drone.")

    def disarm(self):
        print("Disarming the drone")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        self.armed = False
        print("Drone is disarmed")

    def takeoff(self, altitude):
        if not self.armed or self.mode != "GUIDED" or self.flying:
            print("Invalid conditions for takeoff.")
            return

        print(f"Taking off to altitude: {altitude} meters")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Takeoff command accepted")
            self.flying =True
        else:
            print(f"Takeoff command failed with result: {ack}")
            return
        self.wait_for_altitude(altitude)
        print("Takeoff complete")

    def land(self):
        print(f"{not self.flying} {not self.armed} {not self.start} ------- {(not self.flying or not self.armed)}---------{(not self.flying or not self.armed) and not self.start}")
        if (not self.flying or not self.armed) and not self.start:
            print("Invalid conditions for landing.")
            return

        print("Landing")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)
        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Landing command accepted")
            self.flying = False
        else:
            print(f"Landing command failed with result: {ack}")
            return

        self.wait_for_altitude(0.5, compare_func=lambda alt, target: alt <= target)
        print("Landing complete")
        self.disarm()

    def set_position(self, lat, lon, alt):
        if not self.armed or self.mode != "GUIDED" or not self.flying:
            print("Cannot set position while flying.")
            return

        print(f"Setting position: Lat={lat}, Lon={lon}, Alt={alt}")
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        print("Position set")
        self.wait_for_position(lat, lon, alt)
        # No waiting for position since position can be overridden.

    def wait_for_altitude(self, target_altitude, timeout=30, compare_func=lambda alt, target: alt >= target):
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                altitude = msg.relative_alt / 1000.0
                print(f"Current altitude: {altitude:.2f} meters")
                if compare_func(altitude, target_altitude):
                    return True
        print(f"Timeout waiting for altitude {target_altitude}")
        return False
    
    def wait_for_position(self, target_lat, target_lon, target_alt, timeout=60):
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                if (abs(lat - target_lat) < 0.00001 and
                    abs(lon - target_lon) < 0.00001 and
                    abs(alt - target_alt) < 0.5):
                    print("Arrived at target position")
                    return True
        print("Timeout waiting for position")
        return False
    
    def wait_for_ack(self, command):
            while True:
                msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
                if msg.command == command:
                    return msg.result 
                
                # Assuming you have the necessary imports and class definition here

# Example usage
if __name__ == "__main__":
    connection_string = '{}:{}'.format('192.168.160.1', '14550')
    connection = mavutil.mavlink_connection(connection_string, baud=115200)
    connection.wait_heartbeat(timeout=10)
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
    if connection:
        master = connection

    master.mav.request_data_stream_send(
        master.target_system,    # Target system ID
        master.target_component, # Target component ID
        mavutil.mavlink.MAV_DATA_STREAM_ALL, # All streams
        10,  # Hz
        1    # Enable
    )

    # Main loop to receive and process messages
    while True:
        msg = master.recv_match(type=['NAV_PITCH', 'NAV_ROLL'])

        if msg:
            if msg.get_type() == 'NAV_PITCH':
                # NAV_PITCH message received
                print("NAV_PITCH:", msg.pitch)
            elif msg.get_type() == 'NAV_ROLL':
                # NAV_ROLL message received
                print("NAV_ROLL:", msg.roll)
    # # Initialize your DroneController instance
    # drone_controller = DroneController(master)
    # drone_controller.initialize()
    # drone_controller.arm()
    # print(4)
    # drone_controller.takeoff(5)
    # print(2)
    # drone_controller.set_position(-35, 150, 20)
    # drone_controller.takeoff(20)
    # print(3)
    # drone_controller.land()
    # print(5)
    #drone_controller.set_position(-35, 150, 20)
    # drone_controller.change_mode("GUIDED")
    # # Arm the drone
    # drone_controller.arm()

    # # Change mode to GUIDED
    

    # # # Set a position (this can be overridden)
    # # drone_controller.set_position(37.7749, -122.4194, 10)

    # # # Takeoff to an altitude of 15 meters
    # # drone_controller.takeoff(15)

    # # # Set a new position (this will override the previous position)
    # # drone_controller.set_position(37.7749, -122.4194, 20)

    # # Land the drone
    # drone_controller.land()

