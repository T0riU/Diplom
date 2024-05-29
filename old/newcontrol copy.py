import time
from pymavlink import mavutil

class DroneController:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.master = None

    def connect(self):
        print(f"Connecting to drone on {self.connection_string}")
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat()
        print("Heartbeat received from the drone!")

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
                    print(f"Mode changed to {mode}")
                    return True
                else:
                    print(f"Failed to change mode to {mode} with result: {ack.result}")
                    return False
            time.sleep(1)

    def arm(self):
        print("Arming the drone")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Drone is armed")

    def disarm(self):
        print("Disarming the drone")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("Drone is disarmed")

    def takeoff(self, altitude):
        print(f"Taking off to altitude: {altitude} meters")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )
        # Wait for acknowledgment
        ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Takeoff command accepted")
        else:
            print(f"Takeoff command failed with result: {ack}")
            self.disarm()
            return

        # Wait for the drone to reach the takeoff altitude
        self.wait_for_altitude(altitude)
        print("Takeoff complete")

    def land(self):
        print("Landing")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        # Wait for acknowledgment
        ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)
        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Landing command accepted")
        else:
            print(f"Landing command failed with result: {ack}")
            return

        # Wait for the drone to land
        self.wait_for_altitude(0.5, compare_func=lambda alt, target: alt <= target)
        print("Landing complete")
        self.disarm()

    def set_position(self, lat, lon, alt):
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
                print(f"Current position: Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt:.2f} meters")
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

def main():
    connection_string = 'udp:192.168.160.1:14550'  # Change as per your connection
    drone = DroneController(connection_string)

    drone.connect()

    # Change to GUIDED mode before arming
    if drone.change_mode('GUIDED'):
        drone.arm()
        drone.takeoff(10)  # Takeoff to 10 meters
        time.sleep(5)  # Wait for 5 seconds

        drone.set_position(47.397742, 8.545594, 20)  # Example coordinates and altitude
        time.sleep(5)  # Wait for 5 seconds

        drone.land()
        time.sleep(5)  # Wait for 5 seconds

if __name__ == "__main__":
    main()
