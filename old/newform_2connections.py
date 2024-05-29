import threading
import time
import webview
from jinja2 import Environment, FileSystemLoader, select_autoescape
from pymavlink import mavutil
import tkinter as tk
from tkinter import ttk

file_names = ['hud.html', 'test3.html', 'test4.html']
def check_mavlink_connection(ip, port, timeout=10):
    try:
        # Connect to the MAVLink device
        connection_string = '{}:{}'.format(ip, port)
        connection1 = mavutil.mavlink_connection(connection_string, baud=115200)
        connection2 = mavutil.mavlink_connection(connection_string, baud=115200)
        # Wait for the heartbeat message to confirm the connection
        connection1.wait_heartbeat(timeout=timeout)
        
        print(connection1.target_system)
        connection2.wait_heartbeat(timeout=timeout)
        print("-----------------------")
        print(connection2.target_system)
        if connection1.target_system == 0 or connection2.target_system == 0:
            raise Exception("Port is not working or Drone is not in the network")
        print("Heartbeat from system 1 (system %u component %u)" % (connection1.target_system, connection1.target_component))
        print("Heartbeat from system 2 (system %u component %u)" % (connection2.target_system, connection2.target_component))
        return connection1, connection2
    except Exception as e:
        exception_form = ExceptionForm(f"Failed to connect to MAVLink device: {e}")
        print(f"Failed to connect to MAVLink device: {e}")
        return None, None
    
class ExceptionForm:

    def __init__(self, exception_message):
        self.root = tk.Tk()
        self.root.title("Exception")
        self.exception_label = tk.Label(self.root, text=exception_message, wraplength=300, justify='center')
        self.exception_label.pack(padx=10, pady=10)
        self.close_button = tk.Button(self.root, text="Close", command=self.root.destroy)
        self.close_button.pack(pady=10)

        self.root.mainloop()
        
class TelemetryApp:

    def __init__(self, root):
        self.root = root
        self.root.title("MAVLink Connection")
        self.default_ip = '192.168.160.1'
        self.default_port = '14550'
        self.create_widgets()

    def create_widgets(self):
        ttk.Label(self.root, text="IP Address:").grid(row=0, column=0, padx=5, pady=5)
        self.ip_entry = ttk.Entry(self.root)
        self.ip_entry.insert(0, self.default_ip) 
        self.ip_entry.grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(self.root, text="Port:").grid(row=1, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(self.root)
        self.port_entry.insert(0, self.default_port)
        self.port_entry.grid(row=1, column=1, padx=5, pady=5)
        self.connect_button = ttk.Button(self.root, text="Connect", command=self.connect_to_mavlink)
        self.connect_button.grid(row=2, column=0, columnspan=2, padx=5, pady=10)

    def connect_to_mavlink(self):
        ip = self.ip_entry.get()
        port = self.port_entry.get()
        connection1, connection2 = check_mavlink_connection(ip, port)
        if connection1 and connection2:
            self.master1 = connection1
            self.master2 = connection2
            self.connected = True
            self.root.destroy()
            self.open_telemetry_form()

    def open_telemetry_form(self):
        if self.connected:
            telemetry_gui = TelemetryGUI(self.master1, self.master2)
            telemetry_gui.open_gui()
            
class TelemetryGUI:
    def __init__(self, master1, master2):
        self.master1 = master1
        self.master2 = master2
        self.clwin = False
        self.data = {
            'gps_lat': '',
            'gps_lon': '',
            'gps_alt': '',
            'battery_voltage': '',
            'battery_current': '',
            'battery_remaining': '',
            'imu_roll': '',
            'imu_pitch': '',
            'imu_yaw': '',
            'compass_heading': '',
            'compass_groundspeed': '',
            'temperature': ''
        }

    def open_gui(self):
        env = Environment(
            loader=FileSystemLoader('.'),
            autoescape=select_autoescape(['html', 'xml'])
        )
        templates = [env.get_template(file_name) for file_name in file_names]

        # Create the interface GUI window
        window1 = webview.create_window("HUD", html=templates[0].render(), width=760, height=760, resizable=False, fullscreen=False)
        # Create the data GUI window
        window2 = webview.create_window("Sensors", html=templates[1].render(), width=350, height=850, resizable=False, fullscreen=False)
        # Create the control GUI window
        window3 = webview.create_window("Test Control", html=templates[2].render(), width=350, height=450, resizable=False, fullscreen=False)

        def on_closed():
            self.clwin = True
            
        windows = [window1, window2, window3]
        
        for window in windows:
            window.events.closed += on_closed
        
        
            
        def call_update_data(windows):
            number = 45
            try:
                while True:
                    time.sleep(1)
                    if number > 350:
                        number = 0
                    number += 5
                    StatusError = False
                    windows[0].evaluate_js('updateJS({})'.format(number))
                    windows[1].evaluate_js('updateVariables(' + str(self.data) + ')')
                    windows[2].evaluate_js('setStatus("test", {})'.format(str(StatusError).lower()))
                    print(f"Update")   
                    self.get_telemetry()
                    # Check if forms closed
                    if self.clwin:
                        for window in windows:
                            try:
                                window.destroy()
                            except:
                                pass
                        print("Window closed. Exiting program.")
                        self.master1.close()
                        break
            except webview.WebViewException:
                pass
        
        threading.Thread(target=lambda: call_update_data(windows)).start()

        def call_control(windows):
            number = 45
            try:
                while True:
                    time.sleep(1)
                    print("Control")
                    self.control_send()
                    if self.clwin:
                        print("Control connection closed")
                        self.master2.close()
                        break
            except webview.WebViewException:
                pass

        threading.Thread(target=lambda: call_control(windows)).start()
        
        # Define function for js
        def toggleGUI():
            print("Finally")
        def land():
            pass
        def takeoff(alt):
            print(alt)
        def setPosition(lat, lon, alt):
            print(f"{lat}, {lon}, {alt}")
        # Expose the toggleGUI function to the webview
        window2.expose(toggleGUI)
        window3.expose(land)
        window3.expose(takeoff)
        window3.expose(setPosition)
        #Start program
        webview.start(debug=True)
    def control_send(self):
        pass
    def get_telemetry(self):
        try:
            # Запит даних з GPS
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                self.data['gps_lat'] = msg.lat / 1e7
                self.data['gps_lon'] = msg.lon / 1e7
                self.data['gps_alt'] = msg.alt / 1e3

            # Запит даних з сенсору батареї
            msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True)
            if msg:
                self.data['battery_voltage'] = msg.voltages[0] / 1000
                self.data['battery_current'] = msg.current_battery / 100
                self.data['battery_remaining'] = msg.battery_remaining

            # Запит даних з IMU
            msg = self.master.recv_match(type='ATTITUDE', blocking=True)
            if msg:
                self.data['imu_roll'] = msg.roll
                self.data['imu_pitch'] = msg.pitch
                self.data['imu_yaw'] = msg.yaw

            # Запит даних з компасу
            msg = self.master.recv_match(type='VFR_HUD', blocking=True)
            if msg:
                self.data['compass_heading'] = msg.heading
                self.data['compass_groundspeed'] = msg.groundspeed

            # Запит даних з сенсору температури
            msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=True)
            if msg:
                self.data['temperature'] = msg.temperature / 100.0

        except Exception as e:
            print(f"Error: {e}")
            
    class DroneController:
        def __init__(self, master):
            self.master = master

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
                0, 0, 0, 0, 0, 0, 0, altitude
            )
            ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
            if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Takeoff command accepted")
            else:
                print(f"Takeoff command failed with result: {ack}")
                self.disarm()
                return
            self.wait_for_altitude(altitude)
            print("Takeoff complete")

        def land(self):
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
            else:
                print(f"Landing command failed with result: {ack}")
                return

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

if __name__ == '__main__':
    print('Starting...')
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
