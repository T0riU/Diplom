import threading
import time
import webview
from jinja2 import Environment, FileSystemLoader, select_autoescape
from pymavlink import mavutil
import tkinter as tk
from tkinter import ttk
import queue
import ctypes

file_names = ['hud.html', 'data.html', 'control.html']
def check_mavlink_connection(ip, port, timeout=10):
    try:
        # Connect to the MAVLink device
        connection_string = '{}:{}'.format(ip, port)
        connection = mavutil.mavlink_connection(connection_string, baud=115200)
        # Wait for the heartbeat message to confirm the connection
        connection.wait_heartbeat(timeout=timeout)
        if(connection.target_system==0):
            raise Exception("Port is not working Or Drone not in Network")
        print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
        return connection
    except Exception as e:
        exception_form = ExceptionForm(f"Failed to connect to MAVLink device: {e}")
        print(f"Failed to connect to MAVLink device: {e}")
        return None
    
class ExceptionForm:

    def __init__(self, exception_message):
        self.root = tk.Tk()
        self.root.title("Exception")
        self.exception_label = tk.Label(self.root, text=exception_message, wraplength=300, justify='center')
        self.exception_label.pack(padx=10, pady=10)
        self.close_button = tk.Button(self.root, text="Close", command=self.root.destroy)
        self.close_button.pack(pady=10)

        self.root.mainloop()
class StatusManager:
    status_string = ''
    status_error = False

    @staticmethod
    def set_status(status_string, is_error=False):
        print("[Control] "+StatusManager.status_string)
        StatusManager.status_string = status_string
        StatusManager.status_error = is_error

    @staticmethod
    def get_status():
        return StatusManager.status_string, StatusManager.status_error
    
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
        connection = check_mavlink_connection(ip, port)
        if connection:
            self.master = connection
            self.connected = True
            self.root.destroy()
            self.open_telemetry_form()

    def open_telemetry_form(self):
        if self.connected:
            telemetry_gui = TelemetryGUI(master=self.master)
            telemetry_gui.open_gui()
            
class TelemetryGUI:
    def __init__(self, master):
        self.master = master
        self.DEBUG = False
        self.clwin = False
        self.command_queue = queue.Queue()
        
        self.control_data = {
            'takeoff_alt': '',
            'gps_lat': '',
            'gps_lon': '',
            'gps_alt': ''
        }
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
            'temperature': '',
            'time': '',
            'nav_pitch': '',
            'nav_roll': ''
        }

    def open_gui(self):
        def get_screen_dimensions():
            user32 = ctypes.windll.user32
            screen_width = user32.GetSystemMetrics(0)
            screen_height = user32.GetSystemMetrics(1)
            return screen_width, screen_height
        def create_window(title, url, position, window_width, window_height):
            screen_width, screen_height = get_screen_dimensions()

            if position == 'center':
                x = (screen_width - window_width) // 2
                y = (screen_height - window_height) // 2
            elif position == 'left':
                x = 0
                y = (screen_height - window_height) // 2
            elif position == 'right':
                x = screen_width - window_width
                y = (screen_height - window_height) // 2
            else:
                raise ValueError("Invalid position. Choose from 'center', 'left', or 'right'.")

            return webview.create_window(title, html=url, width=window_width, height=window_height, x=x, y=y, resizable=False, fullscreen=False)
        env = Environment(
            loader=FileSystemLoader('.'),
            autoescape=select_autoescape(['html', 'xml'])
        )
        templates = [env.get_template(file_name) for file_name in file_names]

        # Create the interface GUI window
        window1 = create_window("HUD", url=templates[0].render(),  position='center', window_width=760, window_height=760)
        # Create the data GUI window
        window2 = create_window("Sensors", url=templates[1].render(),  position='left', window_width=350, window_height=900)
        # Create the control GUI window
        window3 = create_window("Test Control", url=templates[2].render(),  position='right', window_width=350, window_height=450)
        def on_closed():
            self.clwin = True
            
        windows = [window1, window2, window3]
        
        for window in windows:
            window.events.closed += on_closed
        
        
        def call_control():
            try:
                drone_controller = DroneController(self.master)
                drone_controller.initialize()
                while True:
                    time.sleep(1)
                    print("Control Waiting...")
                    if self.clwin:
                        print("Control connection closed")
                        break
                    try:
                        command = self.command_queue.get(block=True) 
                        command_name = command
                        if command_name == "l":
                            drone_controller.land()
                        elif command_name == "t":
                            drone_controller.arm()
                            drone_controller.takeoff(self.control_data['takeoff_alt'])
                        elif command_name == "p":
                            drone_controller.set_position(self.control_data['gps_lat'], self.control_data['gps_lon'], self.control_data['gps_alt'])
                        elif command_name == "e": 
                            break
                    except Exception as e:
                        print(f"Error processing command: {e}")
                    
            except webview.WebViewException:
                pass

        threading.Thread(target=lambda: call_control()).start()
        def call_update_js(windows):
            try:
                firstGet = False
                # self.get_telemetry()
                
                while True:
                    time.sleep(0.1)
                    windows[0].evaluate_js('updateJS('+str(self.data)+')')
                    windows[1].evaluate_js('updateVariables(' + str(self.data) + ')')
                    status_text, status_error = StatusManager.get_status()
                    windows[2].evaluate_js( f"setStatus('{status_text}', {'true' if status_error else 'false'})")
                    # print(f"Update {str(self.data)}::::")   
                    self.get_telemetry()
                    if not firstGet:
                        windows[2].evaluate_js('setValues({}, {}, {})'.format(self.data['gps_lat'], self.data['gps_lon'], 15))
                        firstGet = True
                    
                    # Check if forms closed
                    if self.clwin:
                        self.command_queue.put('e')
                        for window in windows:
                            try:
                                window.destroy()
                            except:
                                pass
                        print("Window closed. Exiting program.")
                        self.master.close()
                        break

            except webview.WebViewException:
                pass

        threading.Thread(target=lambda: call_update_js(windows)).start()
        
        # Define function for js
        def land():
            self.command_queue.put('l')
        def takeoff(alt):
            self.control_data['takeoff_alt'] = int(alt)
            self.command_queue.put("t")
        def setPosition(lat, lon, alt):
            self.control_data['gps_lat'] = int(lat)
            self.control_data['gps_lon'] = int(lon)
            self.control_data['gps_alt'] = int(alt)
            self.command_queue.put("p") 
        # Expose the toggleGUI function to the webview
        window3.expose(land)
        window3.expose(takeoff)
        window3.expose(setPosition)
        #Start program
        webview.start(debug=self.DEBUG)
    def get_telemetry(self):
        try:
            # Запит даних з GPS
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,timeout=1)
            if msg:
                self.data['gps_lat'] = msg.lat / 1e7
                self.data['gps_lon'] = msg.lon / 1e7
                self.data['gps_alt'] = msg.relative_alt / 1000


            # Запит даних з сенсору батареї
            msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True,timeout=1)
            if msg:
                self.data['battery_voltage'] = msg.voltages[0] / 1000
                self.data['battery_current'] = msg.current_battery / 100
                self.data['battery_remaining'] = msg.battery_remaining

            # Запит даних з IMU
            msg = self.master.recv_match(type='ATTITUDE', blocking=True,timeout=1)
            if msg:
                self.data['imu_roll'] = msg.roll
                self.data['imu_pitch'] = msg.pitch
                self.data['imu_yaw'] = msg.yaw

            # Запит даних з компасу
            msg = self.master.recv_match(type='VFR_HUD', blocking=True,timeout=1)
            if msg:
                self.data['compass_heading'] = msg.heading
                self.data['compass_groundspeed'] = msg.groundspeed

            # Запит даних з сенсору температури
            msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=True,timeout=1)
            if msg:
                self.data['temperature'] = msg.temperature / 100.0
                
            msg = self.master.recv_match(type="SYSTEM_TIME", blocking=True, timeout=1)
            if msg:
                # форматUNIX timestamp
                self.data['time'] = time.strftime("%H:%M:%S", time.localtime( msg.time_unix_usec / 1e6))
            msg = self.master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True, timeout=1)
            if msg:
                self.data['nav_pitch'] = msg.nav_pitch
                self.data['nav_roll'] = msg.nav_roll
        except Exception as e:
            print(f"Error: {e}")
            
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
    #    self.change_mode("GUIDED")
       self.start = False
       
    def change_mode(self, mode, timeout=69, max_restarts=3):
        StatusManager.set_status(f"Changing mode to {mode}", False)
        if mode not in self.master.mode_mapping():
            StatusManager.set_status(f"Mode {mode} not supported", True)
            return False
        
        restart_count = 0
        while restart_count < max_restarts:
            mode_id = self.master.mode_mapping()[mode]
            self.master.set_mode(mode_id)
            start_time = time.time()
            while time.time() - start_time < timeout:
                ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.mode = mode
                        StatusManager.set_status(f"Mode changed to {mode}", False)
                        return True
                    else:
                        StatusManager.set_status(f"Failed to change mode to {mode} with result: {ack.result}", True)
                        break  # Break inner loop if failed result
            StatusManager.set_status(f"Timeout changing mode. Restart", True)
            restart_count += 1
            self.master.set_mode(mode_id)  # Attempt to set mode again
        StatusManager.set_status(f"Maximum restarts reached. Failed to change mode.", True)
        return False

    def arm(self, timeout=10):
        StatusManager.set_status(f"Arming the drone", False)
        if self.armed:
            StatusManager.set_status(f"\tDrone is already armed.", False)
            return
        # Check if the mode is armable
        if self.mode != "LAND":
            try:
                self.master.arducopter_arm()
                time.sleep(1)
                start_time = time.time()
                while not self.master.motors_armed():
                    if time.time() - start_time > timeout:
                        raise TimeoutError("Arming the drone timed out.", True)
                    time.sleep(0.1) 
                self.armed = True
                StatusManager.set_status(f"Drone is armed", False)
            except TimeoutError as te:
                StatusManager.set_status(f"\tTimeout error: {te}", True)
                self.armed = False
            except Exception as e:
                StatusManager.set_status(f"\tAn error occurred while trying to arm the drone: {e}", True)
                self.armed = False
        else:
            StatusManager.set_status(f"LAND mode is not armable. Cannot arm the drone.", True)

    def disarm(self):
        StatusManager.set_status(f"Disarming the drone", False)
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        self.armed = False
        StatusManager.set_status(f"Drone is disarmed", False)

    def takeoff(self, altitude):
        if not self.armed or self.mode != "GUIDED" or self.flying:
            StatusManager.set_status(f"Invalid conditions for takeoff.", True)
            return
        StatusManager.set_status(f"Taking off to altitude: {altitude} meters", False)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            StatusManager.set_status(f"Takeoff command accepted", False)
            self.flying =True
        else:
            StatusManager.set_status(f"Takeoff command failed with result: {ack}", True)
            return
        self.wait_for_altitude(altitude)
        StatusManager.set_status(f"Takeoff complete", False)

    def land(self):
        if (not self.flying or not self.armed) and not self.start:
            StatusManager.set_status(f"Invalid conditions for landing.", True)
            return

        StatusManager.set_status(f"Landing", False)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        ack = self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)
        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            StatusManager.set_status(f"Landing command accepted", False)
            self.flying = False
        else:
            StatusManager.set_status(f"Landing command failed with result: {ack}", True)
            return

        self.wait_for_altitude(0.5, compare_func=lambda alt, target: alt <= target)
        StatusManager.set_status(f"Landing complete", False)
        self.disarm()
        self.change_mode("GUIDED")

    def set_position(self, lat, lon, alt):
        if not self.armed or self.mode != "GUIDED" or not self.flying:
            StatusManager.set_status(f"Cannot set position while not flying.", True)
            return
        StatusManager.set_status(f"Setting position: Lat={lat}, Lon={lon}, Alt={alt}", False)
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
        StatusManager.set_status(f"Position set", False)
        #self.wait_for_position(lat, lon, alt)

    def wait_for_altitude(self, target_altitude, timeout=30, compare_func=lambda alt, target: alt >= target):
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            time.sleep(2)
            if msg:
                altitude = msg.relative_alt / 1000.0
                if compare_func(altitude, target_altitude):
                    return True
        StatusManager.set_status(f"Timeout waiting for altitude {target_altitude}", True)
        return False
    
    def wait_for_position(self, target_lat, target_lon, target_alt, timeout=60):
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            time.sleep(4)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                if (abs(lat - target_lat) < 0.00001 and
                    abs(lon - target_lon) < 0.00001 and
                    abs(alt - target_alt) < 0.5):
                    StatusManager.set_status(f"Arrived at target position", False)
                    return True
        StatusManager.set_status(f"Timeout waiting for position", True)
        return False
    
    def wait_for_ack(self, command):
       while True:
                msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
                if msg.command == command:
                    return msg.result 

if __name__ == '__main__':
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
