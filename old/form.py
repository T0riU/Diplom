import tkinter as tk
from tkinter import ttk
from pymavlink import mavutil
import threading
import time
import webview
from jinja2 import Environment, FileSystemLoader, select_autoescape
file_name = 'hud.html'

def check_mavlink_connection(ip, port, timeout=10):
    try:
        # Connect to the MAVLink device
        connection_string = '{}:{}'.format(ip, port)
        connection = mavutil.mavlink_connection(connection_string, baud=115200)
        # Wait for the heartbeat message to confirm the connection
        connection.wait_heartbeat(timeout=timeout)
        if(connection.target_system==0):
            raise Exception("Port is not working")
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
            root = tk.Tk()
            telemetry_app = TelemetryData(root, self.master)
            root.mainloop()

class TelemetryData:

    def __init__(self, root, master):
        self.root = root
        self.master = master
        self.root.title("Telemetry Data")
        self.windows = []
        self.create_widgets()

        # Start telemetry update in a separate thread
        self.telemetry_thread = threading.Thread(target=self.update_telemetry)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()
        
    def start_webview(self):
        env = Environment(
            loader=FileSystemLoader('.'),
            autoescape=select_autoescape(['html', 'xml'])
        )
        template = env.get_template(file_name).render()  
        window = webview.create_window("HTML", html=template, width=750, height=750, resizable=False, fullscreen=False)
        self.windows.append(window)
    
    def on_webview_closed(self, window):
        if self.windows:
            if window in self.windows:
                self.windows.remove(window)
            if not self.windows:
                self.root.deiconify()

    def open_gui(self):
        self.start_webview()
        webview.start(lambda window: self.on_webview_closed(window))

        
    def start_webview_in_main_thread(self):
        self.root.after(0, self.start_webview)
        
    def update_telemetry(self):
        number = 45
        while True:
            time.sleep(1)
            if number > 350:
                number = 0
            number += 5
            for window in self.windows:
                try:
                    window.evaluate_js(f'updateJS({number})')
                except Exception:
                    pass 
            self.get_telemetry()
            time.sleep(1)
            
    def create_entry(self, label_text, row, column):
        label = ttk.Label(self.root, text=label_text)
        label.grid(row=row, column=column, padx=5, pady=5)
        entry = ttk.Entry(self.root)
        entry.grid(row=row, column=column + 1, padx=5, pady=5)
        return entry
            
    def create_widgets(self):
         # Create and place labels and entry widgets for GPS data
        ttk.Label(self.root, text="GPS Data").grid(row=0, column=0, padx=5, pady=5)
        self.gps_lat = self.create_entry("Latitude:", 1, 0)
        self.gps_lon = self.create_entry("Longitude:", 2, 0)
        self.gps_alt = self.create_entry("Altitude:", 3, 0)

        # Create and place labels and entry widgets for Battery data
        ttk.Label(self.root, text="Battery Data").grid(row=4, column=0, padx=5, pady=5)
        self.battery_voltage = self.create_entry("Voltage:", 5, 0)
        self.battery_current = self.create_entry("Current:", 6, 0)
        self.battery_remaining = self.create_entry("Remaining:", 7, 0)

        # Create and place labels and entry widgets for IMU data
        ttk.Label(self.root, text="IMU Data").grid(row=0, column=2, padx=5, pady=5)
        self.imu_roll = self.create_entry("Roll:", 1, 2)
        self.imu_pitch = self.create_entry("Pitch:", 2, 2)
        self.imu_yaw = self.create_entry("Yaw:", 3, 2)

        # Create and place labels and entry widgets for Compass data
        ttk.Label(self.root, text="Compass Data").grid(row=4, column=2, padx=5, pady=5)
        self.compass_heading = self.create_entry("Heading:", 5, 2)
        self.compass_groundspeed = self.create_entry("Ground Speed:", 6, 2)

        # Create and place labels and entry widgets for Temperature data
        ttk.Label(self.root, text="Temperature Data").grid(row=8, column=0, padx=5, pady=5)
        self.temperature = self.create_entry("Temperature:", 9, 0)

        # Add a button to open the GUI
        self.open_gui_button = ttk.Button(self.root, text="Open GUI", command=self.open_gui)
        self.open_gui_button.grid(row=10, column=0, columnspan=3, pady=10)
        
    def get_telemetry(self):
        try:
             # Запит даних з GPS
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                self.gps_lat.delete(0, tk.END)
                self.gps_lat.insert(0, f"{msg.lat / 1e7}")
                self.gps_lon.delete(0, tk.END)
                self.gps_lon.insert(0, f"{msg.lon / 1e7}")
                self.gps_alt.delete(0, tk.END)
                self.gps_alt.insert(0, f"{msg.alt / 1e3}")

            # Запит даних з сенсору батареї
            msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True)
            if msg:
                self.battery_voltage.delete(0, tk.END)
                self.battery_voltage.insert(0, f"{msg.voltages[0] / 1000}")
                self.battery_current.delete(0, tk.END)
                self.battery_current.insert(0, f"{msg.current_battery / 100}")
                self.battery_remaining.delete(0, tk.END)
                self.battery_remaining.insert(0, f"{msg.battery_remaining}")

            # Запит даних з IMU
            msg = self.master.recv_match(type='ATTITUDE', blocking=True)
            if msg:
                self.imu_roll.delete(0, tk.END)
                self.imu_roll.insert(0, f"{msg.roll}")
                self.imu_pitch.delete(0, tk.END)
                self.imu_pitch.insert(0, f"{msg.pitch}")
                self.imu_yaw.delete(0, tk.END)
                self.imu_yaw.insert(0, f"{msg.yaw}")

            # Запит даних з компасу
            msg = self.master.recv_match(type='VFR_HUD', blocking=True)
            if msg:
                self.compass_heading.delete(0, tk.END)
                self.compass_heading.insert(0, f"{msg.heading}")
                self.compass_groundspeed.delete(0, tk.END)
                self.compass_groundspeed.insert(0, f"{msg.groundspeed}")

            # Запит даних з сенсору температури
            msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=True)
            if msg:
                self.temperature.delete(0, tk.END)
                self.temperature.insert(0, f"{msg.temperature / 100.0}")

        except Exception as e:
            print(f"Error: {e}")


# Create and run the app
if __name__ == "__main__":
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
