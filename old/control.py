from pymavlink import mavutil
import time

# Функція для підключення до дрону
def connect_to_drone(port='192.168.160.1:14550', baud=115201):
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
    return master

# Функція для відправки команди взльоту
def takeoff(master, altitude):
    # Відправляємо команду ARM
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Motors armed")

    # Відправляємо команду для взльоту
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude
    )
    print(f"Takeoff to {altitude} meters")

    # Чекаємо досягнення висоти
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Висота в метрах
            print(f"Current altitude: {current_altitude} meters")
            if current_altitude >= altitude * 0.95:
                print("Target altitude reached")
                break
        time.sleep(1)

# Функція для відправки команди польоту до вказаної точки
def goto_position(master, latitude, longitude, altitude):
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (not used)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # type_mask (only positions enabled)
        int(latitude * 1e7),  # lat (degrees * 1e7)
        int(longitude * 1e7),  # lon (degrees * 1e7)
        altitude,  # alt (meters)
        0, 0, 0,  # x, y, z velocity in m/s (not used)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    print(f"Flying to position: lat={latitude}, lon={longitude}, alt={altitude}")

    # Чекаємо досягнення позиції
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_latitude = msg.lat / 1e7
            current_longitude = msg.lon / 1e7
            current_altitude = msg.relative_alt / 1000.0
            print(f"Current position: lat={current_latitude}, lon={current_longitude}, alt={current_altitude}")
            if (abs(current_latitude - latitude) < 0.0001 and
                abs(current_longitude - longitude) < 0.0001 and
                abs(current_altitude - altitude) < 0.5):
                print("Target position reached")
                break
        time.sleep(1)

# Основна функція
def main():
    master = connect_to_drone()
    
    # Виконуємо взльот
    takeoff(master, altitude=10)

    # Летимо до заданої точки
    goto_position(master, latitude=47.397742, longitude=8.545594, altitude=10)

    # Примітка: використовуйте свої координати

if __name__ == "__main__":
    main()
