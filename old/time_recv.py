from pymavlink import mavutil
import time


# Функція для підключення до дрона
def connect_to_drone(port="192.168.160.1:14550", baud=115200):
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print(
        "Heartbeat from system (system %u component %u)"
        % (master.target_system, master.target_component)
    )
    return master


# Функція для отримання часу з дрона
def get_drone_time(master):
    # Надсилаємо запит на отримання системного часу
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        1,  # Період передачі (1 Hz)
        1,  # Активувати потік даних
    )

    # Очікуємо повідомлення з часом
    while True:
        msg = master.recv_match(type="SYSTEM_TIME", blocking=True)
        if msg:
            # Повертаємо час у форматі UNIX timestamp
            return msg.time_unix_usec / 1e6


# Основна функція
def main():
    master = connect_to_drone()
    drone_time = get_drone_time(master)
    print("Drone time:", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(drone_time)))


if __name__ == "__main__":
    main()
