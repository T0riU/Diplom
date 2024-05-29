import time
from pymavlink import mavutil

# Підключення до дрону через послідовний порт (або інший інтерфейс)
# Замініть 'COM3' на потрібний порт або IP-адресу для TCP/UDP підключення
connection_string = '192.168.160.1:14550'  # Порт по умолчанию для MAVLink
master = mavutil.mavlink_connection(connection_string, baud=115200)

# Очікуємо першого пакету Heartbeat для підтвердження з'єднання
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

def get_telemetry():
    try:
        # Запит даних з GPS
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            gps_data = {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.alt / 1e3,
                'relative_alt': msg.relative_alt / 1e3,
                'vx': msg.vx / 100,
                'vy': msg.vy / 100,
                'vz': msg.vz / 100
            }
            print(f"GPS Data: {gps_data}")

        # Запит даних з сенсору батареї
        msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
        if msg:
            battery_data = {
                'voltage': msg.voltages[0] / 1000,
                'current': msg.current_battery / 100,
                'remaining': msg.battery_remaining
            }
            print(f"Battery Data: {battery_data}")

        # Запит даних з IMU
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            imu_data = {
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'rollspeed': msg.rollspeed,
                'pitchspeed': msg.pitchspeed,
                'yawspeed': msg.yawspeed
            }
            print(f"IMU Data: {imu_data}")

        # Запит даних з компасу
        msg = master.recv_match(type='VFR_HUD', blocking=True)
        if msg:
            compass_data = {
                'heading': msg.heading,
                'groundspeed': msg.groundspeed,
                'airspeed': msg.airspeed,
                'climb': msg.climb
            }
            print(f"Compass Data: {compass_data}")

        # Запит даних з сенсору температури
        msg = master.recv_match(type='SCALED_PRESSURE', blocking=True)
        if msg:
            temperature_data = {
                'temperature': msg.temperature / 100.0  # Температура у градусах Цельсія
            }
            print(f"Temperature Data: {temperature_data}")

        # Запит даних з додаткових IMU
        msg = master.recv_match(type='SCALED_IMU2', blocking=True)
        if msg:
            imu2_data = {
                'xacc': msg.xacc,
                'yacc': msg.yacc,
                'zacc': msg.zacc,
                'xgyro': msg.xgyro,
                'ygyro': msg.ygyro,
                'zgyro': msg.zgyro,
                'xmag': msg.xmag,
                'ymag': msg.ymag,
                'zmag': msg.zmag
            }
            print(f"IMU2 Data: {imu2_data}")

        # Запит даних з гіроскопа
        msg = master.recv_match(type='RAW_IMU', blocking=True)
        if msg:
            gyro_data = {
                'xgyro': msg.xgyro,
                'ygyro': msg.ygyro,
                'zgyro': msg.zgyro
            }
            print(f"Gyro Data: {gyro_data}")

        # Запит даних з сенсору відстані
        # msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg and False:
            distance_data = {
                'min_distance': msg.min_distance,
                'max_distance': msg.max_distance,
                'current_distance': msg.current_distance,
                'type': msg.type,
                'id': msg.id,
                'orientation': msg.orientation,
                'covariance': msg.covariance
            }
            print(f"Distance Sensor Data: {distance_data}")

    except KeyboardInterrupt:
        print("Операцію перервано користувачем")

# Головна функція для періодичного опитування
def main():
    while True:
        get_telemetry()
        time.sleep(1)  # Опитування кожну секунду

if __name__ == "__main__":
    main()
