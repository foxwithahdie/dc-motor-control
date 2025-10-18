import socket
import time
import sys
import pandas as pd


def main() -> None:
    sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = input("Input IP Address: ")
    print(ip, end="")
    buffer: int = 1024
    port = 8182
    response: bytes = bytes()
    sock.settimeout(5)

    left_speed_data = []
    right_speed_data = []
    left_sensor_data = []
    right_sensor_data = []

    start_time = time.time()
    try:
        while (time.time() - start_time) < 30:
            sock.sendto("send".encode(), (ip, port))
            try:
                response, _ = sock.recvfrom(buffer)
                if len(response.decode()) > 1:
                    total_response = {data.split(" ")[0]: list(map(lambda val: int(val), data.split(" ")[1].strip("<>").split(","))) for data in response.decode().splitlines()}
                    left_speed_data += total_response['left_speed']
                    right_speed_data += total_response['right_speed']
                    left_sensor_data += total_response['left_sensor']
                    right_sensor_data += total_response['right_sensor']
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        sock.close()

    packed_format = {
        "left_speed_data": left_speed_data,
        "right_speed_data": right_speed_data,
        "left_sensor_data": left_sensor_data,
        "right_sensor_data": right_sensor_data,
    }

    pd.DataFrame(packed_format).to_csv("data/speed_and_sensor_data.csv", index=False)


if __name__ == "__main__":
    main()
