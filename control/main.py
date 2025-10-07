"""
Command window for the Arduino
"""

import socket
import sys


def main() -> None:
    """
    Main function.
    """
    sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = input("Input IP Address: ")
    print(ip, end="")
    buffer: int = 1024
    port = 8182
    response: bytes = bytes()
    sock.settimeout(5)

    try:
        while True:
            sock.sendto(input("Command: ").encode(), (ip, port))
    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
