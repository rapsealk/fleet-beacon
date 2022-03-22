import argparse
import json
import socket
import struct
import time
from datetime import datetime

from socketx import socketx


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    while True:
        """ STEP 01 """
        packet_bytes = socketx.recvall(sock, bufsize=4)
        packet_bytes, *_ = struct.unpack(">I", packet_bytes)
        data = socketx.recvall(sock, bufsize=packet_bytes).decode("utf-8")
        print(f'[{datetime.now().isoformat()}] Client Received: {data}')

        """ STEP 02 """
        data = json.dumps({"message": "hello"}).encode("utf-8")
        packet_bytes = struct.pack(">I", len(data))
        # packet_bytes = socket.ntohl(packet_bytes)
        sock.sendall(packet_bytes)
        sock.sendall(data)
        print(f'[{datetime.now().isoformat()}] Client Sent: {data}')

        time.sleep(1)


if __name__ == "__main__":
    main()
