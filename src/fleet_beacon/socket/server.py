import argparse
import json
import multiprocessing
import socket
import struct
from datetime import datetime

from socketx import socketx


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--backlog', type=int, default=1)
    return parser.parse_args()


def communicate(conn: socket.socket, address):
    print(f'[{datetime.now().isoformat()}] Connected to {conn.getsockname()}, {address}')
    while True:
        """ STEP 01 """
        data = json.dumps({"host": address[0], "port": address[1]}).encode("utf-8")
        packet_bytes = struct.pack(">I", len(data))
        # packet_bytes = socket.ntohl(packet_bytes)
        conn.sendall(packet_bytes)
        conn.sendall(data)
        print(f'[{datetime.now().isoformat()}] Server Sent: {data}')

        """ STEP 02 """
        packet_bytes = socketx.recvall(conn, bufsize=4)
        packet_bytes, *_ = struct.unpack(">I", packet_bytes)
        data = socketx.recvall(conn, bufsize=packet_bytes).decode("utf-8")
        print(f'[{datetime.now().isoformat()}] ({conn.getsockname()}) Server Received: {data}')

        # ConnectionResetError


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', args.port))
    sock.listen(args.backlog)

    pool = multiprocessing.Pool()
    while True:
        conn, address = sock.accept()
        pool.apply_async(communicate, (conn, address))


if __name__ == "__main__":
    main()
