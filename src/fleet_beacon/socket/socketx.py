import socket


class socketx:
    @staticmethod
    def recvall(sock: socket.socket, bufsize: int) -> bytes:
        buffer = b''
        while len(buffer) < bufsize:
            chunk = sock.recv(bufsize - len(buffer))
            if not chunk:
                raise EOFError('CLOSE_WAIT: Chunk is None!')
            buffer += chunk
        return buffer
