from typing import Union

import redis


class Publisher:
    def __init__(self, host: str = "127.0.0.1", port: int = 6379):
        self._host = host
        self._port = port

    def initialize(self):
        self._redis = redis.Redis(host=self.host, port=self.port)

    def publish(self, channel: str, message: Union[bytes, str]):
        self._redis.publish(channel=channel, message=message)

    @property
    def host(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port
