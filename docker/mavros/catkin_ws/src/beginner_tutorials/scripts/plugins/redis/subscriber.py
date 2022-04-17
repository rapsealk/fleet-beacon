import redis


class Subscriber:
    def __init__(self, host: str = "127.0.0.1", port: int = 6379, db: int = 0):
        self._host = host
        self._port = port
        self._db = db

    def initialize(self):
        self._redis = redis.Redis(host=self.host, port=self.port, db=self.db).pubsub()

    def subscribe(self, channel: str):
        self._redis.subscribe(channel)

    def get_message(self, timeout: float = 0.0):
        return self._redis.get_message(timeout=timeout)

    @property
    def db(self) -> int:
        return self._db

    @property
    def host(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port
