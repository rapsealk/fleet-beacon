import argparse

from redis import Redis


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=6379)
    parser.add_argument("--channel", type=str, default="heartbeat")
    return parser.parse_args()


def _handle_redis_message(message: dict, *, redis: Redis):
    channel = message.get("channel", b"").decode("utf-8")
    if channel == "heartbeat":
        _handle_heartbeat_message(message, redis=redis)


def _handle_heartbeat_message(message: dict, *, redis: Redis):
    data = message.get("data", b"{}").decode("utf-8")
    redis.set(data["uuid"], data["global_position"])


def main():
    args = parse_args()

    redis = Redis(host=args.host, port=args.port, db=0)
    s = redis.pubsub()

    s.subscribe(args.channel)

    while True:
        res = s.get_message(timeout=5)
        if res is not None:
            message_type = res.get('type', None)
            if message_type == "subscribe":
                pass
            elif message_type == "message":
                _handle_redis_message(res, redis=redis)


if __name__ == "__main__":
    main()
