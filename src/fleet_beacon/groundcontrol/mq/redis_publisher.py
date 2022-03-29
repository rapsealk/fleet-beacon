import argparse
import json
import random
import time

import redis

UUID = "6f62b1c5-833e-41c8-b7ca-bd632766a9dc"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=6379)
    parser.add_argument("--channel", type=str, default="b3676b08-fa53-4430-8361-a2e1e77ca3eb")
    return parser.parse_args()


def main():
    args = parse_args()

    r = redis.Redis(host=args.host, port=args.port)

    while True:
        message = {
            "timestamp": int(time.time() * 1000),
            "uuid": UUID,
            "global_position": {
                "latitude": 37.60283752264964 + random.random() / 10,
                "longitude": 126.86838656169049 + random.random() / 10,
                "altitude": 0
            }
        }
        r.publish(channel=args.channel, message=json.dumps(message))
        print(f"[Redis] Message published: {message}")
        time.sleep(3)


if __name__ == "__main__":
    main()
