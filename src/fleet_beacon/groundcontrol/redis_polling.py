import argparse
import multiprocessing
import time
import uuid
from datetime import datetime

from redis import Redis


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=6379)
    return parser.parse_args()


def _polling(interval: float, key: str):
    args = parse_args()
    redis = Redis(host=args.host, port=args.port, db=0)
    while True:
        print(f"[{datetime.now().isoformat()}] {redis.get(key)}")
        time.sleep(interval)


def main():
    args = parse_args()

    redis = Redis(host=args.host, port=args.port, db=0)

    key = str(uuid.uuid4())
    process = multiprocessing.Process(target=_polling, args=(2.0, key), daemon=True)
    process.start()

    while True:
        redis.set(key, f"{{\"value\": {datetime.now().isoformat()}}}")
        time.sleep(2.0)


if __name__ == "__main__":
    main()
