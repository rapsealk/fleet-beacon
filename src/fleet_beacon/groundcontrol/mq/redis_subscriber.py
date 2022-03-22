import argparse

import redis


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=6379)
    parser.add_argument("--channel", type=str, default="b3676b08-fa53-4430-8361-a2e1e77ca3eb")
    return parser.parse_args()


def main():
    args = parse_args()

    r = redis.Redis(host=args.host, port=args.port, db=0)
    s = r.pubsub()

    s.subscribe(args.channel)

    while True:
        res = s.get_message(timeout=5)
        if res is not None:
            message_type = res.get('type', None)
            if message_type == "subscribe":
                pass
            elif message_type == "message":
                print(f"res: {type(res)} {res}")


if __name__ == "__main__":
    main()
