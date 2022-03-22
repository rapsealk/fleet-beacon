import os

from kafka import KafkaConsumer


if __name__ == "__main__":
    consumer = KafkaConsumer("topic")
    # consumer = KafkaConsumer("topic", group_id="group_id")
    for msg in consumer:
        print(f'[{os.getpid()}] Consumer: {msg}')
