import time

from kafka import KafkaProducer


if __name__ == "__main__":
    producer = KafkaProducer(bootstrap_servers="localhost:9092")
    for _ in range(100):
        producer.send("topic", value=f"Index: {_}, Time: {time.time()}".encode("utf-8"))
        time.sleep(1)
