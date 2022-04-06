import asyncio

import aioredis


@asyncio.coroutine
async def consume_heartbeat(host: str = "localhost", port: int = 6379, db: int = 0):
    conn = await aioredis.from_url(f"redis://{host}:{port}/{db}", encoding="utf-8", decode_responses=True)
    pubsub = conn.pubsub()

    async def consumer_handler(pubsub: aioredis.client.PubSub, channel: str):
        await pubsub.subscribe(channel)
        try:
            while True:
                if message := await pubsub.get_message(ignore_subscribe_messages=True):
                    print(f"[Redis::Heartbeat] message={message}")
                    # TODO
        except Exception as e:
            print(f"[Redis::Heartbeat] Exception: {e}")

    consumer_task = consumer_handler(pubsub=pubsub, channel="heartbeat")
    done, pending = await asyncio.wait(
        [consumer_task], return_when=asyncio.FIRST_COMPLETED
    )
    for task in pending:
        task.cancel()
