import asyncio
import logging
from typing import Iterable

import aioredis
from aioredis.client import PubSub
from fastapi import FastAPI, WebSocket

# logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def get_redis_pool():
    return await aioredis.from_url("redis://localhost", encoding="utf-8", decode_responses=True)    # :6379


async def redis_connector(websocket: WebSocket, channels: Iterable[str]):
    async def producer_handler(pubsub: PubSub, ws: WebSocket, channels: Iterable[str]):
        for channel in channels:
            await pubsub.subscribe(channel)
        try:
            while True:
                if message := await pubsub.get_message(ignore_subscribe_messages=True, timeout=5.0):
                    print(f"[Redis::Producer] message={message}")
                    await ws.send_text(message.get("data"))
        except Exception as e:
            logger.error(e)

    conn = await get_redis_pool()
    pubsub = conn.pubsub()

    producer_task = producer_handler(pubsub=pubsub, ws=websocket, channels=channels)

    done, pending = await asyncio.wait(
        [producer_task], return_when=asyncio.FIRST_COMPLETED,
    )
    logger.debug(f"Done task: {done}")
    for task in pending:
        logger.debug(f"Canceling task: {task}")
        task.cancel()


def add_websocket_redis_bridge(
    app: FastAPI,
    *,
    topics: Iterable[str],
    redis_host: str = "localhost",
    redis_port: int = 6379,
    redis_db: int = 0
) -> FastAPI:
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        await redis_connector(websocket, channels=["global_position"])

    return app
