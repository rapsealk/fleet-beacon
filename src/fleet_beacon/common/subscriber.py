import asyncio
from typing import Callable, List, Optional

import aioredis


class RedisSubscriber:
    def __init__(self, channels: Optional[List[str]] = None, address: str = "redis://localhost"):
        self._channels = channels or []
        self._address = address

    async def initialize(self):
        self._redis = await aioredis.from_url(self.address)
        self._pubsub = self._redis.pubsub()
        await self._pubsub.subscribe(*self.channels)

    async def subscribe(self, callback: Optional[Callable] = None):
        while True:
            try:
                if message := await self._pubsub.get_message(ignore_subscribe_messages=True, timeout=1.0):
                    if callback is not None:
                        await callback(message["data"])
            except asyncio.TimeoutError:
                pass
            except Exception:
                break

    async def subscribe_sync(self):
        while True:
            try:
                if message := await self._pubsub.get_message(ignore_subscribe_messages=True, timeout=1.0):
                    yield message["data"]
            except asyncio.TimeoutError:
                pass
            except Exception:
                break

    @property
    def address(self) -> str:
        return self._address

    @property
    def channels(self) -> List[str]:
        return self._channels
