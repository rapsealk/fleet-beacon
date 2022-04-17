import asyncio
import json
from datetime import datetime

import aioredis
from sqlalchemy.orm import sessionmaker, scoped_session

from fleet_beacon import config
from fleet_beacon.database import Database
from fleet_beacon.models import KST
from fleet_beacon.unit.models import UnitCreate, UnitUpdate
from fleet_beacon.unit.service import create as create_unit
from fleet_beacon.unit.service import get_by_uuid as get_unit_by_uuid
from fleet_beacon.unit.service import update as update_unit


@asyncio.coroutine
async def consume_heartbeat(host: str = "localhost", port: int = 6379, db: int = 0):
    conn = await aioredis.from_url(f"redis://{host}:{port}/{db}", encoding="utf-8", decode_responses=True)
    pubsub = conn.pubsub()

    database = Database(config.SQLALCHEMY_DATABASE_URI)
    session = scoped_session(sessionmaker(bind=database.engine))

    async def consumer_handler(pubsub: aioredis.client.PubSub, channel: str):
        await pubsub.subscribe(channel)
        try:
            while True:
                if message := await pubsub.get_message(ignore_subscribe_messages=True, timeout=5.0):
                    # print(f"[Redis::Heartbeat] message={message} ({type(message)})")
                    data = json.loads(message["data"])
                    with session() as sess:
                        if not (unit := await get_unit_by_uuid(db_session=sess, uuid=data["uuid"])):
                            unit_in = UnitCreate(uuid=data["uuid"], warehouse_id=data["warehouse_id"])
                            unit = await create_unit(db_session=sess, unit_in=unit_in)
                        unit_in = UnitUpdate(connected=True, heartbeat=datetime.now(tz=KST))
                        await update_unit(db_session=sess, unit=unit, unit_in=unit_in)
        except Exception as e:
            print(f"[Redis::Heartbeat] Exception: {e}")

    consumer_task = consumer_handler(pubsub=pubsub, channel="heartbeat")
    done, pending = await asyncio.wait(
        [consumer_task], return_when=asyncio.FIRST_COMPLETED
    )
    for task in pending:
        task.cancel()
