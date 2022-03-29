import json
import time

import redis
from sqlalchemy.orm import Session

from fleet_beacon.models import PrimaryKey
from fleet_beacon.fleet.service import get as get_fleet
from fleet_beacon.mission.service import get as get_mission


async def publish_fleet_mission_job(
    db_session: Session,
    fleet_id: PrimaryKey,
    mission_id: PrimaryKey,
    host: str = "127.0.0.1",
    port: int = 6379
):
    fleet = await get_fleet(db_session=db_session, fleet_id=fleet_id)
    mission = await get_mission(db_session=db_session, mission_id=mission_id)

    if not fleet or not mission:
        return

    uuids = [unit.uuid for unit in fleet.units]
    waypoints = [{
        "latitude": waypoint.latitude,
        "longitude": waypoint.longitude
    } for waypoint in mission.waypoints]

    r = redis.Redis(host=host, port=port)
    for uuid in uuids:
        message = json.dumps({
            "timestamp": int(time.time() * 1000),
            "uuid": uuid,
            "type": "mission",
            "waypoints": waypoints
        })
        r.publish(channel=uuid, message=message)
