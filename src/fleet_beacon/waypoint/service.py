from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from fleet_beacon.waypoint.models import Waypoint
from fleet_beacon.waypoint.models import WaypointCreate, WaypointUpdate

KST = timezone(timedelta(hours=9))


async def create(*, db_session: Session, waypoint_in: WaypointCreate) -> Waypoint:
    waypoint = Waypoint(**waypoint_in.dict())

    db_session.add(waypoint)
    db_session.commit()

    return waypoint


async def get(*, db_session: Session, waypoint_id: int) -> Optional[Waypoint]:
    return db_session.query(Waypoint).filter(Waypoint.id == waypoint_id).first()


async def get_by_mission(*, db_session: Session, mission_id: int) -> List[Waypoint]:
    return db_session.query(Waypoint).filter(Waypoint.mission_id == mission_id).all()


async def update(*, db_session: Session, waypoint: Waypoint, waypoint_in: WaypointUpdate) -> Waypoint:
    waypoint_data = waypoint.dict()
    update_data = waypoint_in.dict(skip_defaults=True, exclude={"created_at"})

    for field in waypoint_data:
        if field in update_data:
            setattr(waypoint, field, update_data[field])
    waypoint.updated_at = datetime.now(tz=KST)

    db_session.commit()

    return waypoint


async def delete(*, db_session: Session, waypoint_id: int):
    db_session.query(Waypoint).filter(Waypoint.id == waypoint_id).delete()
    db_session.commit()
