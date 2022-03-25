from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from src.fleet_beacon.unit.models import Unit, UnitCreate, UnitList, UnitUpdate

KST = timezone(timedelta(hours=9))


async def create(*, db_session: Session, robot_in: UnitCreate) -> Unit:
    """Creates a new robot."""
    robot = Unit(**robot_in.dict(skip_defaults=True))
    db_session.add(robot)
    db_session.commit()

    return robot


async def get(*, db_session: Session, robot_id: int) -> Optional[Unit]:
    return db_session.query(Unit).filter(Unit.id == robot_id).first()


async def get_all(*, db_session: Session) -> List[Unit]:
    return db_session.query(Unit).all()


async def get_by_uuid(*, db_session: Session, uuid: str) -> Optional[Unit]:
    return db_session.query(Unit).filter(Unit.uuid == uuid).first()


async def get_by_warehouse(*, db_session: Session, warehouse_id: int) -> UnitList:
    robots = db_session.query(Unit).filter(Unit.warehouse == warehouse_id).all()
    return UnitList(total=len(robots), items=robots)


async def get_or_create(*, db_session: Session, robot_in: UnitCreate) -> Unit:
    if robot := await get_by_uuid(db_session=db_session, uuid=robot_in.uuid):
        return robot
    return await create(db_session=db_session, robot_in=robot_in)


async def update(*, db_session: Session, robot: Unit, robot_in: UnitUpdate) -> Unit:
    robot_data = robot.dict()
    update_data = robot_in.dict(skip_defaults=True, exclude={"created_at"})

    for field in robot_data:
        if field in update_data:
            setattr(robot, field, update_data[field])
    robot.updated_at = datetime.now(tz=KST)

    db_session.commit()

    return robot


async def delete(*, db_session: Session, robot_id: int):
    db_session.query(Unit).filter(Unit.id == robot_id).delete()
    db_session.commit()
