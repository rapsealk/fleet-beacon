from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from src.fleet_beacon.robot.models import Robot, RobotCreate, RobotUpdate

KST = timezone(timedelta(hours=9))


async def create(*, db_session: Session, robot_in: RobotCreate) -> Robot:
    """Creates a new robot."""
    robot = Robot(
        uuid=robot_in.uuid,
        system_status=robot_in.system_status,
        latitude=robot_in.latitude,
        longitude=robot_in.longitude,
        altitude=robot_in.altitude,
        yaw=robot_in.yaw,
        warehouse=robot_in.warehouse
    )
    db_session.add(robot)
    db_session.commit()

    return robot


async def get(*, db_session: Session, robot_id: int) -> Optional[Robot]:
    return db_session.query(Robot).filter(Robot.id == robot_id).first()


async def get_all(*, db_session: Session) -> List[Robot]:
    return db_session.query(Robot).all()


async def get_by_uuid(*, db_session: Session, uuid: str) -> Optional[Robot]:
    return db_session.query(Robot).filter(Robot.uuid == uuid).first()


async def get_or_create(*, db_session: Session, robot_in: RobotCreate) -> Robot:
    if robot := await get_by_uuid(db_session=db_session, uuid=robot_in.uuid):
        return robot
    return await create(db_session=db_session, robot_in=robot_in)


async def update(*, db_session: Session, robot: Robot, robot_in: RobotUpdate) -> Robot:
    robot_data = robot.dict()
    update_data = robot_in.dict(skip_defaults=True, exclude={"created_at"})

    for field in robot_data:
        if field in update_data:
            setattr(robot, field, update_data[field])
    robot.updated_at = datetime.now(tz=KST)

    db_session.commit()

    return robot


async def delete(*, db_session: Session, robot_id: int):
    db_session.query(Robot).filter(Robot.id == robot_id).delete()
    db_session.commit()
