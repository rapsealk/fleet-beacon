from datetime import datetime, timedelta, timezone

from sqlalchemy.orm import Session

from src.fleet_beacon.models import PrimaryKey
from src.fleet_beacon.mission.models import Mission, MissionCreate, MissionRead, MissionList
from src.fleet_beacon.warehouse.models import WarehouseList
from src.fleet_beacon.waypoint.service import create as create_waypoint

KST = timezone(timedelta(hours=9))


async def create(*, db_session: Session, mission_in: MissionCreate) -> Mission:
    mission = Mission()
    db_session.add(mission)
    db_session.commit()

    waypoints = []
    for waypoint in mission_in.waypoints:
        waypoint.mission_id = mission.id
        print(f"Waypoint to be created: {waypoint}")
        waypoints.append(await create_waypoint(db_session=db_session, waypoint_in=waypoint))

    return MissionRead(id=mission.id, waypoints=[])


async def get_all(*, db_session: Session) -> MissionList:
    missions = db_session.query(Mission).all()
    return MissionList(total=len(missions), items=missions)


async def get(*, db_session: Session, mission_id: PrimaryKey) -> Mission:
    return db_session.query(Mission).filter(Mission.id == mission_id).first()


async def assign_mission(*, db_session: Session, mission_in: MissionCreate) -> Mission:
    pass


"""
async def find_near_warehouses(*, db_session: Session, mission_id: PrimaryKey) -> WarehouseList:
    if not (mission := await get(db_session=db_session, mission_id=mission_id)):
        return WarehouseList(total=0, items=[])
    if not mission.waypoints:
        return WarehouseList(total=0, items=[])
    warehouses = await filter_by_distance(
        db_session=db_session, latitude=mission.waypoints[0].latitude, longitude=mission.waypoints[0].longitude, distance_km=3)
    return WarehouseList(total=len(warehouses), items=warehouses)
"""


"""
async def get(*, db_session: Session, waypoint_id: int) -> Optional[Waypoint]:
    return db_session.query(Waypoint).filter(Waypoint.id == waypoint_id).first()


async def get_by_mission(*, db_session: Session, mission_id: int) -> List[Waypoint]:
    return db_session.query(Waypoint).filter(Waypoint.mission == mission_id).all()


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
"""
