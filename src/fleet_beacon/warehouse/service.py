from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from fleet_beacon.warehouse.models import Warehouse, WarehouseCreate, WarehouseUpdate
from fleet_beacon.mission.service import get as get_mission
from fleet_beacon.unit.models import UnitWarehouseDetail
from fleet_beacon.unit.service import get_by_warehouse
from fleet_beacon.utils.geometry import LatLng, get_haversine_distance

KST = timezone(timedelta(hours=9))


async def create(*, db_session: Session, warehouse_in: WarehouseCreate) -> Warehouse:
    """Creates a new warehouse."""
    warehouse = Warehouse(
        name=warehouse_in.name,
        latitude=warehouse_in.latitude,
        longitude=warehouse_in.longitude
    )
    db_session.add(warehouse)
    db_session.commit()

    return warehouse


async def get(*, db_session: Session, warehouse_id: int) -> Optional[Warehouse]:
    return db_session.query(Warehouse).filter(Warehouse.id == warehouse_id).first()


async def get_all(*, db_session: Session) -> List[Warehouse]:
    return db_session.query(Warehouse).all()


async def get_detail(*, db_session: Session, warehouse_id: int) -> Optional[UnitWarehouseDetail]:
    if warehouse := db_session.query(Warehouse).filter(Warehouse.id == warehouse_id).first():
        units = await get_by_warehouse(db_session=db_session, warehouse_id=warehouse_id)
        return UnitWarehouseDetail(**warehouse.dict(), units=units)
    else:
        return None


async def find_nearby_warehouses(
    *,
    db_session: Session,
    mission_id: Optional[int] = None,
    distance_km: float = 3.0
) -> List[Warehouse]:
    warehouses = db_session.query(Warehouse).all()
    if mission_id and (mission := await get_mission(db_session=db_session, mission_id=mission_id)):
        latitude, longitude = (mission.waypoints[0].latitude, mission.waypoints[0].longitude)
        warehouses = list(filter(lambda x: get_haversine_distance(
            LatLng(latitude, longitude), LatLng(x.latitude, x.longitude)) <= distance_km, warehouses))
        warehouses.sort(key=lambda x: get_haversine_distance(LatLng(latitude, longitude), LatLng(x.latitude, x.longitude)))
    return warehouses


"""
async def get_all_nearby(*, db_session: Session, mission_id: Optional[int] = None, distance_km: float = 3.0) -> List[Warehouse]:
    warehouses = db_session.query(Warehouse).all()
    if mission_id and (mission := await get_mission(db_session=db_session, mission_id=mission_id)):
        latitude, longitude = (mission.waypoints[0].latitude, mission.waypoints[0].longitude)
        warehouses = list(filter(lambda x: get_haversine_distance(
            LatLng(latitude, longitude), LatLng(x.latitude, x.longitude)) <= distance_km, warehouses))
    return warehouses
"""


"""
async def filter_by_distance(
    *,
    db_session: Session,
    latitude: float,
    longitude: float,
    distance_km: float = 3
) -> WarehouseList:
    # TODO: MySQL Spatial Convenience Functions (https://dev.mysql.com/doc/refman/5.7/en/spatial-convenience-functions.html)
    warehouses = db_session.query(Warehouse).all()
    warehouses = list(filter(lambda x: get_haversine_distance(
        LatLng(latitude, longitude), LatLng(x.latitude, x.longitude)) <= distance_km, warehouses))
    return WarehouseList(total=len(warehouses), items=warehouses)
"""


async def update(*, db_session: Session, warehouse: Warehouse, warehouse_in: WarehouseUpdate) -> Warehouse:
    warehouse_data = warehouse.dict()
    update_data = warehouse_in.dict(skip_defaults=True, exclude={"created_at"})

    for field in warehouse_data:
        if field in update_data:
            setattr(warehouse, field, update_data[field])
    warehouse.updated_at = datetime.now(tz=KST)

    db_session.commit()

    return warehouse


async def delete(*, db_session: Session, warehouse_id: int):
    """Deletes an existing warehouse."""
    db_session.query(Warehouse).filter(Warehouse.id == warehouse_id).delete()
    db_session.commit()
