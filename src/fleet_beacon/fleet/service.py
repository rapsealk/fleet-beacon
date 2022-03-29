from typing import List, Optional

from sqlalchemy.orm import Session

from fleet_beacon.models import PrimaryKey
from fleet_beacon.fleet.models import Fleet, FleetCreate
from fleet_beacon.unit.models import Unit, UnitUpdate
from fleet_beacon.unit.service import get as get_unit
from fleet_beacon.unit.service import update as update_unit


async def get(*, db_session, fleet_id: int) -> Optional[Fleet]:
    return db_session.query(Fleet).filter(Fleet.id == fleet_id).first()


async def get_all(*, db_session: Session) -> List[Fleet]:
    return db_session.query(Fleet).all()


async def find_fleets_in_warehouse(*, db_session: Session, warehouse_id: PrimaryKey) -> List[Fleet]:
    return db_session.query(Fleet).filter(Fleet.warehouse_id == warehouse_id).all()


async def get_member_units(*, db_session: Session, fleet_id: PrimaryKey) -> List[Unit]:
    if fleet := db_session.query(Fleet).filter(Fleet.id == fleet_id).first():
        return fleet.units
    return []


async def create(*, db_session: Session, fleet_in: FleetCreate) -> Fleet:
    fleet = Fleet(warehouse_id=fleet_in.warehouse_id)
    db_session.add(fleet)
    db_session.commit()

    for unit_id in fleet_in.unit_ids:
        if unit := await get_unit(db_session=db_session, unit_id=unit_id):
            await update_unit(db_session=db_session, unit=unit, unit_in=UnitUpdate(fleet_id=fleet.id))

    return fleet


async def assign_mission(*, db_session: Session, fleet_id: PrimaryKey, mission_id: PrimaryKey) -> Optional[Fleet]:
    if not (fleet := await get(db_session=db_session, fleet_id=fleet_id)):
        return None
    fleet.mission_id = mission_id
    db_session.commit()
    return fleet
