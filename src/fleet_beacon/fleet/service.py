from typing import List, Optional

from sqlalchemy.orm import Session

from src.fleet_beacon.fleet.models import Fleet, FleetCreate
from src.fleet_beacon.unit.models import Unit, UnitUpdate
from src.fleet_beacon.unit.service import get as get_unit
from src.fleet_beacon.unit.service import update as update_unit


async def get(*, db_session, fleet_id: int) -> Optional[Fleet]:
    pass


async def get_all(*, db_session: Session) -> List[Fleet]:
    return db_session.query(Fleet).all()


async def get_member_units(*, db_session: Session, fleet_id: int) -> List[Unit]:
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
