from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from fleet_beacon.unit.models import Unit, UnitCreate, UnitList, UnitUpdate

KST = timezone(timedelta(hours=9))


async def create(*, db_session: Session, unit_in: UnitCreate) -> Unit:
    """Creates a new unit."""
    unit = Unit(**unit_in.dict(skip_defaults=True))
    db_session.add(unit)
    db_session.commit()

    return unit


async def get(*, db_session: Session, unit_id: int) -> Optional[Unit]:
    return db_session.query(Unit).filter(Unit.id == unit_id).first()


async def get_all(*, db_session: Session) -> List[Unit]:
    return db_session.query(Unit).all()


async def get_filtered_all(
    *,
    warehouse_id: Optional[int] = None,
    fleet_id: Optional[int] = None,
    db_session: Session
) -> List[Unit]:
    query = db_session.query(Unit)
    if warehouse_id is not None:
        query = query.filter(Unit.warehouse_id == warehouse_id)
    if fleet_id is not None:
        query = query.filter(Unit.fleet_id == fleet_id)
    return query.all()


async def get_by_uuid(*, db_session: Session, uuid: str) -> Optional[Unit]:
    return db_session.query(Unit).filter(Unit.uuid == uuid).first()


async def get_by_warehouse(*, db_session: Session, warehouse_id: int, page: Optional[int] = None) -> UnitList:
    _items_per_page = 10
    query = db_session.query(Unit).filter(Unit.warehouse_id == warehouse_id)
    if page:
        query = query.offset(max(0, page-1) * _items_per_page)
    units = query.limit(_items_per_page).all()
    return UnitList(total=len(units), items=units)


async def get_or_create(*, db_session: Session, unit_in: UnitCreate) -> Unit:
    if unit := await get_by_uuid(db_session=db_session, uuid=unit_in.uuid):
        return unit
    return await create(db_session=db_session, unit_in=unit_in)


async def update(*, db_session: Session, unit: Unit, unit_in: UnitUpdate) -> Unit:
    unit_data = unit.dict()
    update_data = unit_in.dict(skip_defaults=True, exclude={"created_at"})

    for field in unit_data:
        if field in update_data:
            setattr(unit, field, update_data[field])
    unit.updated_at = datetime.now(tz=KST)

    db_session.commit()

    return unit


async def delete(*, db_session: Session, unit_id: int):
    db_session.query(Unit).filter(Unit.id == unit_id).delete()
    db_session.commit()
