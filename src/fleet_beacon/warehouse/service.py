from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from src.fleet_beacon.warehouse.models import Warehouse, WarehouseCreate, WarehouseUpdate

KST = timezone(timedelta(hours=9))


def create(*, db_session: Session, warehouse_in: WarehouseCreate) -> Warehouse:
    """Creates a new warehouse."""
    warehouse = Warehouse(
        name=warehouse_in.name,
        latitude=warehouse_in.latitude,
        longitude=warehouse_in.longitude
    )
    db_session.add(warehouse)
    db_session.commit()

    return warehouse


def get(*, db_session: Session, warehouse_id: int) -> Optional[Warehouse]:
    return db_session.query(Warehouse).filter(Warehouse.id == warehouse_id).first()


def get_all(*, db_session: Session) -> List[Warehouse]:
    return db_session.query(Warehouse).all()


def update(*, db_session: Session, warehouse: Warehouse, warehouse_in: WarehouseUpdate) -> Warehouse:
    warehouse_data = warehouse.dict()
    update_data = warehouse_in.dict(skip_defaults=True, exclude={"created_at"})

    for field in warehouse_data:
        if field in update_data:
            setattr(warehouse, field, update_data[field])
    warehouse.updated_at = datetime.now(tz=KST)

    db_session.commit()

    return warehouse


def delete(*, db_session: Session, warehouse_id: int):
    """Deletes an existing warehouse."""
    db_session.query(Warehouse).filter(Warehouse.id == warehouse_id).delete()
    db_session.commit()
