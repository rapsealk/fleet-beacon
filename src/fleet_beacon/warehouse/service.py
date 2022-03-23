from datetime import datetime, timedelta, timezone
from typing import List, Optional

from sqlalchemy.orm import Session

from src.fleet_beacon.warehouse.models import Warehouse, WarehouseCreate, WarehouseUpdate
from src.fleet_beacon.robot.models import RobotWarehouseDetail
from src.fleet_beacon.robot.service import get_by_warehouse

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


async def get_detail(*, db_session: Session, warehouse_id: int) -> Optional[RobotWarehouseDetail]:
    if warehouse := db_session.query(Warehouse).filter(Warehouse.id == warehouse_id).first():
        robots = await get_by_warehouse(db_session=db_session, warehouse_id=warehouse_id)
        print(f'[Warehouse::get_detail] robots={robots}')
        return RobotWarehouseDetail(**warehouse.dict(), robots=robots)
    else:
        return None


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
