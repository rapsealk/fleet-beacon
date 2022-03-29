from datetime import datetime
from typing import List, Optional

from sqlalchemy import Column, Integer, String
from sqlalchemy.dialects.mysql import FLOAT
from sqlalchemy.orm import relationship

from fleet_beacon.database import Base
from fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin


class Warehouse(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    name = Column(String(16))
    latitude = Column(FLOAT(precision=32), nullable=False)
    longitude = Column(FLOAT(precision=32), nullable=False)
    units = relationship("Unit")
    fleets = relationship("Fleet")


# Pydantic models...
class WarehouseBase(FleetBeaconBase):
    name: Optional[str]
    latitude: float
    longitude: float


class WarehouseCreate(WarehouseBase):
    pass


class WarehouseRead(WarehouseBase):
    id: PrimaryKey
    created_at: datetime
    updated_at: datetime


class WarehouseList(FleetBeaconBase):
    total: int
    items: List[WarehouseRead] = []


class WarehouseUpdate(WarehouseBase):
    pass
