from typing import Optional

from sqlalchemy import Column, Float, Integer, String

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin


class Warehouse(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    name = Column(String(16))
    latitude = Column(Float, nullable=False)
    longitude = Column(Float, nullable=False)


# Pydantic models...
class WarehouseBase(FleetBeaconBase):
    id: PrimaryKey
    name: Optional[str]
    latitude: float
    longitude: float


class WarehouseCreate(WarehouseBase):
    pass


class WarehouseUpdate(WarehouseBase):
    pass
