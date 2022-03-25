from datetime import datetime
from typing import List, Optional

from sqlalchemy import Column, Boolean, Float, Integer, String, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.unit.enums import PX4Mode
from src.fleet_beacon.fleet.models import Fleet
from src.fleet_beacon.warehouse.models import Warehouse, WarehouseRead


class Unit(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    uuid = Column(String(36), nullable=False, unique=True)  # uuid.uuid1(): Produced on separate machines
    connected = Column(Boolean, default=False)
    armed = Column(Boolean, default=False)
    guided = Column(Boolean, default=False)
    mode = Column(String(12), default=str(PX4Mode.offboard))
    system_status = Column(Integer, default=0)
    latitude = Column(Float, default=0.0)
    longitude = Column(Float, default=0.0)
    altitude = Column(Float, default=0.0)
    yaw = Column(Float, default=0.0)
    fleet = Column(Integer, ForeignKey(Fleet.id), nullable=True)
    warehouse = Column(Integer, ForeignKey(Warehouse.id), nullable=False)


# Pydantic models...
class UnitBase(FleetBeaconBase):
    uuid: str
    warehouse: int


class UnitCreate(UnitBase):
    pass


class UnitRead(UnitBase):
    id: PrimaryKey
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = str(PX4Mode.offboard)
    system_status: int = 0
    latitude: float
    longitude: float
    altitude: float
    yaw: float
    fleet: Optional[int] = None
    created_at: datetime
    updated_at: datetime


class UnitList(FleetBeaconBase):
    total: int
    items: List[UnitRead] = []


class UnitWarehouseDetail(WarehouseRead):
    robots: UnitList


class UnitUpdate(UnitBase):
    id: PrimaryKey
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = str(PX4Mode.offboard)
    system_status: int = 0
    latitude: float
    longitude: float
    altitude: float
    yaw: float
    fleet: Optional[int] = None
