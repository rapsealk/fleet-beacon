from sqlalchemy import Column, Boolean, Float, Integer, String, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.robot.enums import PX4RobotMode
from src.fleet_beacon.fleet.models import Fleet
from src.fleet_beacon.warehouse.models import Warehouse


class Robot(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    connected = Column(Boolean, default=False)
    armed = Column(Boolean, default=False)
    guided = Column(Boolean, default=False)
    mode = Column(String(12), default=PX4RobotMode.offboard)
    system_status = Column(Integer)
    latitude = Column(Float)
    longitude = Column(Float)
    altitude = Column(Float)
    yaw = Column(Float)
    fleet = Column(Integer, ForeignKey(Fleet.id), nullable=True)
    warehouse = Column(Integer, ForeignKey(Warehouse.id), nullable=False)


# Pydantic models...
class RobotBase(FleetBeaconBase):
    id: PrimaryKey
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = PX4RobotMode.offboard
    system_status: int
    latitude: float
    longitude: float
    altitude: float
    yaw: float
    warehouse: int


class RobotCreate(RobotBase):
    pass


class RobotUpdate(RobotBase):
    pass
