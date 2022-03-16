from sqlalchemy import Column, Integer, String, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.task.enums import TaskStatus
from src.fleet_beacon.mission.models import Mission


class Task(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    type = Column(String(7), nullable=False)
    mission = Column(Integer, ForeignKey(Mission.id, ondelete="CASCADE"), nullable=False)
    status = Column(String(12), default=TaskStatus.assigned)


# Pydantic models...
class TaskBase(FleetBeaconBase):
    id: PrimaryKey


class TaskCreate(TaskBase):
    type: str
    mission: PrimaryKey


class TaskUpdate(TaskBase):
    type: str
    mission: PrimaryKey
    status: str


class TaskDelete(TaskBase):
    pass
