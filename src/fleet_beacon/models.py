from datetime import datetime, timedelta, timezone

from pydantic import BaseModel
from pydantic.types import conint
from sqlalchemy import Column, DateTime, event

PrimaryKey = conint(gt=0, lt=2**31-1)

KST = timezone(timedelta(hours=9))


class TimeStampMixin:
    created_at = Column(DateTime, default=lambda: datetime.now(tz=KST))
    created_at._creation_order = 9998
    updated_at = Column(DateTime, default=lambda: datetime.now(tz=KST))
    updated_at._creation_order = 9998

    @staticmethod
    def _updated_at(mapper, connection, target):
        target.updated_at = datetime.now(tz=KST)

    @classmethod
    def __declare_last__(cls):
        event.listen(cls, "before_update", cls._updated_at)


# Pydantic models...
class FleetBeaconBase(BaseModel):
    class Config:
        orm_mode = True
        validate_assignment = True
        arbitrary_types_allows = True
        anystr_strip_whitespace = True

        json_encoders = {
            datetime: lambda v: v.strftime("%Y-%m-%dT%H:%M:%SZ") if v else None
        }
