import bcrypt
from sqlalchemy import Column, Integer, String, LargeBinary

from fleet_beacon.database import Base
from fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin


class User(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    name = Column(String(4), nullable=False)
    email = Column(String(64), unique=True)
    password = Column(LargeBinary, nullable=False)

    def check_password(self, password: str):
        return bcrypt.checkpw(password.encode("utf-8"), self.password)


# Pydantic models...
class UserBase(FleetBeaconBase):
    name: str
    email: str


class UserCreate(UserBase):
    password: str


class UserRead(UserBase):
    id: PrimaryKey
