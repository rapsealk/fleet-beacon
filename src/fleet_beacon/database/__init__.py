import logging
import os
import re
from contextlib import contextmanager

from sqlalchemy import create_engine, orm, inspect
from sqlalchemy.ext.declarative import declarative_base, declared_attr
from sqlalchemy.orm import Session, sessionmaker

logger = logging.getLogger(__name__)


def resolve_table_name(name):
    names = re.split('(?=[A-Z])', name)
    return '_'.join([x.lower() for x in names if x])


class CustomBase:
    __repr_attrs__ = []
    __repr_max_length__ = 15

    @declared_attr
    def __tablename__(self):
        return resolve_table_name(self.__name__)

    def dict(self):
        return {c.name: getattr(self, c.name) for c in self.__table__.columns}

    @property
    def _id_str(self):
        if ids := inspect(self).identity:
            return '-'.join([str(x) for x in ids]) if len(ids) > 1 else str(ids[0])
        return 'None'

    @property
    def _repr_attrs_str(self):
        max_length = self.__repr_max_length__

        values = []
        single = len(self.__repr_attrs__) == 1
        for key in self.__repr_attrs__:
            if not hasattr(self, key):
                raise KeyError(f"{self.__class__} has incorrect attribute '{key}' in __repr_attrs__")
            value = getattr(self, key)
            wrap_in_quote = isinstance(value, str)

            value = str(value)
            if len(value) > max_length:
                value = value[:max_length] + '...'

            if wrap_in_quote:
                value = f"'{value}'"
            values.append(value if single else f'{key}:{value}')

        return ' '.join(values)

    def __repr__(self):
        # get id like '#123'
        id_str = ('#' + self._id_str) if self._id_str else ''
        # join class name, id and repr_attrs
        return f'<{self.__class__.__name__} {id_str}{" " + self._repr_attrs_str if self._repr_attrs_str else ""}>'


Base = declarative_base(cls=CustomBase)

class Database:
    def __init__(self, url: str = 'mysql://root:0000@localhost:3306/fleet_beacon'):
        self._engine = create_engine(url)
        self._session_factory = orm.scoped_session(
            sessionmaker(autocommit=False, autoflush=False, bind=self._engine))

    def init_database(self):
        from src.fleet_beacon.fleet.models import Fleet
        from src.fleet_beacon.mission.models import Mission
        from src.fleet_beacon.robot.models import Robot
        from src.fleet_beacon.task.models import Task
        from src.fleet_beacon.warehouse.models import Warehouse
        from src.fleet_beacon.waypoint.models import Waypoint

        Base.metadata.create_all(self._engine)

    @contextmanager
    def session(self):
        session: Session = self.session_factory()
        try:
            yield session
        except Exception as e:
            logger.exception(f'{e}\nSession rollback because of exception')
            raise
        finally:
            session.close()

    @property
    def session_factory(self) -> Session:
        return self._session_factory
