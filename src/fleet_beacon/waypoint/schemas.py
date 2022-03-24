from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey


class WaypointBase(FleetBeaconBase):
    id: PrimaryKey


class WaypointCreate(WaypointBase):
    latitude: float
    longitude: float
    altitude: float
    mission: PrimaryKey


class WaypointUpdate(WaypointBase):
    latitude: float
    longitude: float
    altitude: float
    mission: PrimaryKey


class WaypointDelete(WaypointBase):
    pass
