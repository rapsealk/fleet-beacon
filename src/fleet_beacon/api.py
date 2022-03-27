from fastapi import APIRouter

from src.fleet_beacon.fleet.views import router as fleet_router
from src.fleet_beacon.mission.views import router as mission_router
from src.fleet_beacon.unit.views import router as unit_router
from src.fleet_beacon.warehouse.views import router as warehouse_router

router = APIRouter(prefix="/api")

router.include_router(
    fleet_router, prefix="/fleet", tags=["fleet"]
)
router.include_router(
    mission_router, prefix="/mission", tags=["mission"]
)
router.include_router(
    unit_router, prefix="/unit", tags=["unit"]
)
router.include_router(
    warehouse_router, prefix="/warehouse", tags=["warehouse"]
)
