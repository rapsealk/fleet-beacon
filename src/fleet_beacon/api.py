from fastapi import APIRouter

from fleet_beacon.auth.views import router as auth_router
from fleet_beacon.fleet.views import router as fleet_router
from fleet_beacon.mission.views import router as mission_router
from fleet_beacon.unit.views import router as unit_router
from fleet_beacon.warehouse.views import router as warehouse_router

router = APIRouter(prefix="/api")

router.include_router(
    auth_router, prefix="/auth", tags=["auth"]
)
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
