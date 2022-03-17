from fastapi import APIRouter

from src.fleet_beacon.robot.views import router as robot_router
from src.fleet_beacon.warehouse.views import router as warehouse_router

router = APIRouter()

router.include_router(
    robot_router, prefix="/robot", tags=["robot"]
)
router.include_router(
    warehouse_router, prefix="/warehouse", tags=["warehouse"]
)
