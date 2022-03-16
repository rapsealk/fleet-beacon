from fastapi import APIRouter

from src.fleet_beacon.warehouse.views import router as warehouse_router

router = APIRouter()

router.include_router(
    warehouse_router, prefix="/warehouse", tags=["warehouse"]
)
