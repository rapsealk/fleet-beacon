from fastapi import APIRouter

from src.fleet_beacon.groundcontrol.views import router as groundcontrol_router

router = APIRouter()

router.include_router(
    groundcontrol_router, prefix="", tags=["groundcontrol"]
)
