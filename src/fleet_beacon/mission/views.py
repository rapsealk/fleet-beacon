from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from fleet_beacon.database import get_db
from fleet_beacon.models import PrimaryKey
from fleet_beacon.mission.models import MissionCreate, MissionRead, MissionList
from fleet_beacon.mission.service import create, get_all
from fleet_beacon.warehouse.models import WarehouseList
from fleet_beacon.warehouse.service import find_nearby_warehouses

router = APIRouter()


@router.post("", response_model=MissionRead, status_code=status.HTTP_201_CREATED)
async def create_warehouse(*, db_session: Session = Depends(get_db), mission_in: MissionCreate):
    mission = await create(db_session=db_session, mission_in=mission_in)
    return mission


@router.get("", response_model=MissionList)
async def get_missions(*, db_session: Session = Depends(get_db)):
    return await get_all(db_session=db_session)


@router.get("/{mission_id}/nearby/warehouse", response_model=WarehouseList)
async def get_nearby_warehouses(*, mission_id: PrimaryKey, db_session: Session = Depends(get_db)):
    warehouses = await find_nearby_warehouses(db_session=db_session, mission_id=mission_id, distance_km=3.0)
    return WarehouseList(total=len(warehouses), items=warehouses)


#@router.get("/{mission_id}/nearby/fleet", response_model=)


"""
@router.get("/{warehouse_id}", response_model=WarehouseRead)
async def get_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey):
    if warehouse := await get(db_session=db_session, warehouse_id=warehouse_id):
        return warehouse
    return JSONResponse(status_code=status.HTTP_404_NOT_FOUND, content={"detail": "Not Found"})


@router.get("/{warehouse_id}/detail", response_model=RobotWarehouseDetail)
async def get_warehouse_detail(*, warehouse_id: PrimaryKey, db_session: Session = Depends(get_db)):
    if warehouse := await get_detail(db_session=db_session, warehouse_id=warehouse_id):
        return warehouse
    raise HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=[{"msg": f"The warehouse with this id({warehouse_id}) does not exists."}]
    )


@router.get("", response_model=WarehouseList)
async def get_warehouses(*, db_session: Session = Depends(get_db)):
    warehouses = await get_all(db_session=db_session)
    return WarehouseList(total=len(warehouses), items=warehouses)


@router.put("/{warehouse_id}", response_model=WarehouseRead)
async def update_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey, warehouse_in: WarehouseUpdate):
    if not (warehouse := await get(db_session=db_session, warehouse_id=warehouse_id)):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=[{"msg": f"The warehouse with this id({warehouse_id}) does not exists."}]
        )
    warehouse = await update(db_session=db_session, warehouse=warehouse, warehouse_in=warehouse_in)
    return warehouse


@router.delete("/{warehouse_id}")
async def delete_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey):
    await delete(db_session=db_session, warehouse_id=warehouse_id)
    return JSONResponse({"detail": "OK"})
"""
