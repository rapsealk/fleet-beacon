from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from src.fleet_beacon.database import get_db
from src.fleet_beacon.models import PrimaryKey
from src.fleet_beacon.warehouse.models import WarehouseCreate, WarehouseRead, WarehouseList, WarehouseUpdate
from src.fleet_beacon.warehouse.service import create, delete, get, get_all, get_detail, update
from src.fleet_beacon.unit.models import UnitWarehouseDetail

router = APIRouter()


@router.post("", response_model=WarehouseRead, status_code=status.HTTP_201_CREATED)
async def create_warehouse(*, db_session: Session = Depends(get_db), warehouse_in: WarehouseCreate):
    warehouse = await create(db_session=db_session, warehouse_in=warehouse_in)
    return warehouse


@router.get("/{warehouse_id}", response_model=WarehouseRead)
async def get_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey):
    if warehouse := await get(db_session=db_session, warehouse_id=warehouse_id):
        return warehouse
    return JSONResponse(status_code=status.HTTP_404_NOT_FOUND, content={"detail": "Not Found"})


@router.get("/{warehouse_id}/detail", response_model=UnitWarehouseDetail)
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
