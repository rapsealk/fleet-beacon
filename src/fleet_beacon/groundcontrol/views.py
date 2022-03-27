from fastapi import APIRouter, Depends, HTTPException, Request, status
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from sqlalchemy.orm import Session

from src.fleet_beacon.config import KAKAO_MAP_APP_KEY
from src.fleet_beacon.database import get_db
from src.fleet_beacon.models import PrimaryKey
from src.fleet_beacon.mission.service import get as get_mission

router = APIRouter()

templates = Jinja2Templates(directory="templates")


@router.get("/", response_class=HTMLResponse)
def get_main_view(request: Request, db_session: Session = Depends(get_db)):
    return templates.TemplateResponse("index.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY
    })


@router.get("/warehouse", response_class=HTMLResponse)
def get_warehouse_view(request: Request, db_session: Session = Depends(get_db)):
    return templates.TemplateResponse("warehouse.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY
    })


@router.get("/warehouse/{warehouse_id}", response_class=HTMLResponse)
def get_warehouse_detail_view(request: Request, warehouse_id: PrimaryKey, db_session: Session = Depends(get_db)):
    return templates.TemplateResponse("warehouse_detail.html", context={
        "request": request,
        "warehouse_id": warehouse_id
    })


@router.get("/warehouse/{warehouse_id}/fleet/new", response_class=HTMLResponse)
def get_fleet_new_view(request: Request, warehouse_id: PrimaryKey, db_session: Session = Depends(get_db)):
    return templates.TemplateResponse("fleet_new.html", context={
        "request": request,
        "warehouse_id": warehouse_id
    })


@router.get("/fleet", response_class=HTMLResponse)
def get_fleet_view(request: Request, db_session: Session = Depends(get_db)):
    return templates.TemplateResponse("fleet.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY
    })


@router.get("/mission", response_class=HTMLResponse)
def get_mission_view(request: Request, db_session: Session = Depends(get_db)):
    return templates.TemplateResponse("mission.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY
    })


@router.get("/mission/new", response_class=HTMLResponse)
def get_mission_new_view(request: Request):
    return templates.TemplateResponse("mission_new.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY
    })


@router.get("/mission/assignment/{mission_id}")
async def get_mission_assignment_view(request: Request, mission_id: PrimaryKey, db_session: Session = Depends(get_db)):
    mission = await get_mission(db_session=db_session, mission_id=mission_id)
    return templates.TemplateResponse("mission_assignment.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY,
        "mission_id": mission_id,
        "waypoints": mission.waypoints,
        "enumerate": enumerate,
        "len": len
    })


"""
@router.post("", response_model=WarehouseRead, status_code=status.HTTP_201_CREATED)
def create_warehouse(*, db_session: Session = Depends(get_db), warehouse_in: WarehouseCreate):
    warehouse = create(db_session=db_session, warehouse_in=warehouse_in)
    return warehouse


@router.get("/{warehouse_id}", response_model=WarehouseRead)
def get_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey):
    if warehouse := get(db_session=db_session, warehouse_id=warehouse_id):
        return warehouse
    return JSONResponse(status_code=status.HTTP_404_NOT_FOUND, content={"detail": "Not Found"})


@router.get("", response_model=WarehouseList)
def get_warehouses(*, db_session: Session = Depends(get_db)):
    warehouses = get_all(db_session=db_session)
    return WarehouseList(total=len(warehouses), items=warehouses)


@router.put("/{warehouse_id}", response_model=WarehouseRead)
def update_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey, warehouse_in: WarehouseUpdate):
    if not (warehouse := get(db_session=db_session, warehouse_id=warehouse_id)):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=[{"msg": f"The warehouse with this id({warehouse_id}) does not exists."}]
        )
    warehouse = update(db_session=db_session, warehouse=warehouse, warehouse_in=warehouse_in)
    return warehouse


@router.delete("/{warehouse_id}")
def delete_warehouse(*, db_session: Session = Depends(get_db), warehouse_id: PrimaryKey):
    delete(db_session=db_session, warehouse_id=warehouse_id)
    return JSONResponse({"detail": "OK"})
"""
