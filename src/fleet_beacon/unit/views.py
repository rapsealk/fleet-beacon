from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from fleet_beacon.database import get_db
from fleet_beacon.models import PrimaryKey
from fleet_beacon.unit.models import UnitCreate, UnitRead, UnitList, UnitUpdate
from fleet_beacon.unit.service import delete, get, get_all, get_filtered_all, get_or_create, update

router = APIRouter()


@router.post("", response_model=UnitRead, status_code=status.HTTP_201_CREATED)
async def create_unit(*, db_session: Session = Depends(get_db), unit_in: UnitCreate):
    unit = await get_or_create(db_session=db_session, unit_in=unit_in)
    return unit


@router.get("", response_model=UnitList)
async def get_units(*, db_session: Session = Depends(get_db)):
    units = await get_all(db_session=db_session)
    return UnitList(total=len(units), items=units)


@router.get("/filtered", response_model=UnitList)
async def get_filtered_units(*, fleet: Optional[PrimaryKey] = None, db_session: Session = Depends(get_db)):
    units = await get_filtered_all(db_session=db_session, fleet_id=fleet)
    return UnitList(total=len(units), items=units)


@router.get("/{robot_id}", response_model=UnitRead)
async def get_unit(*, db_session: Session = Depends(get_db), robot_id: PrimaryKey):
    if robot := await get(db_session=db_session, robot_id=robot_id):
        return robot
    return JSONResponse(status_code=status.HTTP_404_NOT_FOUND, content={"detail": "Not Found"})


@router.put("/{robot_id}", response_model=UnitRead)
async def update_robot(*, db_session: Session = Depends(get_db), robot_id: PrimaryKey, robot_in: UnitUpdate):
    if not (robot := await get(db_session=db_session, robot_id=robot_id)):
        raise HTTPException(
            status_cide=status.HTTP_404_NOT_FOUND,
            detail=[{"msg": f"The robot with this id({robot_id}) does not exists."}]
        )
    robot = await update(db_session=db_session, robot=robot, robot_in=robot_in)
    return robot


@router.delete("/{robot_id}")
async def delete_robot(*, db_session: Session = Depends(get_db), robot_id: PrimaryKey):
    await delete(db_session=db_session, robot_id=robot_id)
    return JSONResponse({"detail": "OK"})
