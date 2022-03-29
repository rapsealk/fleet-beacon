from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from fleet_beacon.database import get_db
from fleet_beacon.models import PrimaryKey
from fleet_beacon.fleet.models import FleetCreate, FleetRead, FleetList
from fleet_beacon.fleet.service import create, get, get_all, assign_mission
from fleet_beacon.fleet.tasks import publish_fleet_mission_job

router = APIRouter()


@router.post("", response_model=FleetRead, status_code=status.HTTP_201_CREATED)
async def create_fleet(*, db_session: Session = Depends(get_db), fleet_in: FleetCreate):
    return await create(db_session=db_session, fleet_in=fleet_in)


@router.get("/{fleet_id}", response_model=FleetRead)
async def get_fleet(*, db_session: Session = Depends(get_db), fleet_id: PrimaryKey):
    if fleet := await get(db_session=db_session, fleet_id=fleet_id):
        return fleet
    return JSONResponse(status_code=status.HTTP_404_NOT_FOUND, content={"detail": "Not Found"})


@router.get("", response_model=FleetList)
async def get_fleets(*, db_session: Session = Depends(get_db)):
    fleets = await get_all(db_session=db_session)
    return FleetList(total=len(fleets), items=fleets)


@router.put("/{fleet_id}/mission/{mission_id}", response_model=FleetRead)
async def assign_mission_to_fleet(
    background_tasks: BackgroundTasks,
    *,
    fleet_id: PrimaryKey,
    mission_id: PrimaryKey,
    db_session: Session = Depends(get_db)
):
    """특정 `Fleet`에 `Mission`을 할당합니다.

    Args:
        :param :py:class:`fastapi.BackgroundTasks` background_tasks: ...
        :param int fleet_id: ...
        :param int mission_id: ...
        :param :py:class:`sqlalchemy.orm.Session` db_session: ...

    Returns:
        :returns: :py:class:`fleet_beacon.fleet.models.FleetRead`
    """
    background_tasks.add_task(publish_fleet_mission_job, db_session=db_session, fleet_id=fleet_id, mission_id=mission_id)
    if not (fleet := await assign_mission(db_session=db_session, fleet_id=fleet_id, mission_id=mission_id)):
        raise HTTPException(
            status_cide=status.HTTP_404_NOT_FOUND,
            detail=[{"msg": f"The fleet with this id({fleet_id}) does not exists."}]
        )
    return fleet


"""
@router.get("", response_model=UnitList)
async def get_units(*, db_session: Session = Depends(get_db)):
    units = await get_all(db_session=db_session)
    print(f"[GET /api/unit] units={units}")
    for unit in units:
        print(f"[GET /api/unit] unit={unit}")
        print(f"[GET /api/unit] unit.dict()={unit.dict()}")
    return UnitList(total=len(units), items=units)
"""


"""
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
"""
