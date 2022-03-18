from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from src.fleet_beacon.database import get_db
from src.fleet_beacon.models import PrimaryKey
from src.fleet_beacon.robot.models import RobotCreate, RobotRead, RobotList, RobotUpdate
from src.fleet_beacon.robot.service import delete, get, get_all, get_or_create, update

router = APIRouter()


@router.post("", response_model=RobotRead, status_code=status.HTTP_201_CREATED)
async def create_robot(*, db_session: Session = Depends(get_db), robot_in: RobotCreate):
    robot = await get_or_create(db_session=db_session, robot_in=robot_in)
    return robot


@router.get("/{robot_id}", response_model=RobotRead)
async def get_robot(*, db_session: Session = Depends(get_db), robot_id: PrimaryKey):
    if robot := await get(db_session=db_session, robot_id=robot_id):
        return robot
    return JSONResponse(status_code=status.HTTP_404_NOT_FOUND, content={"detail": "Not Found"})


@router.get("", response_model=RobotList)
async def get_robots(*, db_session: Session = Depends(get_db)):
    robots = await get_all(db_session=db_session)
    return RobotList(total=len(robots), items=robots)


@router.put("/{robot_id}", response_model=RobotRead)
async def update_robot(*, db_session: Session = Depends(get_db), robot_id: PrimaryKey, robot_in: RobotUpdate):
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