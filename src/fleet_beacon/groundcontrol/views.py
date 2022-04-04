import os
import pathlib
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.requests import Request
from fastapi.responses import HTMLResponse
from fastapi.security import OAuth2PasswordRequestForm
from fastapi.templating import Jinja2Templates
from starlette.responses import RedirectResponse
from sqlalchemy.orm import Session

from fleet_beacon.config import KAKAO_MAP_APP_KEY, SECRET_KEY
from fleet_beacon.database import get_db
from fleet_beacon.models import PrimaryKey
from fleet_beacon.auth.models import UserRead
from fleet_beacon.auth.service import get_current_user, sign_in_with_email_and_password
from fleet_beacon.mission.service import get as get_mission

router = APIRouter()

templates = Jinja2Templates(directory="templates")
auth_templates = Jinja2Templates(directory=pathlib.Path("templates") / "auth")


@router.get("/signin", response_class=HTMLResponse)
async def get_signin_view(request: Request):
    return auth_templates.TemplateResponse("signin.html", context={
        "request": request
    })


@router.post("/signin", response_class=RedirectResponse)
async def signin(
    request: Request,
    form_data: OAuth2PasswordRequestForm = Depends(),
    secret: str = SECRET_KEY,
    db_session: Session = Depends(get_db)
):
    if not (user := await sign_in_with_email_and_password(
        db_session=db_session, email=form_data.username, password=form_data.password, secret=secret
    )):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=[{"msg": f"The user with this username({form_data.username}) or password does not exists."}]
        )
    user = UserRead(**user.dict())
    request.state.session["user"] = user.dict()
    return RedirectResponse("/", status_code=status.HTTP_302_FOUND)


@router.get("/", response_class=HTMLResponse)
def get_main_view(
    request: Request,
    current_user: Optional[UserRead] = Depends(get_current_user),
    db_session: Session = Depends(get_db)
):
    print(f"[{os.getpid()}::GET /] User: {current_user}")
    if not current_user:
        return RedirectResponse("/signin", status_code=status.HTTP_307_TEMPORARY_REDIRECT)
    return templates.TemplateResponse("index.html", context={
        "request": request,
        "kakao_map_app_key": KAKAO_MAP_APP_KEY,
        "current_user": current_user
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
