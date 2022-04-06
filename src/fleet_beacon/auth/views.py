from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from starlette.requests import Request
from sqlalchemy.orm import Session

from fleet_beacon.config import SECRET_KEY
from fleet_beacon.database import get_db
from fleet_beacon.auth.models import UserCreate, UserRead
from fleet_beacon.auth.service import create_user, sign_in_with_email_and_password

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

router = APIRouter()


@router.post("", response_model=UserRead)
async def auth_with_email_and_password(
    request: Request,
    db_session: Session = Depends(get_db),
    form_data: OAuth2PasswordRequestForm = Depends(),
    secret: str = SECRET_KEY
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
    return user


@router.post("/signup", response_model=UserRead)
async def sign_up(user_in: UserCreate, db_session: Session = Depends(get_db), secret: str = SECRET_KEY):
    return await create_user(db_session=db_session, user_in=user_in, secret=secret)


@router.post("/token")
async def auth_with_token(
    token: str = Depends(oauth2_scheme),
    db_session: Session = Depends(get_db),
    secret: str = SECRET_KEY
):
    print(f"[POST /auth/token] token={token}")
    return {}
