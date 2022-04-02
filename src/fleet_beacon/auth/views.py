from fastapi import APIRouter, Depends
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm

from fleet_beacon.auth.service import sign_in_with_email_and_password

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

router = APIRouter()


@router.post("")
async def auth_with_email_and_password(
    form_data: OAuth2PasswordRequestForm = Depends()
    # secret: str = Depends(),
    # auth_service: ...
):
    token = await sign_in_with_email_and_password(email=form_data.username, password=form_data.password)
    return {"token": token}


@router.post("/token")
async def auth_with_token(
    token: str = Depends(oauth2_scheme)
    # secret: str = Depends(),
    # auth_service: ...
):
    pass
