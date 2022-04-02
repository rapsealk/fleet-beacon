from typing import Optional

import jose
from jose import jwt
from sqlalchemy.orm import Session

from fleet_beacon.auth.models import User


async def sign_in_with_email_and_password(
    *,
    db_session: Session,
    email: str,
    password: str,
    secret: str = ""
) -> Optional[str]:
    if user := db_session.query(User).filter(User.email == email).first():
        if user.check_password(password):
            return await get_bearer_token(email=user.email, secret=secret)
    return None


async def get_bearer_token(*, email: str, secret: str) -> str:
    """사용자 정보가 담긴 `JWT` 토큰을 발급합니다.

    Args:
        :param str email: 사용자의 이메일 주소 (e.g. johndoe@domain.com)
        :param str secret: Authorization 토큰을 인코딩/디코딩 하는 데 사용하는 문자열

    Returns:
        :returns: str
    """
    return jwt.encode({"email": email}, key=secret, algorithm="HS256")


async def verify_bearer_token(*, token: str, secret: str) -> Optional[str]:
    try:
        payload = jwt.decode(token, key=secret, algorithms="HS256")
    except jose.exceptions.JWTError:
        return None
    return payload.get("email")
