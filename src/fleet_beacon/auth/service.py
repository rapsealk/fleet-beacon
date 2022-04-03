import secrets
import string
from typing import Optional

import bcrypt
import jose
from jose import jwt
from sqlalchemy.orm import Session

from fleet_beacon.auth.models import User, UserCreate


def generate_password():
    """Generates a reasonable password if none is provided."""
    alphanumeric = string.ascii_letters + string.digits
    while True:
        password = "".join(secrets.choice(alphanumeric) for i in range(10))
        if (
            any(c.islower() for c in password)
            and any(c.isupper() for c in password)  # noqa
            and sum(c.isdigit() for c in password) >= 3  # noqa
        ):
            break
    return password


def hash_password(password: str, salt: str = None):
    """Generates a hashed version of the provided password."""
    pw = bytes(password, "utf-8")
    salt = salt or bcrypt.gensalt()
    return bcrypt.hashpw(pw, salt)


async def create_user(*, db_session: Session, user_in: UserCreate, secret: Optional[str] = None) -> User:
    if user := await find_user(db_session=db_session, email=user_in.email):
        return user
    user = User(name=user_in.name, email=user_in.email, password=hash_password(user_in.password, salt=salt))
    db_session.add(user)
    db_session.commit()
    return user


async def find_user(*, db_session: Session, email: str) -> Optional[User]:
    return db_session.query(User).filter(User.email == email).first()


async def sign_in_with_email_and_password(
    *,
    db_session: Session,
    email: str,
    password: str,
    secret: str = ""
) -> Optional[User]:
    if user := await find_user(db_session=db_session, email=email):
        if user.check_password(password):
            return user
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
