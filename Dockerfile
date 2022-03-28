FROM python:3.8-slim-buster

ENV PYTHONUNBUFFERED=1
ENV HOST=0.0.0.0
ENV PORT=8000

WORKDIR /app
COPY requirements.txt requirements.txt

RUN apt-get update

RUN pip install --upgrade pip
RUN pip install -r requirements.txt

COPY . .

CMD python main.py --host ${HOST} --port ${PORT}
