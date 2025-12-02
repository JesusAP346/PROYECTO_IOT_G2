FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .

ENV INFLUX_HOST=influxdb
ENV INFLUX_PORT=8086
ENV INFLUX_DB=ruido_db

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9000"]
