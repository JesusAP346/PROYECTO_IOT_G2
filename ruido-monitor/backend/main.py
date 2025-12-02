from fastapi import FastAPI, Request
from fastapi.responses import FileResponse
from datetime import datetime
from influxdb import InfluxDBClient
import json, base64, os

# --------------------------
# CONFIG
# --------------------------

INFLUX_HOST = os.getenv("INFLUX_HOST", "influxdb")
INFLUX_PORT = int(os.getenv("INFLUX_PORT", "8086"))
INFLUX_DB   = os.getenv("INFLUX_DB", "ruido_db")

client = InfluxDBClient(host=INFLUX_HOST, port=INFLUX_PORT)
client.create_database(INFLUX_DB)
client.switch_database(INFLUX_DB)

app = FastAPI(title="Ruido API")

# --------------------------
# HELPERS
# --------------------------

def decode_b64(data):
    try:
        raw = base64.b64decode(data).decode("utf-8")
        return json.loads(raw)
    except:
        return {}

def extract_payload(event):
    """Obtiene el dict con los valores de ruido."""

    # prioridad: object
    if isinstance(event.get("object"), dict):
        return event["object"]

    # fallback: data base64
    if "data" in event:
        decoded = decode_b64(event["data"])
        if isinstance(decoded, dict):
            return decoded

    return {}

def extract_point(event):

    ts = event.get("time", datetime.utcnow().isoformat() + "Z")
    device = event.get("deviceInfo", {}).get("deviceName", "unknown")

    p = extract_payload(event)
    if not p:
        return None  # no hay info útil

    # soportar diferentes nombres
    max_db = p.get("max_db") or p.get("max")
    min_db = p.get("min_db") or p.get("min")
    avg_db = p.get("avg_db") or p.get("avg")

    # noise_db = avg_db si no hay noise directo
    noise_db = p.get("noise") or p.get("noise_db") or avg_db

    if noise_db is None:
        return None

    fields = {
        "noise_db": float(noise_db)
    }

    if max_db is not None: fields["max_db"] = float(max_db)
    if min_db is not None: fields["min_db"] = float(min_db)
    if avg_db is not None: fields["avg_db"] = float(avg_db)

    if "alert" in p:
        fields["alert"] = bool(p["alert"])

    if "quality" in p:
        fields["quality"] = str(p["quality"])

    sensor = p.get("sensor_type", "unknown")
    unit   = p.get("unit", "dB")

    return {
        "measurement": "noise_measurements",
        "tags": {
            "device": device,
            "sensor": sensor,
            "unit": unit,
        },
        "time": ts,
        "fields": fields
    }

# --------------------------
# ENDPOINTS
# --------------------------

@app.post("/chirpstack")
async def chirpstack_webhook(request: Request):
    data = await request.json()

    if isinstance(data, dict):
        data = [data]

    saved = []
    for event in data:
        point = extract_point(event)
        if point:
            client.write_points([point])
            saved.append(point)

    return {
        "status": "ok" if saved else "no-valid-fields",
        "saved": saved
    }

@app.get("/history")
def history(limit: int = 50):
    q = f'SELECT * FROM noise_measurements ORDER BY time DESC LIMIT {limit}'
    result = client.query(q)
    points = list(result.get_points())
    return {"count": len(points), "data": points}

@app.get("/latest")
def latest():
    q = 'SELECT last("noise_db") AS noise_db, last("max_db") AS max_db, last("avg_db") AS avg_db, last("min_db") AS min_db FROM noise_measurements'
    result = client.query(q)
    pts = list(result.get_points())
    return pts[0] if pts else {}

@app.get("/dashboard")
def dashboard():
    return FileResponse("app/static/historial_ruido.html")
