import requests
import json
import time

api_ips = [
    "10.159.82.199:8090",
    "10.13.102.226:8090",
    "100.90.101.119:8090" 
]

max_retries = 3
auth = ("", "") 

def try_get_status(ip):
    status_url = f"http://{ip}/api/main/status"
    for attempt in range(1, max_retries + 1):
        try:
            response = requests.get(status_url, auth=auth, timeout=3)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            print(f"[{ip}] Attempt {attempt} failed: {e}")
            time.sleep(0.5)
    return None

data = None
for ip in api_ips:
    data = try_get_status(ip)
    if data:
        print(f"Connected to Stellarium at {ip}")
        break
else:
    print("All IPs failed to connect.")
    exit(1)

location = data.get("location", {})
time_info = data.get("time", {})

altitude = location.get("altitude")
latitude = location.get("latitude")
longitude = location.get("longitude")

local_time = time_info.get("local")
utc_time = time_info.get("utc")
time_zone = time_info.get("timeZone")

print("--- Observer Info ---")
print(f"Latitude:  {latitude}")
print(f"Longitude: {longitude}")
print(f"Altitude:  {altitude}")
print(f"Local Time: {local_time}")
print(f"UTC Time:   {utc_time}")
print(f"Time Zone:  {time_zone}")

object_name = "Sun"
object_url = (
    f"http://{ip}/api/objects/info"
    f"?name={object_name}"
    f"&observer={longitude},{latitude},{altitude}"
    f"&date={utc_time}"
    f"&format=json"
)

try:
    obj_response = requests.get(object_url, auth=auth, timeout=3)
    obj_response.raise_for_status()
    obj_data = obj_response.json()

    sun_alt = obj_data.get("altitude")
    sun_az = obj_data.get("azimuth")
    above_horizon = obj_data.get("above-horizon")

    print("\n--- Sun Info ---")
    print(f"Above Horizon: {above_horizon}")
    print(f"Sun Altitude:  {sun_alt}")
    print(f"Sun Azimuth:   {sun_az}")

except requests.exceptions.RequestException as e:
    print(f"Error fetching Sun info: {e}")
