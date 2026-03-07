# BetaSpray HTTP API

Connect to the `BetaSpray` Wi-Fi AP, then hit `http://192.168.4.1`.

---

## POST /capture

Trigger a camera capture and store the frame in memory.

```sh
curl -X POST http://192.168.4.1/capture
```

**Response:** `OK` (or `Camera disabled` if disabled)

---

## GET /get

Retrieve the last captured frame as a JPEG image.

```sh
curl http://192.168.4.1/get > capture.jpg
```

**Response:** JPEG binary data (image/jpeg) or "No frame captured"

---

## POST /configure

Configure camera settings: enable/disable, resolution, and format.

**Body:** JSON object with optional fields:
- `enabled` (boolean) — Initialize/deinitialize the camera. When `false`, the camera is deinitialized and resources are released (useful before writing to FatFS).
- `resolution` (string) — `QVGA`, `VGA`, or `SVGA` (requires camera to be initialized)
- `format` (string) — `JPEG`, `RGB565`, or `GRAYSCALE` (requires camera to be initialized)

```sh
# Disable camera (releases resources for FatFS writes)
curl -X POST http://192.168.4.1/configure \
  -H "Content-Type: application/json" \
  -d '{"enabled": false}'

# Enable camera and set configuration
curl -X POST http://192.168.4.1/configure \
  -H "Content-Type: application/json" \
  -d '{"enabled": true, "resolution": "QVGA", "format": "JPEG"}'
```

**Response:** `OK`

---

## POST /servo

Command an arbitrary servo to move to a specified angle.

**Body:** JSON object:
- `servo` (int) — Servo ID (0 to NUM_SERVOS-1)
- `angle` (int) — Target angle in degrees (0-180)

```sh
curl -X POST http://192.168.4.1/servo \
  -H "Content-Type: application/json" \
  -d '{"servo":0,"angle":90}'
```

**Response:** `OK`

---

## Route Management

### POST /route/create

Create a route file on FatFS with a list of (x,y) hold coordinates.

**Body:** JSON object:
- `route` (int) — Route number (0–99)
- `holds` (array of arrays) — List of [x, y] pixel coordinates

**File format:** Binary packed floats (4-byte header + 8 bytes per hold)
- Bytes 0–3: `uint32` hold count
- Bytes 4+: alternating `float32` x, y coordinates

```sh
curl -X POST http://192.168.4.1/route/create \
  -H "Content-Type: application/json" \
  -d '{"route":1,"holds":[[160,120],[80,60],[240,180]]}'
```

**Response:** `OK`

---

### POST /route/delete

Delete a route file from FatFS.

**Body:** JSON object:
- `route` (int) — Route number to delete

```sh
curl -X POST http://192.168.4.1/route/delete \
  -H "Content-Type: application/json" \
  -d '{"route":1}'
```

**Response:** `OK`

---

### POST /route/load

Load a route from FatFS into RAM. Parses CSV and prepares for playback.

**Body:** JSON object:
- `route` (int) — Route number to load

```sh
curl -X POST http://192.168.4.1/route/load \
  -H "Content-Type: application/json" \
  -d '{"route":1}'
```

**Response:** `OK`

---

### POST /route/play

Load a route and start playback. Servos begin stepping through holds.

**Body:** JSON object:
- `route` (int) — Route number to load and play

```sh
curl -X POST http://192.168.4.1/route/play \
  -H "Content-Type: application/json" \
  -d '{"route":1}'
```

**Response:** `OK` (or error message if route fails to load)

---

### GET /route/pause

Pause playback. Servos hold their current position.

```sh
curl http://192.168.4.1/route/pause
```

**Response:** `OK`

---

### GET /route/next

Advance to the next hold in the route (manual step).

```sh
curl http://192.168.4.1/route/next
```

**Response:** `OK`

---

### POST /route/restart

Restart the loaded route from the first hold.

**Body:** Empty or `{}`

```sh
curl -X POST http://192.168.4.1/route/restart
```

**Response:** `OK`

---

## Deprecated Endpoints

### POST /route

Send hold position data for a climbing route.

**Body:** raw string (route/hold data)

```sh
curl -X POST http://192.168.4.1/route -d '{"holds":[{"x":0.5,"y":0.8},{"x":0.2,"y":0.4}]}'
```

**Response:** `OK`

---

## GET /start

Begin projecting the loaded route.

```sh
curl http://192.168.4.1/start
```

**Response:** `OK`

---

## GET /stop

Stop projection.

```sh
curl http://192.168.4.1/stop
```

**Response:** `OK`

---

## POST /test

Echo back the request body prefixed with `ECHO: `. Useful for verifying the server is reachable.

```sh
curl -X POST http://192.168.4.1/test -d "hello world"
```

**Response:** `ECHO: hello world`
