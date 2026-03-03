# BetaSpray HTTP API

Connect to the `BetaSpray` Wi-Fi AP, then hit `http://192.168.4.1`.

---

## POST /route

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
