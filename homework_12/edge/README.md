# Edge Docker Compose — Quick Start

## Prerequisites

- Docker & Docker Compose v2+
- QGroundControl (desktop or web)
- Network access between host machines (if running QGC remotely)

---

## 1. Start All Services

From the `homework_12` root directory:

```bash
# Step 1 — Start ArduRover SITL simulation
docker compose -f sim/compose.sitl.yml up -d --build

# Verify SITL is running
docker compose -f sim/compose.sitl.yml ps
docker exec fc_sim sh -lc 'tail -n 80 /tmp/Rover.log'
# Expected: UDP connection 127.0.0.1:14550 and 127.0.0.1:14551

# Step 2 — Start edge stack (auto_stub + c2_service)
cd edge
docker compose up -d --build

# Verify both services are healthy
docker compose ps
```

Expected results:
- `auto_stub` becomes **healthy**
- `c2_service` becomes **healthy** after first MAVLink HEARTBEAT from FC
- Logs available at `edge/logs/c2.log`

---

## 2. QGroundControl Connection

### Local QGC (same host)

Connect to UDP port **14550** — this is the SITL GCS output port.

### Remote QGC (custom IP on another host)

If QGroundControl runs on a different machine, configure the SITL container to broadcast on the correct network interface:

#### Option A: Set `GCS_HOST` to the QGC machine IP

```bash
# On the SITL host, find your network IP (e.g., 192.168.1.100)
export GCS_HOST="192.168.1.100"

cd ../sim
docker compose -f sim/compose.sitl.yml down
docker compose -f sim/compose.sitl.yml up -d --build
```

In QGroundControl:
1. Go to **Companion Computer** settings
2. Set GCS IP to the SITL host IP (`192.168.1.50` in this example)
3. Set port to `14550`
4. Connect

#### Option B: Publish SITL ports explicitly (Docker Desktop / macOS)

Replace `network_mode: host` in `sim/compose.sitl.yml` with:

```yaml
ports:
  - "14550:14550/udp"
  - "14561:14561/udp"
```

Then in QGroundControl on the remote host, set the companion computer IP to the SITL host's IP address.

#### Option C: Use `--network` alias for cross-host communication

If both hosts are on the same Docker network:

```bash
# Create a shared network
docker network create cpp-miltech-net

# Start SITL on the network
docker compose -f sim/compose.sitl.yml up -d --build
docker network connect cpp-miltech-net fc_sim

# In QGC, set companion computer IP to fc_sim (container name resolves via Docker DNS)
```

---

## 3. C2 Scenario in QGC

For the C2 state-machine scenario:

1. Open `qgc/lyman-patrol.plan` in QGC (**Plan > Open**)
2. Connect to the drone via UDP **14550**
3. Switch mode to **Guided**
4. Click **Arm**
5. **Do NOT click Start Mission** — waypoints flow through `auto_stub → c2_service → FC`

C2 reacts to FC state changes:

| QGC Action | C2 State | Behavior |
|---|---|---|
| Disarm | `DISARMED` | Blocks all waypoints |
| Arm + Guided | `ARMED_GUIDED` | Forwards waypoints to FC |
| Arm + Hold | `ARMED_HOLD` | Sends `fc.hold()`, blocks waypoints |
| Arm + Manual | `ARMED_MANUAL` | Blocks waypoints, allows manual control |

---

## 4. Useful Commands

```bash
# Follow C2 logs
docker compose logs -f c2_service

# Follow auto_stub logs
docker compose logs -f auto_stub

# Inspect C2 config inside container
docker compose exec c2_service cat /etc/c2/c2_config.json

# View C2 log file on host
cat ./logs/c2.log

# Restart auto_stub (resets waypoint timer)
docker compose restart auto_stub

# Check restart policy
docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "$(docker compose ps -q c2_service)"
```

---

## 5. Configuration

C2 reads its config from `config/c2_config.json`:

```json
{
  "fc_port": 14551
}
```

This port is where C2 connects to the FC (ArduRover SITL). The default matches the SITL UDP output port.

---

## 6. File Structure

```
edge/
├── README.md              ← you are here
├── docker-compose.yml     ← edge stack definition
├── config/
│   └── c2_config.json    ← C2 config (read-only mount)
├── c2/
│   ├── Dockerfile         ← c2_service image build
│   ├── CMakeLists.txt     ← CMake config
│   └── src/               ← C2 source code
└── logs/                  ← mounted from /var/log/c2 (created on first run)
```
