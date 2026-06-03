// =========================================================
// Weighted navigation for the 9x9 (treated as 11x11) arena
// Used Main and RobotBehaviour
// =========================================================

// =============================== PLEASE CONSIDER ================================
// x/y are 0 - 10. The playable 9x9 arena is x=1 - 9, y=1 - 9
// The border is wall except entrance (0,7) and exit (0,3)
// 21 bytes contain 81 packed two-bit cells for the inner 9x9 grid
// 0 = unknown, 1 = seeded, 2 = fertile, 3 = infertile
// All four states are traversable unless a wall or temporary obstacle blocks them
// Dijkstra is used because unknown cells and non-linefollowing cells cost more
// Explored cells and cells with y <= 4 are preferred though not required
// ================================================================================

#include "ArenaTypes.h" // header file

constexpr uint8_t ARENA_W = 11;
constexpr uint8_t ARENA_H = 11;
constexpr uint8_t ARENA_CELL_COUNT = ARENA_W * ARENA_H;
constexpr unsigned long ARENA_TEMP_OBSTACLE_UNKNOWN_MS = 20000UL;
constexpr unsigned long ARENA_TEMP_OBSTACLE_EXPLORED_MS = 5000UL;
constexpr unsigned long ARENA_GETMAP_INTERVAL_MS = 5000UL;
constexpr float ARENA_OBSTACLE_NEAR_MM = 180.0f;

// Path-cost tuning values.
constexpr uint16_t ARENA_COST_EXPLORED = 10;       // States 1, 2, 3
constexpr uint16_t ARENA_COST_UNKNOWN_EXTRA = 8;  // Added to state 0
constexpr uint16_t ARENA_COST_LOW_Y_EXTRA = 0;     // y <= 4 preferred
constexpr uint16_t ARENA_COST_HIGH_Y_EXTRA = 25;    // y > 4 less preferred
constexpr uint16_t ARENA_COST_TURN_EXTRA = 1;      // Tiny smoothing preference
constexpr uint16_t ARENA_INF_COST = 60000;         // Setting infinite cost

// For hard coding the map

const RFIDLocation RFID_LOCATIONS[] = {
  //{"UID", 1, 1}, ...
  {"", -1, -1}
};

constexpr uint8_t RFID_LOCATION_COUNT = sizeof(RFID_LOCATIONS) / sizeof(RFID_LOCATIONS[0]);

// Map state for every planner cell
// Border cells are not encoded by getMap, they are managed by arenaIsWallCell
uint8_t arena_cell_state[ARENA_H][ARENA_W];
unsigned long arena_temp_blocked_until[ARENA_H][ARENA_W];

ArenaPoint arena_pose = {-1, -1};
ArenaHeading arena_heading = ArenaHeading::East;
bool arena_have_pose = false;

unsigned long arena_last_getmap_ms = 0;

constexpr ArenaPoint ARENA_ENTRANCE = {9, 3};
constexpr ArenaPoint ARENA_EXIT     = {9, 7};

// Checks whether a coordinate is inside the 11x11 planner grid
bool arenaInside(int8_t x, int8_t y)
{
  return x >= 0 && x < ARENA_W && y >= 0 && y < ARENA_H;
}

//Returns true for the two border cells that are legal openings
bool arenaIsDoorway(int8_t x, int8_t y)
{
  return (x == ARENA_ENTRANCE.x && y == ARENA_ENTRANCE.y) ||
         (x == ARENA_EXIT.x && y == ARENA_EXIT.y);
}

// Treats the arena border as wall except the two doorways
bool arenaIsWallCell(int8_t x, int8_t y)
{
  if (!arenaInside(x, y)) {
    return true;
  }

  const bool perimeter = (x == 0 || x == ARENA_W - 1 || y == 0 || y == ARENA_H - 1);
  return perimeter && !arenaIsDoorway(x, y);
}

// Resets the planner grid while keeping doorways free
void arenaClearMapToUnknown()
{
  for (uint8_t y = 0; y < ARENA_H; ++y) {
    for (uint8_t x = 0; x < ARENA_W; ++x) {
      arena_cell_state[y][x] = 0;
    }
  }

  // Treat the two door cells as known free travel cells.
  arena_cell_state[ARENA_ENTRANCE.y][ARENA_ENTRANCE.x] = 1;
  arena_cell_state[ARENA_EXIT.y][ARENA_EXIT.x] = 1;
}

// Read two bits from the 21-byte packed 9x9 map
// Extracts one 2-bit cell value from the packed server map
uint8_t arenaReadTwoBitCell(const uint8_t* map21, uint16_t cell_index)
{
  const uint16_t bit_index = cell_index * 2;
  const uint8_t byte_index = bit_index / 8;
  const uint8_t shift = bit_index % 8;

  uint16_t value = map21[byte_index] >> shift;
  if (shift > 6 && byte_index + 1 < 21) {
    value |= static_cast<uint16_t>(map21[byte_index + 1]) << (8 - shift);
  }

  return static_cast<uint8_t>(value & 0x03);
}

// Decode 21 bytes into the 9x9 inner arena
// Assumption: row-major order over inner cells only:
//   index = inner_y * 9 + inner_x
//   grid x = inner_x + 1, grid y = inner_y + 1
// Each cell uses two bits: 0 unknown, 1 seeded, 2 fertile, 3 infertile
// Converts the packed 9x9 map into the 11x11 planner grid
void arenaDecodeGetMap21(const uint8_t* map21)
{
  arenaClearMapToUnknown();

  if (map21 == nullptr) {
    return;
  }

  for (uint8_t inner_y = 0; inner_y < 9; ++inner_y) {
    for (uint8_t inner_x = 0; inner_x < 9; ++inner_x) {
      const uint16_t cell_index = inner_y * 9 + inner_x;
      const uint8_t state = arenaReadTwoBitCell(map21, cell_index);
      const uint8_t grid_x = inner_x + 1;
      const uint8_t grid_y = inner_y + 1;
      arena_cell_state[grid_y][grid_x] = state;
    }
  }

  arena_cell_state[ARENA_ENTRANCE.y][ARENA_ENTRANCE.x] = 1;
  arena_cell_state[ARENA_EXIT.y][ARENA_EXIT.x] = 1;
}

// Requests getMap periodically and imports the latest reply if present
void arenaRequestAndRefreshMap()
{
  const unsigned long now = millis();

  if (now - arena_last_getmap_ms >= ARENA_GETMAP_INTERVAL_MS || arena_last_getmap_ms == 0) {
    arena_last_getmap_ms = now;
    messagesSendGetMap();
  }

  uint8_t map21[21];
  if (messagesCopyLastMap(map21, sizeof(map21))) {
    arenaDecodeGetMap21(map21);
  }
}

// Forgets dynamic obstacles after ARENA_TEMP_OBSTACLE_MS (20s)
void arenaExpireTemporaryObstacles()
{
  const unsigned long now = millis();

  for (uint8_t y = 0; y < ARENA_H; ++y) {
    for (uint8_t x = 0; x < ARENA_W; ++x) {
      if (arena_temp_blocked_until[y][x] != 0 && now >= arena_temp_blocked_until[y][x]) {
        arena_temp_blocked_until[y][x] = 0;
      }
    }
  }
}

// A cell is blocked by walls or active temporary obstacles only
bool arenaBlocked(int8_t x, int8_t y)
{
  if (!arenaInside(x, y)) {
    return true;
  }

  arenaExpireTemporaryObstacles();

  return arenaIsWallCell(x, y) || arena_temp_blocked_until[y][x] != 0;
}

// Gives unknown cells a longer obstacle memory than explored cells
unsigned long arenaTemporaryObstacleDuration(ArenaPoint p)
{
  if (!arenaInside(p.x, p.y)) {
    return ARENA_TEMP_OBSTACLE_UNKNOWN_MS;
  }

  if (arenaCellExplored(p.x, p.y)) {
    return ARENA_TEMP_OBSTACLE_EXPLORED_MS;
  }

  return ARENA_TEMP_OBSTACLE_UNKNOWN_MS;
}

//  States 1/2/3 count as explored; state 0 remains unknown
bool arenaCellExplored(int8_t x, int8_t y)
{
  if (!arenaInside(x, y)) {
    return false;
  }

  return arena_cell_state[y][x] == 1 ||
         arena_cell_state[y][x] == 2 ||
         arena_cell_state[y][x] == 3;
}

// Scores movement so explored and low-y cells are preferred
uint16_t arenaTravelCost(ArenaPoint from, ArenaPoint to, ArenaPoint parent)
{
  if (arenaBlocked(to.x, to.y)) {
    return ARENA_INF_COST;
  }

  uint16_t cost = ARENA_COST_EXPLORED;

  if (!arenaCellExplored(to.x, to.y)) {
    cost += ARENA_COST_UNKNOWN_EXTRA;
  }

  if (to.y <= 4) {
    cost += ARENA_COST_LOW_Y_EXTRA;
  } else {
    cost += ARENA_COST_HIGH_Y_EXTRA;
  }

  // Small tie-breaker: prefer fewer turns when costs are otherwise close.
  if (parent.x >= 0 && parent.y >= 0) {
    const int8_t dx1 = from.x - parent.x;
    const int8_t dy1 = from.y - parent.y;
    const int8_t dx2 = to.x - from.x;
    const int8_t dy2 = to.y - from.y;

    if (dx1 != dx2 || dy1 != dy2) {
      cost += ARENA_COST_TURN_EXTRA;
    }
  }

  return cost;
}

// Finds next point to drive to
ArenaPoint arenaNextPoint(ArenaPoint p, ArenaHeading h)
{
  switch (h) {
    case ArenaHeading::North: return {p.x, static_cast<int8_t>(p.y - 1)};
    case ArenaHeading::East:  return {static_cast<int8_t>(p.x + 1), p.y};
    case ArenaHeading::South: return {p.x, static_cast<int8_t>(p.y + 1)};
    case ArenaHeading::West:  return {static_cast<int8_t>(p.x - 1), p.y};
  }

  return p;
}

// Temporarily blocks a cell seen by the ultrasonic sensor
void arenaMarkTemporaryObstacle(ArenaPoint p)
{
  if (!arenaInside(p.x, p.y)) {
    return;
  }

  if (arenaIsWallCell(p.x, p.y)) {
    return;  // Permanent walls are already handled by arenaIsWallCell.
  }

  const unsigned long duration_ms = arenaTemporaryObstacleDuration(p);
  arena_temp_blocked_until[p.y][p.x] = millis() + duration_ms;

  Serial.print("arena_temp_obstacle x=");
  Serial.print(p.x);
  Serial.print(" y=");
  Serial.print(p.y);
  Serial.print(" explored=");
  Serial.print(arenaCellExplored(p.x, p.y) ? 1 : 0);
  Serial.print(" duration_ms=");
  Serial.println(duration_ms);
}

// Reads the forward ultrasonic sensor for near obstacles
bool arenaObstacleAhead()
{
  const float d = forward_distance_mm();
  return d > US_MIN_VALID_MM && d < ARENA_OBSTACLE_NEAR_MM;
}

// Converts the reader UID bytes into the text key used by the server/table
void uidToText(char* out, size_t out_size)
{
  if (out_size == 0) {
    return;
  }

  out[0] = '\0';

  for (byte i = 0; i < rfid.uid.size; ++i) {
    char part[3];
    snprintf(part, sizeof(part), "%02X", rfid.uid.uidByte[i]);
    strncat(out, part, out_size - strlen(out) - 1);
  }
}

// ULooks up a UID in RFID_LOCATIONS (for offline usage only)
bool arenaLookupRFID(const char* uid, ArenaPoint* out)
{
  if (uid == nullptr || out == nullptr) {
    return false;
  }

  for (uint8_t i = 0; i < RFID_LOCATION_COUNT; ++i) {
    if (strcmp(uid, RFID_LOCATIONS[i].uid) == 0) {
      out->x = RFID_LOCATIONS[i].x;
      out->y = RFID_LOCATIONS[i].y;
      return true;
    }
  }

  return false;
}

// Localises from isFertile reply then falls back to the local UID table.
// Autonomous version of the '9' diagnostic.
// Non-blocking: if no tag is presented right now, returns false
// immediately so the control loop keeps ticking. When a tag is
// present we use the same scanRFIDAndQueryServer() helper as the
// manual '9' command so both paths share one implementation.
bool arenaUpdatePoseFromRFID()
{
  // Quick non-blocking presence check. If nothing is on the reader,
  // bail out — the autonomous loop cannot afford to wait 5 s here.
  char uid[32];
  if (!tryScanRFIDOnce(uid, sizeof(uid))) {
    return false;
  }

  // Debounce repeat reads of the same tag.
  if (millis() - lastScanTime < RFID_SCAN_COOLDOWN_MS) {
    return false;
  }
  lastScanTime = millis();

  Serial.print("arena_rfid_uid=");
  Serial.println(uid);

  // Send isFertile and wait briefly for the matching server reply.
  // We've already consumed the scan above, so we just do the query.
  bool located_from_server = false;
  if (messagesSendIsFertile(uid)) {
    int8_t server_x = -1;
    int8_t server_y = -1;
    uint8_t server_state = 0;

    if (messagesWaitForFertilityReply(uid, RFID_AUTO_REPLY_TIMEOUT_MS,
                                      &server_x, &server_y, &server_state)) {
      if (arenaInside(server_x, server_y) && !arenaIsWallCell(server_x, server_y)) {
        arena_pose = {server_x, server_y};
        arena_have_pose = true;
        located_from_server = true;

        if (server_state <= 3) {
          arena_cell_state[server_y][server_x] = server_state;
        }

        Serial.print("arena_pose_from_isfertile x=");
        Serial.print(arena_pose.x);
        Serial.print(" y=");
        Serial.print(arena_pose.y);
        Serial.print(" state=");
        Serial.println(server_state);
      }
    } else {
      Serial.println("arena_warning=isfertile_no_position_reply");
    }
  }

  // Fallback: offline / no server reply → look UID up locally.
  if (!located_from_server) {
    ArenaPoint located;
    if (arenaLookupRFID(uid, &located)) {
      arena_pose = located;
      arena_have_pose = true;

      Serial.print("arena_pose_from_local_rfid_table x=");
      Serial.print(arena_pose.x);
      Serial.print(" y=");
      Serial.println(arena_pose.y);
    } else {
      Serial.println("arena_warning=rfid_uid_not_in_location_table_and_no_server_position");
    }
  }

  return arena_have_pose;
}

// Converts a neighbouring target cell into the required heading
ArenaHeading arenaHeadingForStep(ArenaPoint from, ArenaPoint to)
{
  if (to.x > from.x) return ArenaHeading::East;
  if (to.x < from.x) return ArenaHeading::West;
  if (to.y > from.y) return ArenaHeading::South;
  return ArenaHeading::North;
}

// Computes the shortest heading change
int8_t arenaHeadingDiff(ArenaHeading from, ArenaHeading to)
{
  int8_t diff = static_cast<int8_t>(to) - static_cast<int8_t>(from);

  while (diff > 2) diff -= 4;
  while (diff < -2) diff += 4;

  return diff;
}

// Turns the robot to face the next path cell
bool arenaTurnTo(ArenaHeading target)
{
  const int8_t diff = arenaHeadingDiff(arena_heading, target);

  if (diff == 0) {
    return true;
  }

  bool ok = false;

  if (diff == 1) {
    ok = turnRight90WithLines();
  } else if (diff == -1) {
    ok = turnLeft90WithLines();
  } else {
    ok = turnRight180WithLines();
  }

  if (ok) {
    arena_heading = target;
  }

  return ok;
}

// URuns Dijkstra over the weighted occupancy grid
bool arenaFindPath(ArenaPoint start, ArenaPoint goal, ArenaPoint* path, uint8_t* path_len)
{
  if (path == nullptr || path_len == nullptr) {
    return false;
  }

  *path_len = 0;

  if (!arenaInside(start.x, start.y) || !arenaInside(goal.x, goal.y)) {
    return false;
  }

  if (arenaBlocked(start.x, start.y) || arenaBlocked(goal.x, goal.y)) {
    return false;
  }

  bool closed[ARENA_H][ARENA_W] = {};
  uint16_t dist[ARENA_H][ARENA_W];
  ArenaPoint parent[ARENA_H][ARENA_W];

  for (uint8_t y = 0; y < ARENA_H; ++y) {
    for (uint8_t x = 0; x < ARENA_W; ++x) {
      dist[y][x] = ARENA_INF_COST;
      parent[y][x] = {-1, -1};
    }
  }

  dist[start.y][start.x] = 0;

  const int8_t dx[4] = {0, 1, 0, -1};
  const int8_t dy[4] = {-1, 0, 1, 0};

  while (true) {
    uint16_t best = ARENA_INF_COST;
    ArenaPoint cur = {-1, -1};

    for (uint8_t y = 0; y < ARENA_H; ++y) {
      for (uint8_t x = 0; x < ARENA_W; ++x) {
        if (!closed[y][x] && dist[y][x] < best) {
          best = dist[y][x];
          cur = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        }
      }
    }

    if (cur.x < 0 || cur.y < 0) {
      break;
    }

    if (cur.x == goal.x && cur.y == goal.y) {
      break;
    }

    closed[cur.y][cur.x] = true;

    for (uint8_t i = 0; i < 4; ++i) {
      const ArenaPoint next = {
        static_cast<int8_t>(cur.x + dx[i]),
        static_cast<int8_t>(cur.y + dy[i])
      };

      if (!arenaInside(next.x, next.y) || closed[next.y][next.x] || arenaBlocked(next.x, next.y)) {
        continue;
      }

      const uint16_t step_cost = arenaTravelCost(cur, next, parent[cur.y][cur.x]);
      if (step_cost >= ARENA_INF_COST) {
        continue;
      }

      const uint16_t alt = dist[cur.y][cur.x] + step_cost;
      if (alt < dist[next.y][next.x]) {
        dist[next.y][next.x] = alt;
        parent[next.y][next.x] = cur;
      }
    }
  }

  if (dist[goal.y][goal.x] >= ARENA_INF_COST) {
    return false;
  }

  ArenaPoint reversed[ARENA_CELL_COUNT];
  uint8_t count = 0;
  ArenaPoint cur = goal;

  while (!(cur.x == start.x && cur.y == start.y) && count < ARENA_CELL_COUNT) {
    reversed[count++] = cur;
    cur = parent[cur.y][cur.x];

    if (cur.x < 0 || cur.y < 0) {
      return false;
    }
  }

  reversed[count++] = start;

  for (uint8_t i = 0; i < count; ++i) {
    path[i] = reversed[count - 1 - i];
  }

  *path_len = count;
  return true;
}

// Follows the line one grid cell, watching RFID and obstacles
bool arenaDriveOneCellTo(ArenaPoint next)
{
  const ArenaHeading target_heading = arenaHeadingForStep(arena_pose, next);

  if (!arenaTurnTo(target_heading)) {
    return false;
  }

  if (arenaObstacleAhead()) {
    arenaMarkTemporaryObstacle(next);
    stopMotors();
    return false;
  }

  // Drive one cell. RFID confirms the pose; dead reckoning is used only if no tag confirms it.
  const unsigned long start_ms = millis();
  bool reached_by_rfid = false;

  while (millis() - start_ms < 3500UL) {
    if (checkStopInputsDuringTest() || !messagesRobotAllowedToMove()) {
      stopMotors();
      return false;
    }

    serviceServoPulses();
    update_turn_angle();
    messagesLoop();
    arenaRequestAndRefreshMap();

    if (arenaObstacleAhead()) {
      arenaMarkTemporaryObstacle(next);
      stopMotors();
      return false;
    }

    read_rc_discharge_times();
    update_calibrated_values();

    const int32_t pos = estimate_line_position();
    if (pos >= 0) {
      follow_line(pos - LINE_CENTER);
    } else {
      drive_forward();
    }

    if (arenaUpdatePoseFromRFID()) {
      reached_by_rfid = (arena_pose.x == next.x && arena_pose.y == next.y);
      if (reached_by_rfid) {
        break;
      }
    }

    delay(5);
  }

  stopMotors();
  delay(80);

  if (!reached_by_rfid) {
    arena_pose = next;       // best-effort dead-reckoning estimate
    arena_have_pose = true;
    Serial.println("arena_warning=pose_estimated_without_rfid");
  }

  return true;
}

// Replans course until the goal is reached
bool arenaPathfindTo(ArenaPoint goal)
{
  if (startup_cal_state != StartupCalState::Ready) {
    Serial.println("arena_error=not_calibrated");
    return false;
  }

  if (!running) {
    Serial.println("arena_error=not_running");
    return false;
  }

  arenaClearMapToUnknown();
  arenaRequestAndRefreshMap();

  if (!arena_have_pose) {
    Serial.println("arena_waiting_for_first_known_rfid");

    const unsigned long start_ms = millis();
    while (!arena_have_pose && millis() - start_ms < 8000UL) {
      messagesLoop();
      arenaRequestAndRefreshMap();
      arenaUpdatePoseFromRFID();
      delay(5);
    }
  }

  if (!arena_have_pose) {
    Serial.println("arena_error=no_known_rfid_pose");
    return false;
  }

  while (!(arena_pose.x == goal.x && arena_pose.y == goal.y)) {
    if (checkStopInputsDuringTest() || !messagesRobotAllowedToMove()) {
      stopMotors();
      return false;
    }

    arenaRequestAndRefreshMap();
    arenaExpireTemporaryObstacles();

    ArenaPoint path[ARENA_CELL_COUNT];
    uint8_t path_len = 0;

    if (!arenaFindPath(arena_pose, goal, path, &path_len) || path_len < 2) {
      Serial.println("arena_error=no_path");
      stopMotors();
      return false;
    }

    Serial.print("arena_path_len=");
    Serial.println(path_len);
    Serial.print("arena_next x=");
    Serial.print(path[1].x);
    Serial.print(" y=");
    Serial.print(path[1].y);
    Serial.print(" state=");
    Serial.println(arena_cell_state[path[1].y][path[1].x]);

    const ArenaPoint next = path[1];

    if (!arenaDriveOneCellTo(next)) {
      // Re-plan after a failed step, usually because a temporary obstacle was added.
      continue;
    }
  }

  stopMotors();
  Serial.println("arena_destination_reached");
  return true;
}

// Routes to the entrance square (0,7)
bool arenaReturnToEntrance()
{
  return arenaPathfindTo(ARENA_ENTRANCE);
}

// Routes to the exit square (0,3)
bool arenaGoToExit()
{
  return arenaPathfindTo(ARENA_EXIT);
}
