// =====================================================
// ArenaTypes.h
// Shared arena structs/enums kept in a header so Arduino prototypes see them.
// Used by Main.ino and ArenaPathfinding.ino.
// =====================================================

#ifndef ARENA_TYPES_H
#define ARENA_TYPES_H

#include <Arduino.h>

// Put these small types in a header so Arduino's automatic .ino prototype
// generation sees them before it generates prototypes for ArenaPathfinding.ino.
struct ArenaPoint {
  int8_t x;
  int8_t y;
};

enum class ArenaHeading : uint8_t {
  North = 0,
  East  = 1,
  South = 2,
  West  = 3
};

struct RFIDLocation {
  const char* uid;
  int8_t x;
  int8_t y;
};

#endif
