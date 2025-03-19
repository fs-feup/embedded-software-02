#pragma once

struct Code {
  int key;
  int code;
};

enum class State { AS_MANUAL, AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY };

enum class Mission { MANUAL, ACCELERATION, SKIDPAD, AUTOCROSS, TRACKDRIVE, EBS_TEST, INSPECTION, CHUCK};

enum class StateTransition {TO_MANUAL, TO_OFF, TO_READY, TO_DRIVING, TO_FINISHED, TO_EMERGENCY};
