#pragma once

enum class ObjectAction { None, Pickup, Release };
enum class ObjectActionStatus { Unavailable, Busy, Complete, Failed };

// Integration seam for the future gripper/lift subsystem. Complete must mean
// physically confirmed success, not merely that a motor timer elapsed.
// Hardware currently implements Unavailable and NEVER fabricates success.
