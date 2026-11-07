#include "mid_motion.h"
#include "task_motor_control.h"

bool Motion_Execute(MotionCommand_t command, RobotControlState_t required_mode,
                    TickType_t issued_tick) {
    int32_t left, right;
    switch (command) {
        case MOTION_FORWARD: left = MOTION_DRIVE_CPS; right = MOTION_DRIVE_CPS; break;
        case MOTION_REVERSE: left = -MOTION_DRIVE_CPS; right = -MOTION_DRIVE_CPS; break;
        case MOTION_TURN_LEFT: left = -MOTION_TURN_CPS; right = MOTION_TURN_CPS; break;
        case MOTION_TURN_RIGHT: left = MOTION_TURN_CPS; right = -MOTION_TURN_CPS; break;
        case MOTION_HALT: left = 0; right = 0; break;
        default: return false;
    }
    return TaskMotorControl_TrySetTargetSpeedsAt(left, right, issued_tick, required_mode);
}
