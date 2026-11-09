#include "mid_motion.h"
#include "task_motor_control.h"
#include "mid_navigation.h"

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

bool Motion_NavigationAdvance(TickType_t issued_tick, uint32_t session) {
    return TaskMotorControl_TrySetNavigationSpeedsAt(MOTION_DRIVE_CPS, MOTION_DRIVE_CPS,
                                                    issued_tick, session);
}

bool Motion_NavigationReverse(TickType_t issued_tick, uint32_t session) {
    return TaskMotorControl_TrySetNavigationSpeedsAt(-MOTION_DRIVE_CPS, -MOTION_DRIVE_CPS,
                                                    issued_tick, session);
}

bool Motion_NavigationTurn(TickType_t issued_tick, uint32_t session) {
#if NAV_EXPERIMENTAL_TURN_ENABLED
    return TaskMotorControl_TrySetNavigationSpeedsAt(NAV_TURN_LEFT_CPS, NAV_TURN_RIGHT_CPS,
                                                    issued_tick, session);
#else
    (void)issued_tick; (void)session;
    return false;
#endif
}
