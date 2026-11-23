#ifndef MID_LOCAL_EXPLORATION_H
#define MID_LOCAL_EXPLORATION_H
#include <stdint.h>
#include <stdbool.h>
#include "mid_odometry.h"
#define LOCAL_SECTORS 36U
#define LOCAL_SECTOR_DEG 10U
#define LOCAL_EVIDENCE_MS 4000U
#define LOCAL_CLEAR_MM 350U /* 275 preventive + 50 segment + 25 planning reserve, not a Safety threshold. */
#define LOCAL_SEGMENT_MM 50U
#define LOCAL_FRONT_CONFIRMATIONS 2U /* Distinct fresh, pose-associated captures for bounded straight advance, not a free cone. */
#define LOCAL_FRONT_BEARING_RAD 0.0872664626f /* At most 5 degrees from current forward bearing. */
#define LOCAL_HISTORY_RADIUS_M 0.05f
#define LOCAL_RETREAT_PROGRESS_M 0.010f /* Net backward separation of withdrawal origins; half a 20-mm move, not the visit radius. */
#define LOCAL_CHANGE_MM 50U
#define LOCAL_CHANGE_TOLERANCE_MM 25U
#define LOCAL_CHANGE_OBSERVATIONS 3U
#define LOCAL_TURN_STEP_DEG 10.0f
#define LOCAL_TURN_MAX_WHEEL_DIFFERENCE_MM 4U /* Experimental: 2 mm net center travel in the ideal turn model; no slip/clearance guarantee. */
#define LOCAL_TURN_RESERVE_DEG 2.0f
#define LOCAL_MARGIN_MM 30U
#define LOCAL_LINK_TIMEOUT_MS 3000U /* Six 500-ms diagnostic polls without fresh round-trip proof. */
#define LOCAL_LINK_REPLY_MS 1000U /* Bound the echoed response age, independently of navigation authorization. */
#define LOCAL_MAX_SCAN_DEG 180U
#define LOCAL_VISITS 32U
/* Pure bounded evidence/scoring, no control, acquisition, scheduler or heap.
 * 0 unknown, 1 measured/insufficient corridor evidence, 2 occupied, 3 clear
 * along the measured bearing. State 3 NEVER certifies a free acoustic cone. */
typedef struct {
    uint16_t mm[LOCAL_SECTORS];
    uint8_t state[LOCAL_SECTORS];
    TickType_t tick[LOCAL_SECTORS];
    uint32_t sequence[2];
    bool seen[2], origin_set;
    float x,y;
    struct {float x,y,yaw;bool retreat;} visits[LOCAL_VISITS];
    uint8_t visit_count, visit_next;
} LocalEvidence;
void Local_Reset(LocalEvidence *e,float x,float y);
bool Local_Observe(LocalEvidence *e,unsigned sensor,const OdometryRay *r,TickType_t now);
void Local_Expire(LocalEvidence *e,TickType_t now);
int Local_Select(const LocalEvidence *e,float x,float y,float yaw,uint32_t *evaluated,uint32_t *repeated);
bool Local_Visited(const LocalEvidence *e,float x,float y,float yaw);
bool Local_Visit(LocalEvidence *e,float x,float y,float yaw);
bool Local_RetreatAllowed(const LocalEvidence *e,float x,float y,float yaw);
bool Local_RetreatVisit(LocalEvidence *e,float x,float y,float yaw);
float Local_Difference(float angle);
#endif
