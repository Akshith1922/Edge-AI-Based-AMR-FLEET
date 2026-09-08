"""
Algorithm 5 — Heartbeat-Based Failure Detection with Idempotent Recovery.

Every mechanism here is reused from another module rather than duplicated:
the failed robot's footprint becomes a block event (Algorithm 4), its task
goes back through the normal auction (Algorithm 6), and its reservation and
wait-graph entry are invalidated through Algorithm 1's table. That reuse is
what makes concurrent detection by several peers safe — ``declare_failed`` is
idempotent, so two robots noticing at once is a no-op, not a race.
"""


def heartbeat_age(robot, tick):
    return tick - robot.last_heartbeat


def is_failed(robot, tick, ttl):
    return heartbeat_age(robot, tick) > ttl


def declare_failed(robot, table, pool, blocks, tick, lamport, log):
    from .robot import RobotState

    if robot.state == RobotState.FAILED:
        return False                        # idempotent under concurrent detection
    robot.state = RobotState.FAILED
    robot.failed_at = tick
    table.invalidate(robot.id)
    robot.waiting_for = None
    robot.path = []
    blocks.report(robot.pos(), robot.id, lamport, tick,
                  severity="FULL", cause="failed_robot")

    if robot.task is not None:
        task = robot.task
        carrying = task.item_in_transit      # read before the release clears it
        pool.release_for_reassignment(task, tick, item_in_transit=carrying)
        if carrying:
            log(f"ALERT ITEM_LOCATION_UNKNOWN for task T{task.id}")
        robot.task = None
    log(f"Robot {robot.id} declared FAILED (heartbeat TTL expired)")
    return True


def handle_recovery(robot, table, blocks, tick, log):
    """``handle_recovery``: the robot pulls current truth rather than trusting
    its own stale memory — it drops its task and reservation and replans from
    where it actually is."""
    from .robot import RobotState

    robot.state = RobotState.RECOVERING
    table.invalidate(robot.id)
    blocks.clear(robot.pos())
    robot.task = None
    robot.path = []
    robot.path_index = 0
    robot.waiting_for = None
    robot.wait_started = None
    robot.heartbeat_frozen = False
    robot.last_heartbeat = tick
    robot.state = RobotState.IDLE
    log(f"Robot {robot.id} recovered and re-synced with the fleet table")
