import math
from util.objects import Vector3

# This file is for small utilities for math and movement


def backsolve(target, car, time, gravity=650):
    # Finds the acceleration required for a car to reach a target in a specific amount of time
    velocity_required = (target - car.location) / time
    acceleration_required = velocity_required - car.velocity
    acceleration_required[2] += (gravity * time)
    return acceleration_required


def cap(x, low, high):
    # caps/caps a number between a low and high value
    if x < low:
        return low
    elif x > high:
        return high
    return x


def defaultPD(agent, local_target, direction=1.0):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
    target_angles = [
        # angle required to pitch towards target
        math.atan2(local_target[2], local_target[0]),
        # angle required to yaw towards target
        math.atan2(local_target[1], local_target[0]),
        math.atan2(up[1], up[2])]  # angle required to roll upright
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    agent.controller.pitch = steerPD(
        target_angles[0], agent.me.angular_velocity[1]/4)
    agent.controller.yaw = steerPD(
        target_angles[1], -agent.me.angular_velocity[2]/4)
    agent.controller.roll = steerPD(
        target_angles[2], agent.me.angular_velocity[0]/2)
    # Returns the angles, which can be useful for other purposes
    return target_angles


def defaultThrottle(agent, target_speed, direction=1.0):
    # accelerates the car to a desired speed using throttle and boost
    car_speed = agent.me.local(agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * sign(t)/1000, -1.0, 1.0)
    agent.controller.boost = True if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0 else False
    return car_speed


def in_field(point, radius):
    # determines if a point is inside the standard soccer field
    point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))
    if point[0] > 4080 - radius:
        return False
    elif point[1] > 5900 - radius:
        return False
    elif point[0] > 880 - radius and point[1] > 5105 - radius:
        return False
    elif point[0] > 2650 and point[1] > -point[0] + 8025 - radius:
        return False
    return True


def find_slope(shot_vector, car_to_target):
    # Finds the slope of your car's position relative to the shot vector (shot vector is y axis)
    # 10 = you are on the axis and the ball is between you and the direction to shoot in
    # -10 = you are on the wrong side
    # 1.0 = you're about 45 degrees offcenter
    d = shot_vector.dot(car_to_target)
    e = abs(shot_vector.cross((0, 0, 1)).dot(car_to_target))
    return cap(d / e if e != 0 else 10*sign(d), -3.0, 3.0)


def post_correction(ball_location, left_target: Vector3, right_target: Vector3):
    # this function returns target locations that are corrected to account for the ball's radius
    # it also checks to make sure the ball can fit between the corrected locations
    # We purposely make this a bit larger so that our shots have a higher chance of success
    ball_radius = 110
    goal_line_perp = (right_target - left_target).cross((0, 0, 1))
    left_adjusted = left_target + \
        ((left_target - ball_location).normalize().cross((0, 0, -1))*ball_radius)
    right_adjusted = right_target + \
        ((right_target - ball_location).normalize().cross((0, 0, 1))*ball_radius)
    left_corrected = left_target if (
        left_adjusted-left_target).dot(goal_line_perp) > 0.0 else left_adjusted
    right_corrected = right_target if (
        right_adjusted-right_target).dot(goal_line_perp) > 0.0 else right_adjusted

    difference = (right_corrected - left_corrected)
    new_goal_line = difference.normalize()
    new_goal_width = difference.magnitude()
    new_goal_perp = (new_goal_line.cross((0, 0, 1)))
    goal_center = left_corrected + (new_goal_line * new_goal_width * 0.5)
    ball_to_goal = (goal_center - ball_location).normalize()

    ball_fits = new_goal_width * \
        abs(new_goal_perp.dot(ball_to_goal)) > ball_radius * 2
    return left_corrected, right_corrected, ball_fits


def brake_distance(v1, v2, s):
    # Calculate the brake distance
    d1 = v1**2 / (s * 2)
    d2 = v2**2 / (s * 2)
    return d1 - d2


def quadratic(a, b, c):
    # Returns the two roots of a quadratic
    inside = math.sqrt((b*b) - (4*a*c))
    if a != 0:
        return (-b + inside)/(2*a), (-b - inside)/(2*a)
    else:
        return -1, -1


def solve_quadratic(a, b, c, ans, side):
    # Solves the quadratic
    rt = (ans - c) / a + b**2 / (4 * a**2)
    if rt < 0:
        return None
    else:
        return -b / (2 * a) + math.sqrt(rt) * sign(side)
    

def handbrake_handler(agent, target):
    # Handle handbrake usage
    agent.controller.handbrake = agent.me.av.magnitude() > 2 and agent.me.forward.angle_to(target - agent.me.location) >= math.pi / 2 and agent.me.forward.angle_to(agent.me.velocity) < math.pi / 4


def shot_valid(agent, shot, threshold=45):
    # Returns True if the ball is still where the shot anticipates it to be
    # First finds the two closest slices in the ball prediction to shot's intercept_time
    # threshold controls the tolerance we allow the ball to be off by
    slices = agent.get_ball_prediction_struct().slices
    soonest = 0
    latest = len(slices)-1
    while len(slices[soonest:latest+1]) > 2:
        midpoint = (soonest+latest) // 2
        if slices[midpoint].game_seconds > shot.intercept_time:
            latest = midpoint
        else:
            soonest = midpoint
    # preparing to interpolate between the selected slices
    dt = slices[latest].game_seconds - slices[soonest].game_seconds
    time_from_soonest = shot.intercept_time - slices[soonest].game_seconds
    slopes = (Vector3(slices[latest].physics.location) -
              Vector3(slices[soonest].physics.location)) * (1/dt)
    # Determining exactly where the ball will be at the given shot's intercept_time
    predicted_ball_location = Vector3(
        slices[soonest].physics.location) + (slopes * time_from_soonest)
    # Comparing predicted location with where the shot expects the ball to be
    return (shot.ball_location - predicted_ball_location).magnitude() < threshold


def calculate_dribble_threats(agent):
    # Calculates the fastest threat time when going for a dribble
    best_t = math.inf
    for car in agent.foes:
        cur_t = (car.location.flat_dist(agent.ball.location) + 92.75) * safe_div(car.velocity.flat_dist(agent.ball.velocity) + 92.75 + 292 * bool(not car.doublejumped))
        if cur_t < best_t:
            best_t = cur_t
    return cur_t


def velvsacc_to_velvstime(v, t, k, m):
    # Get the velocity reached after a given time
    t1 = math.log(-(v / -(m/k) - 1), math.e**k) # -m/k * (1 - e^(k * t))
    return -m/k * (1 - math.e**(k * (t1 + t)))


def velvsacc_changevstime(v, v2, k, m):
    # Get the time taken to change the velocity to a certain amount
    v1 = cap(-(-v * k / m - 1), 0, math.inf)
    v2 = cap(-(-v2 * k / m - 1), 0, math.inf)
    if v1 > 0 and v2 > 0:
        t1 = math.log(v1, math.e**k) # -m/k * (1 - e^(k * t))
        t2 = math.log(v2, math.e**k) # -m/k * (1 - e^(k * t))
        return t2 - t1
    else:
        return math.inf
    

def velvsacc_to_distvstime(v, t, k, m):
    # Get the distance driven during the car's acceleration
    t1 = math.log(-(v / -(m/k) - 1), math.e**k) # -m/k * (1 - e^(k * t))
    int1 = -m/k * (t1 - (math.e**(k * t1)) / k)
    int2 = -m/k * ((t1 + t) - (math.e**(k * (t1 + t))) / k)
    return int2 - int1


def distvstime(v, t, b, p):
    # Get the end velocity given the time (Current Velocity, Throttle Time, Current Boost, Boost Power (default = 1))
    tb = cap(b / 100 * 3, 0, t)
    td = t - tb
    nv = v
    d = 0
    # The max speed achieved with boost
    if tb > 0 and nv < 1400:
        temp_v = nv
        nv = cap(velvsacc_to_velvstime(nv, tb, -1440 / 1400, 1600 + p * 5950 / 6), 0, 1400)
        offset = velvsacc_changevstime(temp_v, 1400, -1440 / 1400, 1600 + p * 5950 / 6)
        d += velvsacc_to_distvstime(temp_v, cap(offset, 0, tb), -1440 / 1400, 1600 + p * 5950 / 6)
        tb = cap(tb - offset, 0, math.inf)
    if tb > 0 and nv < 1410:
        temp_v = nv
        nv = cap(velvsacc_to_velvstime(nv, tb, -16, 1410 * 16 + p * 5950 / 6), 0, 1410)
        offset = velvsacc_changevstime(temp_v, 1410, -16, 1410 * 16 + p * 5950 / 6)
        d += velvsacc_to_distvstime(temp_v, cap(offset, 0, tb), -16, 1410 * 16 + p * 5950 / 6)
        tb = cap(tb - offset, 0, math.inf)
    if tb > 0 and nv < 2300:
        temp_v = nv
        nv = cap(nv + p * 5950 / 6 * tb, 0, 2300)
        t_dif = (nv - temp_v) / (p * 5950 / 6)
        d += temp_v * t_dif + (nv - temp_v) * t_dif / 2 + nv * (tb - t_dif)
        tb = 0
    # The max speed achieved after boost with throttle
    if td > 0 and nv < 1400:
        temp_v = nv
        nv = cap(velvsacc_to_velvstime(nv, td, -1440 / 1400, 1600), 0, 1400)
        offset = velvsacc_changevstime(temp_v, 1400, -1440 / 1400, 1600)
        d += velvsacc_to_distvstime(temp_v, cap(offset, 0, td), -1440 / 1400, 1600)
        td = cap(td - offset, 0, math.inf)
    if td > 0 and nv < 1410:
        temp_v = nv
        nv = cap(velvsacc_to_velvstime(nv, td, -16, 1410 * 16), 0, 1410)
        offset = velvsacc_changevstime(temp_v, 1410, -16, 1410 * 16)
        d += velvsacc_to_distvstime(temp_v, cap(offset, 0, td), -16, 1410 * 16)
        td = cap(td - offset, 0, math.inf)
        d += nv * td
    if td > 0 and nv >= 1410:
        d += nv * td
    return d


def safe_div(x):
    # Makes sure we aren't dividing by 0
    if x == 0:
        return math.inf
    return 1 / x
    

def side(x):
    # returns -1 for blue team and 1 for orange team
    if x < 0.1:
        return -1
    return 1


def sign(x):
    # returns the sign of a number, -1, 0, +1
    if x < 0.0:
        return -1
    elif x > 0.0:
        return 1
    else:
        return 0.0


def steerPD(angle, rate):
    # A Proportional-Derivative control loop used for defaultPD
    return cap(((35*(angle+rate))**3)/10, -1.0, 1.0)


def lerp(a, b, t):
    # Linearly interpolate from a to b using t
    # For instance, when t == 0, a is returned, and when t == 1, b is returned
    # Works for both numbers and Vector3s
    return (b - a) * t + a


def invlerp(a, b, v):
    # Inverse linear interpolation from a to b with value v
    # For instance, it returns 0 if v == a, and returns 1 if v == b, and returns 0.5 if v is exactly between a and b
    # Works for both numbers and Vector3s
    return (v - a)/(b - a)


def calc_next_bounce(pos, vel, height, g):
    # Calculate the next bounce
    pred_x = (4096 - height - pos.x * sign(vel.x)) * safe_div(vel.x) * sign(vel.x)
    pred_y = (5120 - height - pos.y * sign(vel.y)) * safe_div(vel.y) * sign(vel.y)
    pred_z1 = solve_quadratic(-g / 2, vel.z, pos.z, height, 1)
    if pred_z1 == None:
        pred_z1 = 0
    pred_z2 = solve_quadratic(-g / 2, vel.z, pos.z, 2044 - height, -1)
    if pred_z2 == None:
        pred_z2 = math.inf
    smallest = 0
    li = [pred_z1, pred_x, pred_y, pred_z2]
    for i in range(len(li)):
        if li[i] < li[smallest]:
            smallest = i
    return li[smallest], smallest


def surface_pos(p):
    # Get the surface position
    best = p.z
    np = Vector3(p.x, p.y, 0)
    if 0 <= 4096 - abs(p.x) < best:
        np = Vector3(4096 * sign(p.x), p.y, p.z)
        best = 4096 - abs(p.x)
    if 0 <= 5120 - abs(p.y) < best:
        np = Vector3(p.x, 5120 * sign(p.y), p.z)
        best = 5120 - abs(p.y)
    return np