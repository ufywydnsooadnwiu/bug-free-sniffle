
from util.common import *
from util.objects import BotCommandAgent
from util.routines import flick, pop_ball, flip, delayed_kickoff, kickoff

class State:
    def run(self, agent: BotCommandAgent) -> None:
        pass


class KickoffSelector(State):
    def __init__(self) -> None:
        self.diag = False
        self.middle = False

    def run(self, agent: BotCommandAgent):
        if agent.intent is None:
            if agent.me.velocity.magnitude() < 100:
                self.diag = abs(agent.me.location.x) > 1500
                self.middle = abs(agent.me.location.x) < 100

            if True: # 1v1
                if not self.middle:
                    agent.set_intent(kickoff(agent.me.location))
                else:
                    agent.set_intent(delayed_kickoff())


        if not agent.kickoff_flag:
            agent.state = None




class Dribble(State):
    def __init__(self) -> None:
        pass

    def run(self, agent: BotCommandAgent):
        local_target = agent.me.local(agent.ball.location - agent.me.location)
        if agent.game.time_after_kickoff > 3 and agent.me.location.z < 100 and agent.me.time_on_ground > 0.2 and abs(math.atan2(local_target[1], local_target[0])) < 0.45 and agent.me.location.flat_dist(agent.ball.location) > 2100 and not agent.me.supersonic and not agent.me.airborne:
            agent.set_intent(flip(local_target))
            return
        d_threat = calculate_dribble_threats(agent)
        if agent.me.location.dist(agent.ball.location) < 250 and d_threat < 0.6:
            agent.set_intent(flip(local_target))
            return
        # The ball doesn't have enough upwards velocity to go under the ball. Lift up the ball
        if agent.ball.location.z + agent.ball.velocity.z ** 2 / 1300 < 110:
            offset = math.cos(agent.me.forward.angle_to((agent.ball.location - agent.me.location).flatten())) * agent.me.location.flat_dist(agent.ball.location) - (100 - agent.me.velocity.magnitude() / 10)
            pos = agent.ball.location + sign((agent.me.location - agent.me.left).flatten().dist(agent.ball.location.flatten()) - (agent.me.location + agent.me.left).flatten().dist(agent.ball.location.flatten())) * Vector3(agent.me.forward.y, -agent.me.forward.x, 0).rescale((130 - max(10 - agent.ball.velocity.magnitude() / 20, 0) - (agent.ball.location.z + agent.ball.velocity.z**2 / 1300 - 92.75) * 2 * bool(offset < 15)) * bool(offset >= 0)) + agent.me.forward.flatten().rescale(50)
            if agent.me.location.flat_dist(agent.ball.location) < 130 or math.cos(agent.me.forward.angle_to((agent.ball.location - agent.me.location).flatten())) >= 0:
                agent.controller.throttle = sign(offset + 30 + (agent.ball.location.z - 92.75) - brake_distance(agent.me.velocity.flatten().magnitude(), agent.ball.velocity.flatten().magnitude(), 3500))
            else:
                agent.controller.throttle = 1
            if agent.me.location.flat_dist(agent.ball.location) > 130:
                handbrake_handler(agent, pos)
            if agent.me.velocity.magnitude() < agent.ball.velocity.magnitude() and math.cos(agent.me.forward.angle_to((agent.ball.location - agent.me.location).flatten())) * agent.me.location.flat_dist(agent.ball.location) > 0 and agent.controller.throttle == -1:
                agent.controller.throttle = 0
            defaultPD(agent, agent.me.local(pos - agent.me.location))
        # If the condition is right, dribble the ball
        elif agent.me.location.flat_dist(agent.ball.location) < 100 and (agent.ball.velocity.magnitude() > 700 and agent.ball.velocity.flatten().angle_to((agent.foe_goal.location - agent.ball.location).flatten()) >= math.pi / 6 or agent.ball.location.z > 120 and (agent.me.location.dist(agent.foe_goal.location) >= agent.opp_closest_ball[0].location.dist(agent.foe_goal.location) or d_threat < 1)):
            # If the ball is dribblable, dribble the ball.
            if d_threat < 1 and distvstime(agent.opp_closest_ball[0].velocity.magnitude(), agent.me.location.flat_dist(agent.foe_goal.location) / 1200, agent.opp_closest_ball[0].boost, 1) >= agent.me.location.flat_dist(agent.foe_goal.location):
                di = 5
                pb = 0
                bp = agent.ball.predict(agent, pb)
                while di != 0 and pb < 5:
                    cb, di = calc_next_bounce(bp[0], bp[1], 92.75 + 18, 650)
                    pb += max(cb, 1 / 60)
                    bp = agent.ball.predict(agent, pb)
                bpos = bp[0].flatten()
                bsv = agent.ball.velocity.flatten()
                pos = bpos - bsv.rescale(cap((70 - bsv.magnitude() / 10) / (1 + abs(bp[1].z) / 500), -70, 70)) + Vector3(-bsv.y, bsv.x, 0).rescale(bsv.angle_to(agent.foe_goal.location - bpos) * 50 / (1 + abs(bp[1].z) / 500) * cap(d_threat - 0.5, 0, 1) / math.pi * sign(Vector3(-bsv.y, bsv.x, 0).dist(agent.foe_goal.location - agent.ball.velocity.flatten()) - Vector3(bsv.y, -bsv.x, 0).dist(agent.foe_goal.location - agent.ball.velocity.flatten())))
                throttle = agent.me.location.flatten().dist(pos) - agent.me.velocity.flatten().magnitude() * pb
                if agent.me.velocity.flat_dist(agent.ball.velocity) < 30 and agent.ball.location.z < 140:
                    agent.controller.throttle = bool(throttle > 0)
                else:
                    agent.controller.throttle = sign(throttle)
                handbrake_handler(agent, pos)
                defaultPD(agent, agent.me.local(pos - agent.me.location))
                
                if (d_threat <= 0.5 or agent.ball.location.dist(agent.foe_goal.location) < 2300) and bool(not agent.me.airborne) >= bool(not agent.opp_closest_ball[0].airborne) and agent.me.location.flat_dist(agent.ball.location) <= 100 and agent.me.velocity.flat_dist(agent.ball.velocity) <= 100:
                    # Flick
                    if True: # Front flick if in front of the net
                        agent.set_intent(flick(True))
                    elif False: # Check if someone can follow up
                        agent.set_intent(flick(False))
                    else:
                        agent.set_intent(pop_ball())
            # Else, catch it when it's falling from the sky
            else:
                agent.state = None
                return
                di = 5
                pb = 0
                bp = agent.ball.predict(agent, pb)
                while di != 0 and pb < 5:
                    cb, di = calc_next_bounce(bp[0], bp[1], 92.75 + 18, 650)
                    pb += max(cb, 1 / 60)
                    bp = agent.ball.predict(agent, pb)
                bpos = bp[0].flatten()
                bsv = bp[1].flatten()
                hbv = (bsv - agent.me.velocity.flatten()).magnitude() * math.cos((bsv - agent.me.velocity.flatten()).angle_to(Vector3(-agent.me.forward.y, agent.me.forward.x, 0)))
                vbv = bsv.magnitude() * math.cos(bsv.angle_to(agent.me.forward))
                redirect_angle = bsv.angle_to(agent.foe_goal.location - bpos) * sign(Vector3(-bsv.y, bsv.x, 0).dist(agent.foe_goal.location - agent.ball.location.flatten()) - Vector3(bsv.y, -bsv.x, 0).dist(agent.foe_goal.location - agent.ball.location.flatten()))
                ang_limit = math.pi * cap((agent.ball.velocity.flatten().magnitude() - 300) / 700, 0, 1)
                bounce_angle = cap(redirect_angle, -ang_limit, ang_limit)
                v_offset = min((vbv - 1000 * math.cos(bounce_angle)) / 10 / (1 + bp[1].z / 500), 80)
                h_offset = min((hbv + 1000 * math.sin(bounce_angle)) / 10 / (1 + bp[1].z / 500), 70)
                pos = bpos + agent.me.forward.rescale(v_offset) + Vector3(-agent.me.forward.y, agent.me.forward.x, 0).rescale(h_offset)
                agent.controller.throttle = sign(agent.me.location.flatten().dist(pos) - agent.me.velocity.flatten().magnitude() * pb)
                handbrake_handler(agent, pos)
                defaultPD(agent, agent.me.local(pos - agent.me.location))
        # Get a better position
        else:
            di = 5
            pb = 0
            bp = agent.ball.predict(agent, pb)
            while di != 0 and pb < 5:
                cb, di = calc_next_bounce(bp[0], bp[1], 92.75 + 18, 650)
                pb += max(cb, 1 / 60)
                bp = agent.ball.predict(agent, pb)
            bpos = bp[0].flatten()
            bsv = agent.ball.velocity.flatten()
            tl1 = bpos + Vector3(-bsv.y, bsv.x, 0).rescale(cap(cap(400 * bsv.angle_to(agent.foe_goal.location - agent.ball.location.flatten()) * sign(Vector3(-bsv.y, bsv.x, 0).dist(agent.foe_goal.location - agent.ball.location.flatten()) - Vector3(bsv.y, -bsv.x, 0).dist(agent.foe_goal.location - agent.ball.location.flatten())), -60, 60), -bsv.angle_to((agent.foe_goal.location - agent.me.location).flatten()) * 100, bsv.angle_to((agent.foe_goal.location - agent.me.location).flatten()) * 100))
            tl2 = agent.me.location + (bpos - agent.foe_goal.location)
            tlopttime = (math.pi / 2 - agent.me.forward.angle_to(agent.me.velocity)) / 2 + agent.me.velocity.magnitude() / 3500
            tlopt = distvstime(0, max(pb - tlopttime, 0), 0, 1) >= surface_pos(agent.me.location + agent.me.velocity.rescale(agent.me.velocity.magnitude() * (math.pi / 2 - agent.me.forward.angle_to(agent.me.velocity)) / 2 + agent.me.velocity.magnitude()**2 / 7000)).dist(surface_pos(tl1))
            pos = tl2 * bool(tlopt) + tl1 * bool(not tlopt)
            if agent.ball.location.z + agent.ball.velocity.z**2 / 1300 >= 140 or agent.ball.velocity.flatten().angle_to((agent.foe_goal.location - agent.ball.location).flatten()) >= math.pi / 6:
                agent.controller.throttle = sign(agent.me.location.flatten().dist(pos) - agent.me.velocity.flatten().magnitude() * pb)
            else:
                agent.controller.throttle = sign((agent.me.location - agent.ball.location).flatten().magnitude() - 90 + bsv.magnitude() - agent.me.velocity.flatten().magnitude())
            handbrake_handler(agent, pos)
            agent.controller.boost = 210 > (agent.me.location - agent.ball.location).flatten().magnitude() > 90 and agent.me.velocity.magnitude() <= 2290 and agent.ball.location.z + agent.ball.velocity.z**2 / 1300 < 140 and agent.me.forward.angle_to(agent.ball.location - agent.me.location) < math.pi / 2
            defaultPD(agent, agent.me.local(pos - agent.me.location))




