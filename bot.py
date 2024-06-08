
from util.objects import *
from util.routines import *
from util.states import *
from util.tools import *


class Queso(BotCommandAgent):
    def run(agent):
        if agent.state is None and agent.intent is None:
            if agent.kickoff_flag:
                agent.state = KickoffSelector()

            else:
                if agent.ball.location.dist(agent.foe_goal.location) > 1000 and agent.ball.location.dist(agent.friend_goal.location) > 1000 and (agent.ball.location.z + agent.ball.velocity.z ** 2 / 1300 > 130 or agent.opp_closest_ball[0].location.dist(agent.foe_goal.location)< agent.me.location.dist(agent.foe_goal.location)):
                    Dribble.run(Dribble, agent)
                
                else:
                    targets = {
                        "score": (agent.foe_goal.left_post, agent.foe_goal.right_post),
                        "away": (agent.friend_goal.right_post, agent.friend_goal.left_post)
                    }

                    hits = find_hits(agent,targets)
                    if len(hits["score"]) > 0:
                        agent.set_intent(hits["score"][0])
                        return
                    
                    if len(hits["away"]) > 0:
                        agent.set_intent(hits["away"][0])
                        return


