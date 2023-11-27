from kesslergame import KesslerController
from typing import Dict, Tuple
from skfuzzy import control as ctrl
from skfuzzy import trimf
import numpy as np
import math
from math import sqrt
from math import atan2, pi

class FSController(KesslerController):
    def __init__(self):
        self.eval_frames = 0

        angle_rng = np.arange(-180, 180, 0.01)
        thrust_rng = np.arange(-480, 480, 0.01)
        time_rng = np.arange(0, 4, 0.01)
        bool_rng = np.arange(-1, 1, 0.01)

        bullet_delta = ctrl.Antecedent(angle_rng, 'bullet_delta')
        evade_delta = ctrl.Antecedent(angle_rng, 'evade_delta')
        collision_time = ctrl.Antecedent(time_rng, 'collision_time')
        thrust = ctrl.Consequent(thrust_rng, 'thrust')
        turn_rate = ctrl.Consequent(angle_rng, 'turn_rate')
        fire = ctrl.Consequent(bool_rng, 'fire')

        # 3 peaks
        def mf_5(var, peaks, min_val, max_val):
            var['L'] = trimf(var.universe, [min_val, min_val, peaks[0]])
            var['H'] = trimf(var.universe, [peaks[-1], max_val, max_val])

            labels = ['ML', 'M', 'MH']
            prev = min_val
            for i, l in enumerate(labels):
                top = peaks[i + 1] if i < len(peaks) - 1 else max_val
                var[l] = trimf(var.universe, [prev, peaks[i], top])
                prev = peaks[i]
        
        def mf_3(var, peak, min_val, max_val):
            var['L'] = trimf(var.universe, [min_val, min_val, peak])
            var['M'] = trimf(var.universe, [min_val, peak, max_val])
            var['H'] = trimf(var.universe, [peak, max_val, max_val])
        
        def mf_2(var, min_val, max_val):
            var['L'] = trimf(var.universe, [min_val, min_val, max_val])
            var['H'] = trimf(var.universe, [min_val, max_val, max_val])

        # assigning membership
        mf_5(bullet_delta, [-90, 0, 90], -180, 180)
        mf_5(evade_delta, [-90, 0, 90], -180, 180)
        mf_3(collision_time, 2, 0, 4)
        mf_5(thrust, [-240, 0, 240], -480, 480)
        mf_5(turn_rate, [-90, 0, 90], -180, 180)
        mf_2(fire, -1, 1)

        # rules
        rules = [
            ctrl.Rule(bullet_delta['M'], fire['H']),
            ctrl.Rule(~bullet_delta['M'], fire['L']),

            ctrl.Rule(collision_time['L'] & evade_delta['H'], (thrust ['L'], turn_rate['H'])),
            ctrl.Rule(collision_time['L'] & evade_delta['L'], (thrust ['L'], turn_rate['L'])),
            ctrl.Rule(collision_time['L'] & evade_delta['ML'], (thrust ['MH'], turn_rate['ML'])),
            ctrl.Rule(collision_time['L'] & evade_delta['MH'], (thrust ['MH'], turn_rate['MH'])),
            ctrl.Rule(collision_time['L'] & evade_delta['M'], (thrust ['H'], turn_rate['M'])),    
            
            # ctrl.Rule(collision_time['M'] & evade_delta['H'], (thrust ['ML'], turn_rate['M'])),
            # ctrl.Rule(collision_time['M'] & evade_delta['L'], (thrust ['ML'], turn_rate['M'])),
            # ctrl.Rule(collision_time['M'] & evade_delta['ML'], (thrust ['M'], turn_rate['ML'])),
            # ctrl.Rule(collision_time['M'] & evade_delta['MH'], (thrust ['M'], turn_rate['MH'])),
            # ctrl.Rule(collision_time['M'] & evade_delta['M'], (thrust ['MH'], turn_rate['M'])),

            ctrl.Rule(collision_time['M'] & bullet_delta['H'], (thrust ['ML'], turn_rate['H'])),
            ctrl.Rule(collision_time['M'] & bullet_delta['L'], (thrust ['ML'], turn_rate['L'])),
            ctrl.Rule(collision_time['M'] & bullet_delta['ML'], (thrust ['MH'], turn_rate['ML'])),
            ctrl.Rule(collision_time['M'] & bullet_delta['MH'], (thrust ['MH'], turn_rate['MH'])),
            ctrl.Rule(collision_time['M'] & bullet_delta['M'], (thrust ['MH'], turn_rate['M'])),

            ctrl.Rule(collision_time['H'] & bullet_delta['H'], (thrust ['ML'], turn_rate['H'])),
            ctrl.Rule(collision_time['H'] & bullet_delta['L'], (thrust ['ML'], turn_rate['L'])),
            ctrl.Rule(collision_time['H'] & bullet_delta['ML'], (thrust ['H'], turn_rate['ML'])),
            ctrl.Rule(collision_time['H'] & bullet_delta['MH'], (thrust ['H'], turn_rate['MH'])),
            ctrl.Rule(collision_time['H'] & bullet_delta['M'], (thrust ['H'], turn_rate['M'])),
    ]

        self.control = ctrl.ControlSystem(rules)

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
        """
        Method processed each time step by this controller.
        """

        def dist(ast):
            rel_ax, rel_ay = self.rel_asteroid_pos(ship_state['position'], ast['position'], game_state['map_size'])
            return sqrt((rel_ax) ** 2 + (rel_ay) ** 2)

        n_closest = sorted(game_state['asteroids'], key=dist)[:3]
        ca = self.clear_angle(ship_state, game_state['map_size'], n_closest[:2])

        coll_time = self.calculateCollTime(ship_state,game_state, n_closest)
        dict = self.closest_asteroid(ship_state, game_state)
        aster = self.smallest_asteroid(dict)
        target_angle = self.target_angle(ship_state, aster)

        print(target_angle)
        
        sim = ctrl.ControlSystemSimulation(self.control, flush_after_run=1)
        sim.inputs({
            'bullet_delta': target_angle,
            'evade_delta': ca,
            'collision_time': coll_time,
        })
        
        sim.compute()

        thrust = sim.output['thrust']
        turn_rate = sim.output['turn_rate']
        fire = sim.output['fire'] >= 0

        self.eval_frames +=1

        return thrust, turn_rate, fire
    
    def calculateCollTime(self, ship_state, game_state, n_closest):
        def dist(ast):
            rel_ax, rel_ay = self.rel_asteroid_pos(ship_state['position'], ast['position'], game_state['map_size'])
            return sqrt((rel_ax) ** 2 + (rel_ay) ** 2)

        coll_time_array = []
        for a in n_closest:
            # return the distance to asteroid
            # get velocity
            # v = d/t --> t = d/v
            distance = dist(a)
            rel_vel = self.asteroid_vel(ship_state['velocity'], a['velocity'])
            coll_time = abs(distance / rel_vel)
            coll_time_array.append(coll_time)
        coll_time_array.sort()
        return coll_time_array[0]

    def closest_asteroid(self, ship_state, game_state):

        ship_pos_x = ship_state["position"][0]
        ship_pos_y = ship_state["position"][1]

        closest_3_asteroid = [None] * 3
        closest_3_asteroid[0] = dict(aster=None, dist=999999999.9)
        closest_3_asteroid[1] = dict(aster=None, dist=999999999.9)
        closest_3_asteroid[2] = dict(aster=None, dist=999999999.9)

        for a in game_state["asteroids"]:
            # Loop through all asteroids, find minimum Eudlidean distance
            curr_dist = sqrt((ship_pos_x - a["position"][0]) ** 2 + (ship_pos_y - a["position"][1]) ** 2)
            if closest_3_asteroid[0] is None:
                # Does not yet exist, so initialize first asteroid as the minimum.
                closest_3_asteroid[0] = dict(aster=a, dist=curr_dist)

            else:
                # closest_3_asteroid exists, and is thus initialized.
                for i in range(2, -1, -1):
                    # closer than the third but further than the second
                    if closest_3_asteroid[i]["dist"] > curr_dist:
                        if i == 0:
                            for j in range(2, i, -1):
                                closest_3_asteroid[j]["aster"] = closest_3_asteroid[j - 1]["aster"]
                                closest_3_asteroid[j]["dist"] = closest_3_asteroid[j - 1]["dist"]
                            closest_3_asteroid[i]["aster"] = a
                            closest_3_asteroid[i]["dist"] = curr_dist
                            break
                        if closest_3_asteroid[i - 1]["dist"] <= curr_dist:
                            for j in range(2, i, -1):
                                closest_3_asteroid[j]["aster"] = closest_3_asteroid[j - 1]["aster"]
                                closest_3_asteroid[j]["dist"] = closest_3_asteroid[j - 1]["dist"]
                            closest_3_asteroid[i]["aster"] = a
                            closest_3_asteroid[i]["dist"] = curr_dist
                            break

        return closest_3_asteroid

    # return the smallest asteroid or the closest one if size ties
    def smallest_asteroid(self, closest_3_asteroid):
        smallest_asteroid = None
        for asteroid_dict in closest_3_asteroid:
            # Does not yet exist, so initialize first asteroid as the closest.
            if smallest_asteroid is None:
                smallest_asteroid = asteroid_dict

            # candidate with samller size
            if asteroid_dict["aster"]["size"] < smallest_asteroid["aster"]["size"]:
                smallest_asteroid = asteroid_dict
            # candidate with same size but closer to ship
            elif asteroid_dict["aster"]["size"] == smallest_asteroid["aster"]["size"]:
                if asteroid_dict["dist"] < smallest_asteroid["dist"]:
                    smallest_asteroid = asteroid_dict

        return smallest_asteroid

    # method for estimate angle of target
    def target_angle(self, ship_state, target_asteroid):

        # Find the closest asteroid (disregards asteroid velocity)
        ship_pos_x = ship_state["position"][0]  # See src/kesslergame/ship.py in the KesslerGame Github
        ship_pos_y = ship_state["position"][1]

        # closest_asteroid is now the nearest asteroid object.
        # Calculate intercept time given ship & asteroid position, asteroid velocity vector, bullet speed (not direction).
        # Based on Law of Cosines calculation, see notes.

        asteroid_ship_x = ship_pos_x - target_asteroid["aster"]["position"][0]
        asteroid_ship_y = ship_pos_y - target_asteroid["aster"]["position"][1]

        asteroid_ship_theta = math.atan2(asteroid_ship_y, asteroid_ship_x)

        asteroid_direction = math.atan2(target_asteroid["aster"]["velocity"][1], target_asteroid["aster"]["velocity"][
            0])  # Velocity is a 2-element array [vx,vy].
        my_theta2 = asteroid_ship_theta - asteroid_direction
        cos_my_theta2 = math.cos(my_theta2)
        # Need the speeds of the asteroid and bullet. speed * time is distance to the intercept point
        asteroid_vel = math.sqrt(
            target_asteroid["aster"]["velocity"][0] ** 2 + target_asteroid["aster"]["velocity"][1] ** 2)
        bullet_speed = 800  # Hard-coded bullet speed from bullet.py

        # Determinant of the quadratic formula b^2-4ac
        targ_det = (-2 * target_asteroid["dist"] * asteroid_vel * cos_my_theta2) ** 2 - (
                4 * (asteroid_vel ** 2 - bullet_speed ** 2) * target_asteroid["dist"])

        # Combine the Law of Cosines with the quadratic formula for solve for intercept time. Remember, there are two values produced.
        intrcpt1 = ((2 * target_asteroid["dist"] * asteroid_vel * cos_my_theta2) + math.sqrt(targ_det)) / (
                2 * (asteroid_vel ** 2 - bullet_speed ** 2))
        intrcpt2 = ((2 * target_asteroid["dist"] * asteroid_vel * cos_my_theta2) - math.sqrt(targ_det)) / (
                2 * (asteroid_vel ** 2 - bullet_speed ** 2))

        # Take the smaller intercept time, as long as it is positive; if not, take the larger one.
        if intrcpt1 > intrcpt2:
            if intrcpt2 >= 0:
                bullet_t = intrcpt2
            else:
                bullet_t = intrcpt1
        else:
            if intrcpt1 >= 0:
                bullet_t = intrcpt1
            else:
                bullet_t = intrcpt2

        # Calculate the intercept point. The work backwards to find the ship's firing angle my_theta1.
        intrcpt_x = target_asteroid["aster"]["position"][0] + target_asteroid["aster"]["velocity"][0] * bullet_t
        intrcpt_y = target_asteroid["aster"]["position"][1] + target_asteroid["aster"]["velocity"][1] * bullet_t

        my_theta1 = math.atan2((intrcpt_y - ship_pos_y), (intrcpt_x - ship_pos_x))

        # Lastly, find the difference betwwen firing angle and the ship's current orientation. BUT THE SHIP HEADING IS IN DEGREES.
        shooting_theta = my_theta1 - ((math.pi / 180) * ship_state["heading"])

        # Wrap all angles to (-pi, pi)
        shooting_theta = (shooting_theta + math.pi) % (2 * math.pi) - math.pi
        shooting_theta= shooting_theta*180/math.pi

        return shooting_theta

    def rel_asteroid_pos(self, ship_pos, asteroid_pos, map_size):
        sx, sy = ship_pos
        ax, ay = asteroid_pos
        w, h = map_size

        x1, y1 = ax - sx, ay - sy
        x2, y2 = ax - sx - w, ay - sy - h
        rel_pos = x1 if abs(x1) < abs(x2) else x2, y1 if abs(y1) < abs(y2) else y2

        return rel_pos

    def asteroid_vel(self, ship_vel, asteroid_vel):
        sx, sy = ship_vel
        ax, ay = asteroid_vel

        vel_x = sx - ax
        vel_y = sy - ay
        rel_vel = sqrt(vel_x ** 2 + vel_y ** 2)
        return rel_vel

    # returns heading on which there are no asteroids in the line.
    # heading = midpoint of largest angle distance between two asteroids.
    def clear_angle(self,ship_state, map_size, n_closest_asteroids):
        angles = []
        n = len(n_closest_asteroids)

        for ast in n_closest_asteroids:
            rel_ax, rel_ay = self.rel_asteroid_pos(ship_state['position'], ast['position'], map_size)

            angle = (360 + atan2(rel_ay, rel_ax) * 180 / pi) % 360
            angles.append(angle)

        max_delta = 0
        ca = None
        angles = sorted(angles)
        for i in range(n):
            cur, next = angles[i], angles[(i + 1) % n]
            delta = (next - cur + 360) % 360
            if delta > max_delta:
                max_delta = delta
                ca = (cur + delta / 2) % 360

        if ca is None:
            ca = (angles[0] + 180) % 360

        # Lastly, find the difference betwwen firing angle and the ship's current orientation. BUT THE SHIP HEADING IS IN DEGREES.
        delta = ca - ship_state["heading"]
        if delta > 180:
            delta -= 360

        return delta
    
    @property
    def name(self) -> str:
        return "FSController"
