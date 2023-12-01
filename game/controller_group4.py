from kesslergame import KesslerController
from typing import Dict, Tuple
from skfuzzy import control as ctrl
from skfuzzy import trimf
import numpy as np
import math


class FSController(KesslerController):
    def __init__(self, chromosome = None):
        
        if chromosome is None:
            chromosome = [0.12467835912790237, 0.6585295075987507, 0.10956233934658344, 0.6257074740697369, 0.3136768180499757, 0.6664552106000395, 0.7702516639022675] 

        self.eval_frames = 0

        angle_rng = np.arange(-180, 180, 0.01)
        thrust_rng = np.arange(-480, 480, 0.01)
        turn_rate_rng = np.arange(-200, 200, 0.01)
        dist_rng = np.arange(0, 500, 0.01)
        bool_rng = np.arange(-1, 1, 0.01)

        # determinant
        closest_dist = ctrl.Antecedent(dist_rng, 'closest_dist')

        # evade & target system
        evade_delta = ctrl.Antecedent(angle_rng, 'evade_delta')
        bullet_delta = ctrl.Antecedent(angle_rng, 'bullet_delta')
        thrust = ctrl.Consequent(thrust_rng, 'thrust')
        turn_rate = ctrl.Consequent(turn_rate_rng, 'turn_rate')
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
        closest_dist_genes, [bullet_delta_gene, evade_delta_gene, thrust_gene, turn_rate_gene] = chromosome[:3], chromosome[3:]

        closest_dist_peaks = [
            self.gene_convert(closest_dist_genes[0], 70, 100),
            self.gene_convert(closest_dist_genes[1], 100, 200),
            self.gene_convert(closest_dist_genes[2], 250, 350),
        ]
        bullet_delta_peak = self.gene_convert(bullet_delta_gene, 5, 10)
        evade_delta_peak = self.gene_convert(evade_delta_gene, 80, 100)
        turn_rate_peak = self.gene_convert(turn_rate_gene, 100, 150)
        thrust_peak = self.gene_convert(thrust_gene, 200, 280)

        self.threat_dist = closest_dist_peaks[0]

        mf_5(closest_dist, closest_dist_peaks, 0, 500)
        mf_5(bullet_delta, [-bullet_delta_peak, 0, bullet_delta_peak], -180, 180)
        mf_5(evade_delta, [-evade_delta_peak, 0, evade_delta_peak], -180, 180)
        mf_5(thrust, [-thrust_peak, 0, thrust_peak], -480, 480)
        mf_5(turn_rate, [-turn_rate_peak, 0, turn_rate_peak], -200, 200)
        mf_2(fire, -1, 1)

        # rules
        rules_evade = [
            ctrl.Rule(evade_delta['M'], fire['H']),
            ctrl.Rule(~evade_delta['M'], fire['L']),

            ctrl.Rule((evade_delta['H'] | evade_delta['L']), thrust['MH']),
            ctrl.Rule((evade_delta['MH'] | evade_delta['ML']), thrust['ML']),
            ctrl.Rule(evade_delta['M'], thrust['ML']),
            
            ctrl.Rule(evade_delta['L'], turn_rate['H']),
            ctrl.Rule(evade_delta['H'], turn_rate['L']),
            ctrl.Rule(evade_delta['ML'], turn_rate['L']),
            ctrl.Rule(evade_delta['MH'], turn_rate['H']),
            ctrl.Rule(evade_delta['M'], turn_rate['ML']),    
        ]

        rules_target = [
            ctrl.Rule(bullet_delta['M'] & (closest_dist['L'] | closest_dist['ML']), fire['H']),
            ctrl.Rule(~(bullet_delta['M'] & (closest_dist['L'] | closest_dist['ML'])), fire['L']),
            
            ctrl.Rule(closest_dist['L'], thrust['L']),
            ctrl.Rule(closest_dist['ML'], thrust['M']),
            ctrl.Rule(closest_dist['M'], thrust['MH']),
            ctrl.Rule(closest_dist['MH'], thrust['MH']),
            ctrl.Rule(closest_dist['H'], thrust['H']),

            ctrl.Rule(bullet_delta['H'], turn_rate['H']),
            ctrl.Rule(bullet_delta['L'], turn_rate['L']),
            ctrl.Rule(bullet_delta['ML'], turn_rate['ML']),
            ctrl.Rule(bullet_delta['MH'], turn_rate['MH']),
            ctrl.Rule(bullet_delta['M'], turn_rate['M']),
        ]

        self.evade_control = ctrl.ControlSystem(rules_evade)
        self.target_control = ctrl.ControlSystem(rules_target)

    # convert gene [0, 1] to [min_val, max_val]
    def gene_convert(self, gene, min_val, max_val):
        return min_val + (max_val - min_val) * gene
    
    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
        """
        Method processed each time step by this controller.
        """

        [a_closest_wrapped] = self.get_closest_n_asteroids(ship_state, game_state, 1, True)
        closest_dist = a_closest_wrapped['dist']

        is_threat = closest_dist <= self.threat_dist

        control = None
        inputs = {}

        if is_threat:
            a_angle = self.ast_delta(ship_state, a_closest_wrapped['aster']['position'], game_state['map_size'])
            
            control = self.evade_control
            inputs = {
                'evade_delta': a_angle,
            }
        else:
            n_closest_nowrap = self.get_closest_n_asteroids(ship_state, game_state, 3, False)
            aster = self.get_smallest_asteroid(n_closest_nowrap)
            target_angle = self.target_angle(ship_state, aster)

            control = self.target_control
            inputs = {
                'closest_dist': aster['dist'],
                'bullet_delta': target_angle,
            }
        sim = ctrl.ControlSystemSimulation(control, flush_after_run=1)
        sim.inputs(inputs)
        
        sim.compute()

        thrust = sim.output['thrust']
        turn_rate = sim.output['turn_rate']
        if turn_rate >= 180:
            turn_rate = 180
        elif turn_rate <= -180:
            turn_rate = -180
        
        fire = sim.output['fire'] >= 0 # if not is_threat else False
        # print(is_threat)
        # print(f"CA: {ca}, TA: {target_angle}, CT: {coll_time} -> thrust: {thrust}, turn_rate: {turn_rate}, fire: {fire}")

        self.eval_frames +=1

        return thrust, turn_rate, fire
    
    def rel_asteroid_pos(self, ship_pos, asteroid_pos, map_size, wrap=False):    
        sx, sy = ship_pos
        ax, ay = asteroid_pos
        w, h = map_size

        x1, y1 = ax - sx, ay - sy

        if not wrap:
            return x1, y1
        
        x2, y2 = ax - sx - w, ay - sy - h
        return x1 if abs(x1) < abs(x2) else x2, y1 if abs(y1) < abs(y2) else y2
        
    def get_closest_n_asteroids(self, ship_state, game_state, n, wrap):
        ship_pos = ship_state['position']
        map_size = game_state['map_size']
        def rel_dist(a_pos, wrap):
            rx, ry = self.rel_asteroid_pos(ship_pos, a_pos, map_size, wrap)
            return math.sqrt(rx**2 + ry**2)

        a_list = [{ 'aster': a, 'dist': rel_dist(a['position'], True) } for a in game_state['asteroids']]
        return sorted(a_list, key=lambda a: a['dist'])[:n]

    # get smallest asteroid out of n closest asteroids.
    def get_smallest_asteroid(self, n_closest_asteroids):
        return sorted(n_closest_asteroids, key=lambda a: a['aster']['size'])[0]
    
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
    
    def ast_delta(self, ship_state, a_pos, map_size):
        rel_ax, rel_ay = self.rel_asteroid_pos(ship_state["position"], a_pos, map_size)
        angle = (360 + math.atan2(rel_ay, rel_ax) * 180 / math.pi) % 360

        delta = angle - ship_state["heading"]
        if delta > 180:
            delta -= 360

        return delta

    @property
    def name(self) -> str:
        return "FSController"
