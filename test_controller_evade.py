# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from src.kesslergame import KesslerController
from typing import Dict, Tuple
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import numpy as np
import matplotlib as plt

class TestController(KesslerController):
    # def __init__(self):
    #     self.eval_frames = 0
    #
    # def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
    #     """
    #     Method processed each time step by this controller.
    #     """
    #
    #     thrust = 0
    #     turn_rate = 90
    #     fire = True
    #     self.eval_frames +=1
    #
    #     return thrust, turn_rate, fire
    def __init__(self):
        self.eval_frames = 0  # What is this?

        # self.targeting_control is the targeting rulebase, which is static in this controller.
        # Declare variables
        bullet_time = ctrl.Antecedent(np.arange(0, 1.0, 0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1 * math.pi, math.pi, 0.1), 'theta_delta')  # Radians due to Python
        ship_turn = ctrl.Consequent(np.arange(-180, 180, 1), 'ship_turn')  # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1, 1, 0.1), 'ship_fire')

        # Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe, [0, 0, 0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0, 0.05, 0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe, 0.0, 0.1)

        # Declare fuzzy sets for theta_delta (degrees of turn needed to reach the calculated firing angle)
        theta_delta['NL'] = fuzz.zmf(theta_delta.universe, -1 * math.pi / 3, -1 * math.pi / 6)
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-1 * math.pi / 3, -1 * math.pi / 6, 0])
        theta_delta['Z'] = fuzz.trimf(theta_delta.universe, [-1 * math.pi / 6, 0, math.pi / 6])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe, [0, math.pi / 6, math.pi / 3])
        theta_delta['PL'] = fuzz.smf(theta_delta.universe, math.pi / 6, math.pi / 3)

        # Declare fuzzy sets for the ship_turn consequent; this will be returned as turn_rate.
        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180, -180, -30])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-90, -30, 0])
        ship_turn['Z'] = fuzz.trimf(ship_turn.universe, [-30, 0, 30])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [0, 30, 90])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [30, 180, 180])

        # Declare singleton fuzzy sets for the ship_fire consequent; -1 -> don't fire, +1 -> fire; this will be  thresholded
        #   and returned as the boolean 'fire'
        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1, -1, 0.0])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [0.0, 1, 1])

        # Declare each fuzzy rule
        rule1 = ctrl.Rule(bullet_time['L'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N']))
        rule2 = ctrl.Rule(bullet_time['L'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        rule3 = ctrl.Rule(bullet_time['L'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule4 = ctrl.Rule(bullet_time['L'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule5 = ctrl.Rule(bullet_time['L'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N']))
        rule6 = ctrl.Rule(bullet_time['M'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N']))
        rule7 = ctrl.Rule(bullet_time['M'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        rule8 = ctrl.Rule(bullet_time['M'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule9 = ctrl.Rule(bullet_time['M'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule10 = ctrl.Rule(bullet_time['M'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N']))
        rule11 = ctrl.Rule(bullet_time['S'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['Y']))
        rule12 = ctrl.Rule(bullet_time['S'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        rule13 = ctrl.Rule(bullet_time['S'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule14 = ctrl.Rule(bullet_time['S'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule15 = ctrl.Rule(bullet_time['S'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['Y']))

        # DEBUG
        # bullet_time.view()
        # theta_delta.view()
        # ship_turn.view()
        # ship_fire.view()

        # Declare the fuzzy controller, add the rules
        # This is an instance variable, and thus available for other methods in the same object. See notes.
        # self.targeting_control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15])

        self.targeting_control = ctrl.ControlSystem()
        self.targeting_control.addrule(rule1)
        self.targeting_control.addrule(rule2)
        self.targeting_control.addrule(rule3)
        self.targeting_control.addrule(rule4)
        self.targeting_control.addrule(rule5)
        self.targeting_control.addrule(rule6)
        self.targeting_control.addrule(rule7)
        self.targeting_control.addrule(rule8)
        self.targeting_control.addrule(rule9)
        self.targeting_control.addrule(rule10)
        self.targeting_control.addrule(rule11)
        self.targeting_control.addrule(rule12)
        self.targeting_control.addrule(rule13)
        self.targeting_control.addrule(rule14)
        self.targeting_control.addrule(rule15)

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
        """
        Method processed each time step by this controller.
        """
        # These were the constant actions in the basic demo, just spinning and shooting.
        # thrust = 0 <- How do the values scale with asteroid velocity vector?
        # turn_rate = 90 <- How do the values scale with asteroid velocity vector?

        # Answers: Asteroid position and velocity are split into their x,y components in a 2-element ?array each.
        # So are the ship position and velocity, and bullet position and velocity.
        # Units appear to be meters relative to origin (where?), m/sec, m/sec^2 for thrust.
        # Everything happens in a time increment: delta_time, which appears to be 1/30 sec; this is hardcoded in many places.
        # So, position is updated by multiplying velocity by delta_time, and adding that to position.
        # Ship velocity is updated by multiplying thrust by delta time.
        # Ship position for this time increment is updated after the the thrust was applied.

        # My demonstration controller does not move the ship, only rotates it to shoot the nearest asteroid.
        # Goal: demonstrate processing of game state, fuzzy controller, intercept computation
        # Intercept-point calculation derived from the Law of Cosines, see notes for details and citation.

        # Find the closest asteroid (disregards asteroid velocity)
        ship_pos_x = ship_state["position"][0]  # See src/kesslergame/ship.py in the KesslerGame Github
        ship_pos_y = ship_state["position"][1]
        closest_asteroid = None

        for a in game_state["asteroids"]:
            # Loop through all asteroids, find minimum Eudlidean distance
            curr_dist = math.sqrt((ship_pos_x - a["position"][0]) ** 2 + (ship_pos_y - a["position"][1]) ** 2)
            if closest_asteroid is None:
                # Does not yet exist, so initialize first asteroid as the minimum. Ugh, how to do?
                closest_asteroid = dict(aster=a, dist=curr_dist)

            else:
                # closest_asteroid exists, and is thus initialized.
                if closest_asteroid["dist"] > curr_dist:
                    # New minimum found
                    closest_asteroid["aster"] = a
                    closest_asteroid["dist"] = curr_dist

        # closest_asteroid is now the nearest asteroid object.
        # Calculate intercept time given ship & asteroid position, asteroid velocity vector, bullet speed (not direction).
        # Based on Law of Cosines calculation, see notes.

        # Side D of the triangle is given by closest_asteroid.dist. Need to get the asteroid-ship direction
        #    and the angle of the asteroid's current movement.
        # REMEMBER TRIG FUNCTIONS ARE ALL IN RADAINS!!!

        asteroid_ship_x = ship_pos_x - closest_asteroid["aster"]["position"][0]
        asteroid_ship_y = ship_pos_y - closest_asteroid["aster"]["position"][1]

        asteroid_ship_theta = math.atan2(asteroid_ship_y, asteroid_ship_x)

        asteroid_direction = math.atan2(closest_asteroid["aster"]["velocity"][1], closest_asteroid["aster"]["velocity"][
            0])  # Velocity is a 2-element array [vx,vy].
        my_theta2 = asteroid_ship_theta - asteroid_direction
        cos_my_theta2 = math.cos(my_theta2)
        # Need the speeds of the asteroid and bullet. speed * time is distance to the intercept point
        asteroid_vel = math.sqrt(
            closest_asteroid["aster"]["velocity"][0] ** 2 + closest_asteroid["aster"]["velocity"][1] ** 2)
        bullet_speed = 800  # Hard-coded bullet speed from bullet.py

        # Determinant of the quadratic formula b^2-4ac
        targ_det = (-2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) ** 2 - (
                    4 * (asteroid_vel ** 2 - bullet_speed ** 2) * closest_asteroid["dist"])

        # Combine the Law of Cosines with the quadratic formula for solve for intercept time. Remember, there are two values produced.
        intrcpt1 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) + math.sqrt(targ_det)) / (
                    2 * (asteroid_vel ** 2 - bullet_speed ** 2))
        intrcpt2 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) - math.sqrt(targ_det)) / (
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
        intrcpt_x = closest_asteroid["aster"]["position"][0] + closest_asteroid["aster"]["velocity"][0] * bullet_t
        intrcpt_y = closest_asteroid["aster"]["position"][1] + closest_asteroid["aster"]["velocity"][1] * bullet_t

        my_theta1 = math.atan2((intrcpt_y - ship_pos_y), (intrcpt_x - ship_pos_x))

        # Lastly, find the difference betwwen firing angle and the ship's current orientation. BUT THE SHIP HEADING IS IN DEGREES.
        shooting_theta = my_theta1 - ((math.pi / 180) * ship_state["heading"])

        # Wrap all angles to (-pi, pi)
        shooting_theta = (shooting_theta + math.pi) % (2 * math.pi) - math.pi

        # Pass the inputs to the rulebase and fire it
        shooting = ctrl.ControlSystemSimulation(self.targeting_control, flush_after_run=1)

        shooting.input['bullet_time'] = bullet_t
        shooting.input['theta_delta'] = shooting_theta

        shooting.compute()

        # Get the defuzzified outputs
        turn_rate = shooting.output['ship_turn']

        if shooting.output['ship_fire'] >= 0:
            fire = True
        else:
            fire = False

        # And return your three outputs to the game simulation. Controller algorithm complete.
        thrust = 0.0

        self.eval_frames += 1
        self.calculateCollisionTime(ship_state,game_state)
        # DEBUG
        print(thrust, bullet_t, shooting_theta, turn_rate, fire)

        return thrust, turn_rate, fire
    def calculateCollisionTime(self, ship_state: Dict, game_state: Dict):
        # iterate through asteroids
        # calculate the time
        # return the 3 shortest times
        def dist(ast):
            rel_ax, rel_ay = self.rel_asteroid_pos(ship_state['position'], ast['position'], game_state['map_size'])
            return math.sqrt((rel_ax) ** 2 + (rel_ay) ** 2)

        n_closest = sorted(game_state['asteroids'], key=dist)[:3]
        coll_time_array = []
        for a in n_closest:
            #return the distance to asteroid
            #get velocity
            #v = d/t --> t = d/v
            distance = dist(a)
            rel_vel = self.asteroid_vel(ship_state['velocity'],a['velocity'])
            coll_time = abs(distance/rel_vel)
            coll_time_array.append(coll_time)




    def rel_asteroid_pos(self,ship_pos, asteroid_pos, map_size):
        sx, sy = ship_pos
        ax, ay = asteroid_pos
        w, h = map_size

        x1, y1 = ax - sx, ay - sy
        x2, y2 = ax - sx - w, ay - sy - h
        rel_pos = x1 if abs(x1) < abs(x2) else x2, y1 if abs(y1) < abs(y2) else y2

        return rel_pos

    def asteroid_vel(self,ship_vel, asteroid_vel):
        sx, sy = ship_vel
        ax, ay = asteroid_vel

        vel_x = sx-ax
        vel_y = sy-ay
        rel_vel = math.sqrt(vel_x**2 + vel_y**2)

        return rel_vel



    @property
    def name(self) -> str:
        return "Test Controller"
