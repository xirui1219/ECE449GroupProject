# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from kesslergame import KesslerController
from typing import Dict, Tuple
import math
from math import sqrt
from math import atan2, pi


debug = False


class TestControllerEvade(KesslerController):
    def __init__(self):
        self.eval_frames = 0

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool, bool]:
        """
        Method processed each time step by this controller.
        """
        thrust = 150 - ship_state['speed']

        def dist(ast):
            rel_ax, rel_ay = self.rel_asteroid_pos(ship_state['position'], ast['position'], game_state['map_size'])
            return sqrt((rel_ax) ** 2 + (rel_ay) ** 2)

        n_closest = sorted(game_state['asteroids'], key=dist)[:2]
        ca = self.clear_angle(ship_state, game_state['map_size'], n_closest)
        turn_rate = ca - ship_state['heading']
        if abs(turn_rate) > 180:
            diff = 360 - abs(turn_rate)
            turn_rate = diff if turn_rate < 0 else -diff

        coll_time_array = []
        for a in n_closest:
            # return the distance to asteroid
            # get velocity
            # v = d/t --> t = d/v
            distance = dist(a)
            rel_vel = self.asteroid_vel(ship_state['velocity'], a['velocity'])
            coll_time = abs(distance / rel_vel)
            coll_time_array.append(coll_time)

        dict = self.closest_asteroid(ship_state, game_state)
        aster = self.smallest_asteroid(dict)
        target_angle = self.target_angle(ship_state, aster)

        fire = False

        self.eval_frames += 1

        return thrust, turn_rate, fire

    def closest_asteroid(self, ship_state, game_state):

        ship_pos_x = ship_state["position"][0]
        ship_pos_y = ship_state["position"][1]

        closest_3_asteroid = [None] * 3
        closest_3_asteroid[0] = dict(aster=None, dist=999999999.9)
        closest_3_asteroid[1] = dict(aster=None, dist=999999999.9)
        closest_3_asteroid[2] = dict(aster=None, dist=999999999.9)

        for a in game_state["asteroids"]:
            # Loop through all asteroids, find minimum Eudlidean distance
            curr_dist = math.sqrt((ship_pos_x - a["position"][0]) ** 2 + (ship_pos_y - a["position"][1]) ** 2)
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
        if debug:
            print(str(closest_3_asteroid[0]["dist"]))
            print(str(closest_3_asteroid[1]["dist"]))
            print(str(closest_3_asteroid[2]["dist"]))
            print()
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

        print(shooting_theta)
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

        return ca

    @property
    def name(self) -> str:
        return "Test Controller"
