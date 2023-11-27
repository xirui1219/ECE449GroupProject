# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

from kesslergame import KesslerController
from typing import Dict, Tuple
from angle_clear import clear_angle, rel_asteroid_pos
from math import sqrt

class TestController(KesslerController):
    def __init__(self):
        self.eval_frames = 0

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool, bool]:
        """
        Method processed each time step by this controller.
        """

        thrust = 150 - ship_state['speed']

        def dist(ast):
            rel_ax, rel_ay = rel_asteroid_pos(ship_state['position'], ast['position'], game_state['map_size'])
            return sqrt((rel_ax)**2 + (rel_ay)**2)
        
        n_closest = sorted(game_state['asteroids'], key=dist)[:2]
        ca = clear_angle(ship_state, game_state['map_size'], n_closest)
        turn_rate = ca - ship_state['heading']
        if abs(turn_rate) > 180:
            diff = 360 - abs(turn_rate)
            turn_rate = diff if turn_rate < 0 else -diff

        fire = False
        drop_mine = False

        self.eval_frames +=1

        return thrust, turn_rate, fire, drop_mine

    @property
    def name(self) -> str:
        return "Test Controller"
