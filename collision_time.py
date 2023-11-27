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
