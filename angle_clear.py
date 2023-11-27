from math import atan2, pi

# returns asteroid coordinates relative to ship, based on map size
def rel_asteroid_pos(ship_pos, asteroid_pos, map_size):
    sx, sy = ship_pos
    ax, ay = asteroid_pos
    w, h = map_size
    
    x1, y1 = ax - sx, ay - sy
    x2, y2 = ax - sx - w, ay - sy - h
    rel_pos = x1 if abs(x1) < abs(x2) else x2, y1 if abs(y1) < abs(y2) else y2

    return rel_pos

# returns heading on which there are no asteroids in the line.
# heading = midpoint of largest angle distance between two asteroids.
def clear_angle(ship_state, map_size, n_closest_asteroids):
    angles = []
    n = len(n_closest_asteroids)

    for ast in n_closest_asteroids:
        rel_ax, rel_ay = rel_asteroid_pos(ship_state['position'], ast['position'], map_size)
        
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
            ca = (cur + delta/2) % 360
    
    if ca is None:
        ca = (angles[0] + 180) % 360 
    
    return ca
        
    