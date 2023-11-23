# ECE449GroupProject
## Common scales
- Position: (x, y), with (0, 0) at bottom left
  - Velocity: (vx, vy), updated position: (x + vx, y + vy)
- Angle/heading: 0 degrees is East, with angle increasing counterclockwise
  - i.e., 90 N, 180 W, 270 S, 0/360 E

## variables available under `action()`
### `ship_state`
- "is_respawning": boolean
- "position": float tuple, (x, y)
- "velocity": float tuple, (vx, vy)
- "speed": float, sqrt(vx^2 + vy^2)
- "heading": float, angle
- "mass": float, mass
- "radius": float, radius
- "id": int, id
- "team": str, team
- "lives_remaining": int, lives left
(not available in ship.state under game_state)
- "bullets_remaining": int, bullets remaining
- "mines_remaining": int, mines remaining
- "can_fire": boolean, can fire
- "fire_rate": float, fire rate (bullets per second)
- "thrust_range": float tuple, (min, max) thrust values allowed in ms^-2
- "turn_rate_range": float tuple, (min, max) turn rate values allowed in degrees s^-1
- "max_speed": float, maximum speed of ship in ms^-1
- "drag": float, counter-acceleration to thrust applied in ms^-2

### `game_state`
- 'asteroids': list of asteroid.state for all asteroids
  - each asteroid.state has:
    - 'position': float tuple, (x, y)
    - 'velocity': float tuple, (vx, vy)
    - 'size': int, 1-4 <size will decrease by 1 when shot, and asteroids of size 1 disappear after being shot>
    - 'mass': float, mass
    - 'radius': float, collision radius
- 'ships': ship.state for ship in liveships
  - see top half of ship_state
- 'bullets': list of bullet.state for all bullets
  - each bullet.state has:
    - "position": float tuple, (x, y)
    - "velocity": float tuple, (vx, vy)
    - "heading": float, angle
    - "mass": float, mass
- 'mines': mine.state for mine in mines (not relevant for assignment)
- 'map_size': scenario.map_size,
- 'time': sim_time,
- 'time_step': step
