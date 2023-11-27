import math

def closest_asteroid (ship_state, game_state):

    ship_pos_x = ship_state["position"][0] 
    ship_pos_y = ship_state["position"][1]

    closest_3_asteroid = [None] * 3
    closest_3_asteroid[0] = dict(aster = None, dist = 999999999.9)
    closest_3_asteroid[1] = dict(aster = None, dist = 999999999.9)
    closest_3_asteroid[2] = dict(aster = None, dist = 999999999.9)
    
    for a in game_state["asteroids"]:
        #Loop through all asteroids, find minimum Eudlidean distance
        curr_dist = math.sqrt((ship_pos_x - a["position"][0])**2 + (ship_pos_y - a["position"][1])**2)
        if closest_3_asteroid[0] is None :
            # Does not yet exist, so initialize first asteroid as the minimum.
            closest_3_asteroid[0] = dict(aster = a, dist = curr_dist)
            
        else: 
            # closest_3_asteroid exists, and is thus initialized. 
            for i in range (2, -1, -1):
                # closer than the third but further than the second
                if closest_3_asteroid[i]["dist"] > curr_dist:
                    if i == 0:
                        for j in range (2, i, -1):
                            closest_3_asteroid[j]["aster"] = closest_3_asteroid[j-1]["aster"]
                            closest_3_asteroid[j]["dist"] = closest_3_asteroid[j-1]["dist"]
                        closest_3_asteroid[i]["aster"] = a 
                        closest_3_asteroid[i]["dist"] = curr_dist
                        break
                    if closest_3_asteroid [i-1]["dist"] <= curr_dist:
                        for j in range (2, i, -1):
                            closest_3_asteroid[j]["aster"] = closest_3_asteroid[j-1]["aster"]
                            closest_3_asteroid[j]["dist"] = closest_3_asteroid[j-1]["dist"]
                        closest_3_asteroid[i]["aster"] = a 
                        closest_3_asteroid[i]["dist"] = curr_dist
                        break

    print (str(closest_3_asteroid[0]["dist"]))
    print (str(closest_3_asteroid[1]["dist"]))
    print (str(closest_3_asteroid[2]["dist"]))
    print()
    return closest_3_asteroid

# return the smallest asteroid or the closest one if size ties
def smallest_asteroid (closest_3_asteroid):
    smallest_asteroid = None
    for asteroid_dict in closest_3_asteroid:
        # Does not yet exist, so initialize first asteroid as the closest.
        if smallest_asteroid is None:
            smallest_asteroid = asteroid_dict["aster"]

        # candidate with samller size
        if asteroid_dict["aster"]["size"] < smallest_asteroid["size"]:
            smallest_asteroid = asteroid_dict["aster"]
        # candidate with same size but closer to ship
        elif asteroid_dict["aster"]["size"] == smallest_asteroid["size"]:
            if asteroid_dict["aster"]["dist"] < smallest_asteroid["dist"]:
                smallest_asteroid = asteroid_dict["aster"]

    return smallest_asteroid

# method for estimate angle of target
def target_angle (ship_state, target_asteroid):
    