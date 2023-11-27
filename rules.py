from skfuzzy import control as ctrl

collision_time
bullet_delta
evade_delta
thrust
turn_rate
fire

#Declare each fuzzy rule
[
    ctrl.Rule(collision_time['L'] & evade_delta['H'], (thrust ['L'], turn['M'])),
    ctrl.Rule(collision_time['L'] & evade_delta['L'], (thrust ['L'], turn['M'])),
    ctrl.Rule(collision_time['L'] & evade_delta['ML'], (thrust ['MH'], turn['ML'])),
    ctrl.Rule(collision_time['L'] & evade_delta['MH'], (thrust ['MH'], turn['MH'])),
    ctrl.Rule(collision_time['L'] & evade_delta['M'], (thrust ['H'], turn['M'])),    
    
    ctrl.Rule(collision_time['M'] & evade_delta['H'], (thrust ['ML'], turn['M'])),
    ctrl.Rule(collision_time['M'] & evade_delta['L'], (thrust ['ML'], turn['M'])),
    ctrl.Rule(collision_time['M'] & evade_delta['ML'], (thrust ['M'], turn['ML'])),
    ctrl.Rule(collision_time['M'] & evade_delta['MH'], (thrust ['M'], turn['MH'])),
    ctrl.Rule(collision_time['M'] & evade_delta['M'], (thrust ['MH'], turn['M'])),

    ctrl.Rule(collision_time['H'] & evade_delta['H'], (thrust ['ML'], turn['H'])),
    ctrl.Rule(collision_time['H'] & evade_delta['L'], (thrust ['ML'], turn['L'])),
    ctrl.Rule(collision_time['H'] & evade_delta['ML'], (thrust ['H'], turn['ML'])),
    ctrl.Rule(collision_time['H'] & evade_delta['MH'], (thrust ['H'], turn['MH'])),
    ctrl.Rule(collision_time['H'] & evade_delta['M'], (thrust ['H'], turn['M'])),
]