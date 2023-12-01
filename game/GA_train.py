
import random
import EasyGA
import time

from kesslergame import Scenario, KesslerGame, GraphicsType, TrainerEnvironment
from controller import FSController

def setup_GA():
    train_ga = EasyGA.GA()
    train_ga.gene_impl = lambda: generate_gene()
    train_ga.chromosome_length = 7  # num of antecedents
    train_ga.population_size = 50
    train_ga.target_fitness_type = 'max'
    train_ga.generation_goal = 10
    train_ga.fitness_function_impl = fitness
    train_ga.evolve()

    # Print out the current generation and the population
    train_ga.print_generation()
    train_ga.print_population()

    train_ga.sort_by_best_fitness()
    best_chromosome = train_ga.population[0]
    print(best_chromosome)
    return best_chromosome

def fitness(chromosome):
    my_test_scenario = Scenario(name='Test Scenario',
                                num_asteroids=10,
                                ship_states=[
                                    {'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1}
                                ],
                                map_size=(1000, 800),
                            time_limit=30,
                                ammo_limit_multiplier=0,
                                stop_if_no_ammo=False)

    game_settings = {'perf_tracker': True,
                     'graphics_type': GraphicsType.Tkinter,
                     'realtime_multiplier': 1,
                     'graphics_obj': None}

    game = TrainerEnvironment(settings=game_settings)  # Use this for max-speed, no-graphics simulation
    score, perf_data = game.run(scenario=my_test_scenario, controllers=[FSController(chromosome)])
    team = score.teams[0]
    asteroids_hit = team.asteroids_hit
    deaths = team.deaths
    accuracy = team.accuracy

    final_score = asteroids_hit * accuracy - deaths*50
    print(final_score, chromosome)

    return final_score

def generate_gene():
    return random.random()

if __name__ == "__main__":
    setup_GA()
