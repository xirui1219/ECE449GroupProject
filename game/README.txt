Our group's submission consists of 3 separate files:
- controller.py:
    This contains our controller, FSController.
    FSController() will use the default chromosome (which is the best chromosome from our own training);
    FSController(chromosome) will use a given chromosome to initialize itself.

- GA_train.py:
    This contains the training sequence for FSController.
    In particular, get_GA_controller() will:
    1. Get the best chromosome (best_chromosome) from GA training on FSController.
    2. Return FSController(best_chromosome).

- scenario_test.py:
    This file shows how to use get_GA_controller() to initialize the controller and pass it into the game.
    Alternatively, FSController() could be used instead, which will perform no GA training.