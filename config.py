import numpy as np
import dummy
import nam
import random_noise
import angular
import constant_turn

# TODO: Add your filters here
filters = {
    "Dummy": {
        "color": [0.2, 0.2, 0.4],
        "constantposition": dummy.DummyFilter(),
        "constantvelocity": dummy.DummyFilter(2),
        "constantvelocity2": dummy.DummyFilter(2),
        "constantturn": dummy.DummyFilter(2),
        "randomnoise": dummy.DummyFilter(2),
        "angular": dummy.DummyFilter(2),
    },
    "NAMTeam": {
        "color": [0.5, 0.1, 0.9],
        "constantposition": nam.NamFilter(2),
        "randomnoise": random_noise.RandomNoiseFilter(2),
        "constantvelocity": dummy.DummyFilter(2),
        "constantvelocity2": dummy.DummyFilter(2),
        "constantturn": constant_turn.ConstantTurnFilter(2),
        "angular": angular.AngularFilter(2),

    },
}
if __name__ == "__main__":
    print("This is the config file. Run main.py to start the simulation.")