import dummy
from nofilter import NoFilter
from filter_constantposition import ComplexKalmanFilter
from filter_randomnoise import KalmanFilterWithRandomNoise
from filter_constantvelocity import KalmanFilterConstantVelocity
from filter_constantvelocity2 import KalmanFilterConstantVelocityMultiple
from filter_constantturn import KalmanFilterConstantTurn

# TODO: Add your filters here
filters = {
    "Dummy": {
        "color": [0.2, 0.2, 0.4],
        "constantposition": dummy.DummyFilter(2),
        "constantvelocity": dummy.DummyFilter(2),
        "constantvelocity2": dummy.DummyFilter(2),
        "constantturn": dummy.DummyFilter(2),
        "randomnoise": dummy.DummyFilter(2),
        "angular": dummy.DummyFilter(2),
    },
    "NoFilter": {
        "color": [0.5, 0.1, 0.9],
        "constantposition": NoFilter(),
        "randomnoise": NoFilter(),
        "constantvelocity": NoFilter(),
        "angular": NoFilter(),
        "constantvelocity2": NoFilter(),
        "constantturn": NoFilter()
    },
    "Our Team": {
        "color": [0.5, 0.3, 0.2],
        "constantposition": ComplexKalmanFilter(),
        "randomnoise": KalmanFilterWithRandomNoise(),
        "constantvelocity": KalmanFilterConstantVelocity(),
        "angular": dummy.DummyFilter(2),
        "constantvelocity2": KalmanFilterConstantVelocityMultiple(),
        "constantturn": KalmanFilterConstantTurn()
    }
}