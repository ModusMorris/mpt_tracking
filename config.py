import dummy
from nofilter import NoFilter
from kalman_filter import SimpleKalmanFilter, ComplexKalmanFilter
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
        "constantvelocity": NoFilter()
    },
    "Our Team": {
        "color": [0.5, 0.3, 0.2],
        "constantposition": ComplexKalmanFilter(),
        "constantvelocity": ComplexKalmanFilter()
    }
}