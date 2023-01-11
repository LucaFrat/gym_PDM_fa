from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.urdfObstacle import UrdfObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

import os

obst1Dict = {
    "type": "sphere",
    "geometry": {"position": [2.0, 2.0, 1.0], "radius": 1.0},
}
sphereObst1 = SphereObstacle(name="simpleSphere", content_dict=obst1Dict)
obst2Dict = {
    "type": "sphere",
    'movable': True,
    "geometry": {"position": [2.0, -0.0, 0.5], "radius": 0.2},
}
sphereObst2 = SphereObstacle(name="simpleSphere", content_dict=obst2Dict)
urdfObst1Dict = {
    'type': 'urdf',
    'geometry': {'position': [1.5, 0.0, 0.05]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/duck.urdf'),
}
urdfObst1 = UrdfObstacle(name='duckUrdf', content_dict=urdfObst1Dict)

urdfObst2Dict = {
    'type': 'urdf',
    'geometry': {'position': [8.0, 0.0, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect1.urdf'),
}
urdfObst2 = UrdfObstacle(name='rect1Urdf', content_dict=urdfObst2Dict)

urdfObst3Dict = {
    'type': 'urdf',
    'geometry': {'position': [-8.0, 0.0, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect1.urdf'),
}
urdfObst3 = UrdfObstacle(name='rect2Urdf', content_dict=urdfObst3Dict)

urdfObst4Dict = {
    'type': 'urdf',
    'geometry': {'position': [0.7, 8.0, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect2.urdf'),
}
urdfObst4 = UrdfObstacle(name='rect3Urdf', content_dict=urdfObst4Dict)

urdfObst5Dict = {
    'type': 'urdf',
    'geometry': {'position': [-0.7, -8.0, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect2.urdf'),
}
urdfObst5 = UrdfObstacle(name='rect4Urdf', content_dict=urdfObst5Dict)

urdfObst6Dict = {
    'type': 'urdf',
    'geometry': {'position': [0.0, 0.0, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect3.urdf'),
}
urdfObst6 = UrdfObstacle(name='rect5Urdf', content_dict=urdfObst6Dict)

urdfObst7Dict = {
    'type': 'urdf',
    'geometry': {'position': [0.0, -3, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect3.urdf'),
}
urdfObst7 = UrdfObstacle(name='rect6Urdf', content_dict=urdfObst7Dict)

urdfObst8Dict = {
    'type': 'urdf',
    'geometry': {'position': [0.0, 3, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect3.urdf'),
}
urdfObst8 = UrdfObstacle(name='rect7Urdf', content_dict=urdfObst8Dict)

urdfObst9Dict = {
    'type': 'urdf',
    'geometry': {'position': [0.0, -6, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect3.urdf'),
}
urdfObst9 = UrdfObstacle(name='rect8Urdf', content_dict=urdfObst9Dict)

urdfObst10Dict = {
    'type': 'urdf',
    'geometry': {'position': [0.0, 6, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect3.urdf'),
}
urdfObst10 = UrdfObstacle(name='rect9Urdf', content_dict=urdfObst10Dict)

urdfObst11Dict = {
    'type': 'urdf',
    'geometry': {'position': [3.0, -0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst11 = UrdfObstacle(name='rect10Urdf', content_dict=urdfObst11Dict)

urdfObst12Dict = {
    'type': 'urdf',
    'geometry': {'position': [3.0, 0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst12 = UrdfObstacle(name='rect11Urdf', content_dict=urdfObst12Dict)

urdfObst13Dict = {
    'type': 'urdf',
    'geometry': {'position': [-3.0, -0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst13 = UrdfObstacle(name='rect12Urdf', content_dict=urdfObst13Dict)

urdfObst14Dict = {
    'type': 'urdf',
    'geometry': {'position': [-3.0, 0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst14 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst14Dict)

urdfObst15Dict = {
    'type': 'urdf',
    'geometry': {'position': [-1.0, 0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst15 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst15Dict)

urdfObst16Dict = {
    'type': 'urdf',
    'geometry': {'position': [1.0, 0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst16 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst16Dict)

urdfObst31Dict = {
    'type': 'urdf',
    'geometry': {'position': [1.0, -0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst31 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst31Dict)

urdfObst32Dict = {
    'type': 'urdf',
    'geometry': {'position': [-1.0, -0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst32 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst32Dict)

urdfObst17Dict = {
    'type': 'urdf',
    'geometry': {'position': [-5.0, -0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst17 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst17Dict)

urdfObst18Dict = {
    'type': 'urdf',
    'geometry': {'position': [5.0, -0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst18 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst18Dict)

urdfObst33Dict = {
    'type': 'urdf',
    'geometry': {'position': [5.0, 0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst33 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst33Dict)

urdfObst34Dict = {
    'type': 'urdf',
    'geometry': {'position': [-5.0, 0.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst34 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst34Dict)

urdfObst19Dict = {
    'type': 'urdf',
    'geometry': {'position': [-3.0, 3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst19 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst19Dict)

urdfObst20Dict = {
    'type': 'urdf',
    'geometry': {'position': [3.0, -3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst20 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst20Dict)

urdfObst21Dict = {
    'type': 'urdf',
    'geometry': {'position': [-1.0, 3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst21 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst21Dict)

urdfObst22Dict = {
    'type': 'urdf',
    'geometry': {'position': [1.0, -3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst22 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst22Dict)

urdfObst23Dict = {
    'type': 'urdf',
    'geometry': {'position': [-5.0, 3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst23 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst23Dict)

urdfObst24Dict = {
    'type': 'urdf',
    'geometry': {'position': [5.0, -3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst24 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst24Dict)

urdfObst25Dict = {
    'type': 'urdf',
    'geometry': {'position': [-3.0, -3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst25 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst25Dict)

urdfObst26Dict = {
    'type': 'urdf',
    'geometry': {'position': [3.0, 3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst26 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst26Dict)

urdfObst27Dict = {
    'type': 'urdf',
    'geometry': {'position': [1.0, 3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst27 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst27Dict)

urdfObst28Dict = {
    'type': 'urdf',
    'geometry': {'position': [-1.0, -3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst28 = UrdfObstacle(name='rect28Urdf', content_dict=urdfObst28Dict)

urdfObst29Dict = {
    'type': 'urdf',
    'geometry': {'position': [-5.0, -3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst29 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst29Dict)

urdfObst30Dict = {
    'type': 'urdf',
    'geometry': {'position': [5.0, 3.85, 0.2]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/rect4.urdf'),
}
urdfObst30 = UrdfObstacle(name='rect13Urdf', content_dict=urdfObst30Dict)



dynamicObst1Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ['2.0 - 0.1 * t', '-0.0', '0.1'], "radius": 0.2},
}
dynamicSphereObst1 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst1Dict)
dynamicObst2Dict = {
    "type": "analyticSphere",
    "geometry": {"trajectory": ['0.6', '0.5 - 0.1 * t', '0.8'], "radius": 0.2},
}
dynamicSphereObst2 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst2Dict)
splineDict = {'degree': 2, 'controlPoints': [[0.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 0.0]], 'duration': 10}
dynamicObst3Dict = {
    "type": "splineSphere",
    "geometry": {"trajectory": splineDict, "radius": 0.2},
}
dynamicSphereObst3 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst3Dict)

