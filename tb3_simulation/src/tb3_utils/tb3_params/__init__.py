class TB3Params:
    def __init__(self, robot_model) -> None:
        if robot_model == 'burger':
            self.wheel_separation = 0.160 # [m]
            self.turning_radius = 0.080 # [m]
            self.robot_radius = 0.105 # [m]
        elif robot_model == 'waffle' or robot_model == 'waffle_pi':
            self.wheel_separation = 0.287 # [m]
            self.turning_radius = 0.1435 # [m]
            self.robot_radius = 0.220 # [m]
        else:
            raise Exception(f'Turtlebot3 model {robot_model} not defined')
        self.wheel_radius = 0.033 # [m]
