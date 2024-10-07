import numpy as np
from pydrake.multibody.plant import MultibodyPlant

class COMCalculator:
    def __init__(self, plant: MultibodyPlant):
        self.plant = plant

    def calculate_com(self, context):
        return self.plant.CalcCenterOfMassPositionInWorld(context)