import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import BodyIndex
from pydrake.geometry import Box, ProximityProperties
from pydrake.multibody.plant import CoulombFriction
from pydrake.math import RigidTransform

def add_ground(plant: MultibodyPlant, soft_contact=False):
    # Add a flat ground with friction
    X_WG = RigidTransform()
    X_WG.set_translation([0, 0, 0])  # Ground at z=0

    # Create proximity properties
    props = ProximityProperties()
    friction_coefficient = 0.5
    dissipation = 5.0 if soft_contact else 0.0
    stiffness = 1e5 if soft_contact else 1e8
    props.AddProperty("material", "coulomb_friction", CoulombFriction(friction_coefficient, friction_coefficient))
    props.AddProperty("material", "point_contact_stiffness", stiffness)
    props.AddProperty("material", "hunt_crossley_dissipation", dissipation)

    # Create the ground geometry
    ground_shape = Box(10, 10, 0.1)

    # Register collision geometry
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        X_WG,
        ground_shape,
        "ground_collision",
        props
    )

    # Register visual geometry
    color = np.array([0.5, 0.5, 0.5, 0.5])  # Grey color
    plant.RegisterVisualGeometry(
        plant.world_body(),
        X_WG,
        ground_shape,
        "ground_visual",
        color
    )

# Note: This function should be called before plant.Finalize()
