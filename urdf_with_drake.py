import numpy as np
import os
from pydrake.common import FindResourceOrThrow
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RigidTransform

meshcat = StartMeshcat()

# Create a DiagramBuilder
builder = DiagramBuilder()

# Add a MultibodyPlant to the builder
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Add the robot model
robot_url = "/home/sankaet/drake_projects/sunny/lower_body.urdf"
Parser(plant).AddModels(robot_url)  # Changed to AddModels to address deprecation warning

# Finalize the plant
plant.Finalize()

# Add visualization
AddDefaultVisualization(builder=builder, meshcat=meshcat)

# Build the diagram
diagram = builder.Build()

# Create a context for the diagram
context = diagram.CreateDefaultContext()

# Get the plant context
plant_context = plant.GetMyContextFromRoot(context)

# Get the body we want to move (assuming it's called "pelvis")
pelvis_body = plant.GetBodyByName("pelvis")

# Create a transform to move the pelvis up
X_WB = RigidTransform([0, 0, 0.9])  # Adjust the Z value as needed

# Set the transform of the pelvis
plant.SetFreeBodyPose(plant_context, pelvis_body, X_WB)

# Create a simulator
simulator = Simulator(diagram, context)

# Initialize the simulator
simulator.Initialize()

# Publish the initial state to MeshCat
diagram.ForcedPublish(context)

# Run the simulation/visualization
if "TEST_SRCDIR" in os.environ:
    meshcat.Flush()  # Wait for the messages to be sent
else:
    print("Meshcat is running. Please open the following URL in your web browser:")
    print(meshcat.web_url())
    
    input("Press Enter to exit...")