import mujoco
from mujoco import viewer

path = 'bassie.SLDASM.urdf'

# Load the model
model = mujoco.MjModel.from_xml_path(path)

viewer.launch(model)