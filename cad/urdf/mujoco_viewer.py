import mujoco
from mujoco import viewer

path = 'bassie_urdf/meshes/bassie.xml'

# Load the model
model = mujoco.MjModel.from_xml_path(path)

viewer.launch(model)
