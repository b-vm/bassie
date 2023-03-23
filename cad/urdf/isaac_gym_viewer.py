from isaacgym import gymapi

# Initialize Isaac Gym
gym = gymapi.acquire_gym()
sim_params = gymapi.SimParams()
sim_params.dt = 1/60
sim_params.substeps = 4
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0, 0, -9.81)
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# Load the URDF file for the robot
urdf_path = "bassie_urdf/bassie.urdf"
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.default_dof_drive_mode = 3
# asset_options.flip_visual_attachments = True
robot_asset = gym.load_asset(sim, '.', urdf_path, asset_options)

# create ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
gym.add_ground(sim, plane_params)


# Create an environment
env = gym.create_env(sim, gymapi.Vec3(0, 0, 0), gymapi.Vec3(0, 0, 0), 1)

# Instantiate the robot
robot_pose = gymapi.Transform()
robot_pose.p = gymapi.Vec3(0, 0, 1)
robot_handle = gym.create_actor(env, robot_asset, robot_pose, "robot", 0, 1)

cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)
gym.viewer_camera_look_at(viewer, env, gymapi.Vec3(1, 1, 1), gymapi.Vec3(0, 0, 1))

# Set up the simulation loop
num_steps = 1000
# for i in range(num_steps):
while True:
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

# Clean up
gym.destroy_env(env)
gym.destroy_sim(sim)
