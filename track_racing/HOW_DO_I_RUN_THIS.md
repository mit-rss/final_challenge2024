# Simulator
This will likely only work locally. You need the follow dependencies first:
```bash
pip install trimesh
pip install pyrender
pip install "pyglet<2"
```
Then build (`colcon build && source install/setup.bash`) and run these launch files:
```bash
ros2 launch track_racing track_racing_sim.launch.xml
ros2 launch track_racing track_racing.launch.xml
```

# Real Life (TBD)