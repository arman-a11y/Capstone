# Sink drain robot sim

Husky A200 + UR5 that drives up to a hospital sink and aligns to two AprilTags.
The ROS 2 package is in `sink_sim/` (its README has all the details).

## Run it in GitHub Codespaces

1. Push this folder to a GitHub repo (it can be private).
2. On the repo page: **Code > Codespaces > ... > New with options**, pick the
   **4-core** machine, then **Create codespace**.
3. Wait for setup to finish. The first time takes about 10 to 20 minutes because it
   installs ROS packages and builds. You'll see `==> Done.` in the creation log.
4. Open the desktop: go to the **Ports** tab in VS Code, find port **6080**, click the
   globe icon. Click **Connect**, password `vscode`. Gazebo and RViz windows show up here.
5. Back in VS Code, open terminals (**Terminal > New Terminal**) and run:

```bash
# terminal 1: the sim (window appears in the desktop tab)
ros2 launch sink_sim sim.launch.py

# terminal 2: arm up, once the robot has spawned
ros2 run sink_sim arm_pose --ros-args -r __ns:=/a200_0000 -p use_sim_time:=true -p pose:=up

# terminal 3: tag detection + alignment
ros2 launch sink_sim align.launch.py
```

**It will be slow.** Codespaces has no GPU, so the camera renders in software. Expect
Gazebo to run below real time. The robot logic uses sim time, so it still works, it just
takes longer. The setup uses the front-camera-only robot to help.

**Save your hours.** Codespaces counts time while it's running. Stop it when you're done
(**Code > Codespaces > ... > Stop codespace**), or it stops by itself after 30 minutes idle.
Your files are kept while it's stopped.

Quick check without the sim (runs in seconds):

```bash
cd sink_sim && python3 -m pytest test -q
```

## Run it on your own Ubuntu 24.04

Install ROS 2 Jazzy desktop, then from this folder:

```bash
bash .devcontainer/setup.sh
source ~/.bashrc
```

Then the same three terminals as above.
