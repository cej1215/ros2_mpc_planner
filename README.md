# MPC Planner

This repository provides a ROS 2 version of the MPC Planner adapted for the **Antbot mobile robot**.

The planner is based on the [TUD-AMR MPC Planner](https://github.com/tud-amr/mpc_planner) and uses **Acados** as the MPC solver.  
The current implementation is developed and tested with **ROS 2 Humble**.

---

# Installation

## Step 1: Clone repositories

In your ROS 2 workspace `ws/src`, clone the following repositories:

```bash
cd <path_mpc_workspace>/src

git clone https://github.com/cej1215/ros2_mpc_planner.git mpc_planner
git clone https://github.com/oscardegroot/DecompUtil.git
git clone https://github.com/oscardegroot/ros_tools.git
git clone https://github.com/ROBOTIS-move/antbot.git
```

The workspace should have a structure similar to:

```text
ws/
└── src/
    ├── mpc_planner/
    ├── DecompUtil/
    ├── ros_tools/
    └── antbot/
```

---

## Step 2: Install ROS 2 dependencies

From the root of your ROS 2 workspace:

```bash
cd <path_mpc_workspace>

rosdep install -y -r --from-paths src --ignore-src --rosdistro humble
```

---

## Step 3: Install the solver

The MPC solver is generated using Python and **Acados**.

### Conda

MPC solver 생성용 Python 환경과 ROS 2에서의 python 환경을 분리하기 위함

If you have [Miniconda](https://docs.anaconda.com/miniconda/) installed, create a separate Python environment for the solver generation.

From:

```bash
cd <path_mpc_workspace>/src/mpc_planner
```

create and activate the environment:

```bash
conda create -n mpc_planner python=3.9 pip -y
conda activate mpc_planner
```

Then install the required Python packages:

```bash
pip install -r requirements.txt
```

---

### Install Acados

This project uses the following Acados revision:

```text
v0.4.1-50-gc48cf1135
```

Clone Acados and checkout the corresponding commit:

```bash
git clone https://github.com/acados/acados.git
cd acados

git checkout c48cf1135
git submodule update --recursive --init
```

Build Acados:

```bash
mkdir -p build
cd build

cmake -DACADOS_SILENT=ON ..
make install -j4
```

---

### Link Acados to the Conda environment

With the `mpc_planner` Conda environment activated:

```bash
conda activate mpc_planner
```

install the Acados Python interface:

```bash
pip install -e ~/acados/interfaces/acados_template
```

Then add the Acados paths to your `~/.bashrc`:

```bash
echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:<path_to_acados>/lib"' >> ~/.bashrc
echo 'export ACADOS_SOURCE_DIR="<path_to_acados>"' >> ~/.bashrc
```

Reload the shell configuration:

```bash
source ~/.bashrc
```

---

### Generate the Antbot MPC solver

The solver is configured in:

```text
mpc_planner_antbot/config/settings.yaml
```

Make sure the solver is set to `acados`:

```yaml
solver_settings:
  solver: "acados"
```

Then move to the MPC Planner directory:

```bash
cd <path_mpc_workspace>/src/mpc_planner
```

Activate the Conda environment:

```bash
conda activate mpc_planner
```

Generate the Antbot solver:

```bash
python mpc_planner_antbot/scripts/generate_antbot_solver.py
```

If the solver generation completes successfully, the generated Acados solver will be used by the online MPC planner.

---

## Step 4: Build the planner

Return to the ROS 2 workspace (make sure to deactivate conda first):

```bash
conda deactivate
cd <path_mpc_workspace>
```

Build the workspace:

```bash
colcon build --symlink-install
```

After the build is complete, add the workspace setup file to your `~/.bashrc`:

```bash
echo 'source <path_mpc_workspace>/install/setup.bash' >> ~/.bashrc
```

Reload the shell:

```bash
source ~/.bashrc
```

The Antbot MPC Planner is now ready to use.

## Step 5: Run the planner

Open a new terminal and launch the Antbot Gazebo simulation:

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
```

Then, open another terminal and launch the Antbot MPC Planner:

```bash
ros2 launch mpc_planner_antbot antbot_planner.launch.py
```

After both the Gazebo simulation and MPC Planner are running, use 2D Goal Pose in RViz to select a desired goal position and orientation.

