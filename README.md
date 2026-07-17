# Dynamic Behavior Trees for Autonomous UAV Navigation

This is a proof of concept for an Unmanned Aerial Vehicle (UAV) controlled by an LLM within a Behavior Tree Structure. The UAV is tasked with navigating an arena and hit 3 waypoints without hitting any obstacles.

## System Architecture & Core Concept

A pre-defined behavior tree calls on an LLM (llama3.2 3b) to then use a pre-defined sub-tree to create waypoints to avoid obstacles. The idea is that in a more robust implementation an LLM could choose between a set of behavior trees to choose the best behavior given a complex situation.

The environment consists of box that confines the UAV, containing 3 goals for the UAV to reach. There are 3 obstacles in the environment that the UAV must avoid to successfully complete the challange.

[Video Demonstration](https://drive.google.com/file/d/1iiO69vEuZ35xmpbXOVsUuDfqjPxfFVYO/view?usp=sharing​)

## Installation
Follow the ROSflight tutorials at [rosflight.org](rosflight.org) to set up a rosflight workspace and ensure that you can fly waypoints with a multirotor in rosflight_sim.

If you are using a GPU it is reccomended that you ensure GPU passthrough is working, this will improve the speed of the model.

Execute the following on the host machine to install ollama and pull llama3.2 3b:

```bash
# If you don't have zstd you will need to install it with:
sudo apt-get install zstd
# Install ollama
curl -fsSL https://ollama.com/install.sh | sh

# Ensure the model works
ollama run llama3.2:3b
```

#### Install Python Libraries:

```bash
# Install py_trees
pip install py_trees
# Install the openai module
pip install openai
```

More information about py_trees can be found [here](https://py-trees.readthedocs.io/en/devel/introduction.html).

#### Workspace Compilation:

Clone this repository into your rosflight workspace. We recommend cloning it into your `rosflight_ws/src` directory so it integrates with your ROS 2 workspace. After that you will need to call `colcon build` and then source your terminal(s) with `source /rosflight_ws/install/setup.bash` (or `source /rosflight_ws/install/setup.zsh` if you are using zsh).

## Quick Start

#### Step 1: Launch the Virtual Environment:

```bash
# Launch rosflight_sim 
ros2 launch rosflight_sim multirotor_standalone.launch.py
# In a second terminal, launch ai_agent_dbt
ros2 launch ai_agent_dbt ai_agent_dbt.launch.py
# In a third terminal, set the hold_last parameter to 'true'
ros2 param set path_manager hold_last true
```

Load the config file to see the environment:

1. In the RViz window, click **File → Open Config**
2. Navigate to the `resource/` directory of this repository
3. Select `ai_agent_dbt_rviz_config.rviz`
4. The walls, goals, and obstacles should load in

#### Step 2: Trigger Autonomous Execution Trees

```bash
# Run the navigation script:
ros2 run ai_agent_dbt ai_agent_dbt.launch.py
```

## Package Structure

```text
ai_agent_dbt/
├── CMakeLists.txt                     # Build system definition configuration
├── package.xml                        # C++/Python dependency configuration
├── launch/
│   └── ai_agent_dbt.launch.py         # Lauch file to launch relevant ros2 nodes
├── src/
│   ├── rviz_arena_publisher.cpp       # Enclosure boundary visualization manager
│   └── rviz_hallway_publisher.cpp     # Dual-mode terrain visualizer node
├── scripts/
│   ├── BT_waypoint_navigator.py       # Script that uses only Behavior Trees to reach the goals
│   └── BT_Ollama_waypoint_navigator.py# Script that uses an LLM to reach the goals
```

## BT_Ollama_waypoint_navigator.py flow

