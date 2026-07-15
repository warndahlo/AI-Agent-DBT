# Dynamic Behavior Trees for Autonomous UAV Navigation

This is a proof of concept for an Unmanned Aerial Vehicle (UAV) controlled by an LLM within a Behavior Tree Structure.

## System Architecture & Core Concept

A pre-defined behavior tree calls on an LLM (llama3.2 1b) to then choose between a set of pre-defined sub-trees to create waypoints to avoid obstacles. 

[Video Example](https://drive.google.com/file/d/1iiO69vEuZ35xmpbXOVsUuDfqjPxfFVYO/view?usp=sharing​)

## Operational Navigation Engine

### Stadium Obstacle Routing Mode (`BT_Ollama_maze_navigator.py`)
* **The Problem:** Safely routing targets inside bounded, box-like spaces cluttered with massive column pillar obstacles.
* **The AI Mechanism:** When the drone's proximity sensor flag triggers an early threshold check near an obstacle boundary, the system applies a brief stabilization dampener and queries a local containerized **Ollama Llama 3.2 (1-Billion Parameter)** instance over the internal docker network bridge bridge. 
* **The Prompt Pipeline:** Raw coordinates and object radii are compiled into a strict semantic navigation prompt detailing active tracking checkpoints and the blocking entity's bounding parameters.
* **Tree Mutation:** The `llama3.2:1b` engine returns a singular, highly deterministic spatial resolution vector (`north`, `east`, `south`, or `west`). Upon reading this token, the supervisor calculates a perpendicular safety offset margin ($r_{\text{obs}} + 12.0\text{m}$), dynamically generates a new instance of `DynamicWaypointAction`, safely mutates the running execution tree, and dispatches an asynchronous bypass path request to the flight autopilot.

## Package Structure

```text
ai_agent_dbt/
├── CMakeLists.txt                 # Build system definition configuration
├── package.xml                    # C++/Python dependency configuration
├── launch/
│   └── ai_agent_dbt.launch.py     # Central layout deployment configuration
├── src/
│   ├── rviz_arena_publisher.cpp   # Enclosure boundary visualization manager
│   ├── rviz_custom_maze.cpp       # Structural hallway visualization generator
│   └── rviz_hallway_publisher.cpp # Dual-mode terrain visualizer node
├── scripts/
│   ├── BT_maze_navigator.py       # LLM Behavior Tree Adaptor script
│   └── BT_Ollama_maze_navigator.py# Box-Arena Obstacle Avoidance router script
```

## Installation
Follow the ROSflight tutorials at [rosflight.org](rosflight.org) to set up a rosflight workspace and ensure that you can fly waypoints with a multirotor in rosflight_sim.

If you are using a GPU it is reccomended that you ensure GPU passthrough is working, this will improve the speed of the model.

Execute the following on the host machine to install ollama and pull llama3.2 1b:
```
// If you don't have zstd you will need to install it with:
sudo apt-get install zstd

//Install and run llama3.2:1b
curl -fsSL https://ollama.com/install.sh | sh
ollama run llama3.2:1b
```

#### Python SDK Dependencies:

To install py_trees call `pip install py_trees`. More information about py_trees can be found [here](https://py-trees.readthedocs.io/en/devel/introduction.html).

#### Workspace Compilation:

Clone this repository into your rosflight workspace. We recommend cloning it into your `rosflight_ws/src` directory so it integrates with your ROS 2 workspace. After that you will need to call `colcon build` and then source your terminal(s) with `source /rosflight_ws/install/setup.bash` (or `source /rosflight_ws/install/setup.zsh` if you are using zsh).

## Quick Start

#### Step 1: Launch the Virtual Environment:

```
// Launch rosflight_sim 
ros2 launch rosflight_sim multirotor_standalone.launch.py
// In a second terminal, launch ai_agent_dbt
ros2 launch ai_agent_dbt ai_agent_dbt.launch.py
```

Load the config file to see the environment:

1. In the RViz window, click **File → Open Config**
2. Navigate to the `resource/` directory of this repository
3. Select `ai_agent_dbt_rviz_config.rviz`
4. The walls, goals, and obstacles should load in

#### Step 2: Trigger Autonomous Execution Trees

```
// Run the navigation script:
ros2 run ai_agent_dbt BT_Ollama_maze_navigator.py
```
