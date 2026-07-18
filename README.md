# Dynamic Behavior Trees for UAV Navigation

This is a proof of concept for an Unmanned Aerial Vehicle (UAV) controlled by an LLM within a behavior tree (BT) structure. ROSflight is used for simulation and autonomous waypoint navigation. The UAV is tasked with navigating an arena and hit 3 waypoints without hitting any obstacles.

## System Architecture & Core Concept

A pre-defined BT calls on an LLM (llama3.2 3b) to use a pre-defined sub-tree to create waypoints to avoid obstacles. The idea is that in a more complex implementation, an LLM could choose between a set of BTs to choose the best behavior in a given situation.

The environment consists of box that confines the UAV, containing 3 goals for the UAV to reach. There are 3 obstacles in the environment that the UAV must avoid to successfully complete the challange.

[Video Demonstration](https://youtu.be/fH5avKCXdoU)

[Video Demonstration of no LLM](https://drive.google.com/file/d/1iiO69vEuZ35xmpbXOVsUuDfqjPxfFVYO/view?usp=sharing​)

As you can see, the LLM implementation of this concept doesn't function well, but the structure of the program in `BT_Ollama_waypoint_navigator` serves as an example of how a dynamic behavior tree could use an LLM to make decisions. 

## Installation
Follow the ROSflight tutorials at [rosflight.org](rosflight.org) to set up a rosflight workspace and ensure that you can fly waypoints with a multirotor in rosflight_sim.

If you have a GPU, it is reccomended that you ensure GPU passthrough is working, this will improve the speed of the model.

Execute the following on the host machine to install ollama:

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
│   ├── BT_waypoint_navigator.py       # Script that uses only Behavior Trees to reach the targets
│   └── BT_Ollama_waypoint_navigator.py# Script that uses an LLM to reach the targets
```

## BT_Ollama_waypoint_navigator.py flow

A ROS2 node named `DBT_AI_Nav` is constructed and then spun in `main` at the bottom of the program. It constructs the BTs and set ups the elements necessary for it to run. At the top of the program the locations of the targets and obstacles are listed. This does NOT control the locations of the targets and obstacles, it simply serves as information for the rest of the program. To change the layout of the environment you would need to alter `rviz_hallway_publisher.cpp`. The leaf nodes of the BT are designed in the py_trees sturcture in classes above this node, in the order they are listed below (keep in mind that the composite nodes don't need defined behaviors, they are just instantiated in the "setup" functions in `DBT_AI_NAV`). The `query_llm` function after the BT classes contains the parameters for the LLM.

### Behavior Tree Structure:

#### Main Tree

```text
root (sequence) [→]
├── Check Pos
└── Find Waypoint (selector) [?]
    ├── AI Waypoint
    |      └── *executes sub_tree*
    └── Geometric Waypoint
           └── *executes sub_tree*
```

The `root` node of the main tree starts with the `Check Pos` node that compares the current location of the UAV to it's current goal location (stored in `next_north` and `next_east`). If the UAV reaches a target at any point it increments the target to the next one in the list. When the UAV reaches the location of the current waypoint `Check Pos` returns `SUCCESS` and the root node moves on to the `Find Waypoint` selector node. First the `Find Waypoint` node attempts to run the `AI Waypoint` node. If it succeeds it returns `SUCCESS`. If it fails `Find Waypoint` moves on to `Geometric Waypoint` which attempts to create a waypoint using the calculation in the `calculate_safe_vector` function. When it does it returns `SUCCESS`. Either of these nodes succeeding causes `Find Waypoint` to return `SUCCESS`. This means that all nodes belonging to the root succeed, and the BT succeeds, starting over again.

#### Sub Tree

```text
way_tree (sequence) [→]
└── Create Waypoint
```

The sub tree is constructed by `DBT_AI_Nav`, but is not ticked by it, remaining dormant until it is ticked. When `AI Waypoint` or `Geometric Waypoint` is ready to create a waypoint it calls `sub_tree.tick()` which runs the `Create Waypoint` node.

## Problems and Next Steps

The main problem with this example is the limitations of a small LLM like Llama 3.2 3b. It simply can't keep track of all the obstacles and goals, and isn't actually capable of comprehending the scenario in full. A more robust model is likely to be able to navigate the situation easily, but this takes longer and requires more overhead and cost.

In this scenario a deterministic algorithm is far better suited, but the structure of this code serves as a starting point for incorporating artificial intelligence and dynamic behavior trees together in a broader context. One of the advantages of using AI to control an agent like a UAV is it's ability to react to unexpected changes. With more development, the structure proposed here, or one similar to it, could be used in that context.

There are directions to be explored with how the AI is incorporated in the BT structure serving different purposes like processing human input, managing a swarm of UAVs, etc. Training a model for a given environment is also likely to give better results. 

## Relevant Works

#### Behavior Tree Control with Autonomous Agents​

Heppner, Georg, et al. "Behavior tree capabilities for dynamic multi-robot task allocation with heterogeneous robot teams." 2024 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2024.​

Colledanchise, Michele, and Petter Ögren. Behavior trees in robotics and AI: An introduction. CRC Press, 2018.​

Gil-Castilla, Miguel, Ivan Maza, and Anibal Ollero. "A Modular and Scalable Framework for Autonomous Actuation and Emergency Handling with Behavior Trees for Unmanned Aerial Vehicles." Journal of Intelligent & Robotic Systems 112.1 (2026): 27.​

Wang, Chaoran, et al. "Task Management for Autonomous Flights of Micro Aerial Vehicles: A Behavior Tree Approach." 2023 6th International Symposium on Autonomous Systems (ISAS). IEEE, 2023.​

#### AI Agents in Drone Control​

ILYAS, Mohammad. "Artificial Intelligence for Drone Swarms." Journal of Systemics, Cybernetics and Informatics 23.7 (2025): 18-22.​

#### AI Agents and Behavior Tree Hybrid Approach to Control​

Wang, Chaoran, et al. "LLM-HBT: Dynamic Behavior Tree Construction for Adaptive Coordination in Heterogeneous Robots." arXiv preprint arXiv:2510.09963 (2025).​

Zgurovsky, Michael, et al. "A hybrid approach based on swarm intelligence and behavior trees for coordinating autonomous agents." (2025).​