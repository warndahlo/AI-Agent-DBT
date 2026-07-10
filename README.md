Dynamic Behavior Trees for Autonomous UAV Navigation (Ollama-DBT)
An advanced autonomous multi-rotor navigation framework implementing Runtime Adaptive Behavior Trees (DBT). The system enables real-time tactical trajectory generation through tightly confined spaces and obstacle fields using two distinct paradigms: non-blocking Large Language Model (LLM) semantic reasoning over local containerized APIs, and high-frequency deterministic Geometric Vector Interception Engine calculations.
System Architecture & Core Concept
Standard robotic architectures handle autonomy using rigid, predefined state machines or static Behavior Trees. While predictable, they struggle with structural variations or unexpected configuration blocks.
This repository implements a Mutator Pattern using py_trees. The system instantiates a low-frequency supervisor node that takes live situational data from the drone's blackboard, runs logic models, prunes out old execution paths, and grafts newly constructed action sub-trees into the running master root structure at runtime without dropping multi-rotor flight stabilization loops.
The workspace features two primary operational navigation modes designed for autonomous UAV testing:
1. Semantic Labyrinth Reasoning Mode (BT_maze_navigator.py)
    • • The Problem: Navigating blind structural corridors where standard sensor ranges are blocked by immediate right-angle blind corners.
    • • The AI Mechanism: When the drone approaches a wall bounding box (d < 4.2m), the system pauses execution and queries a containerized Ollama Llama3 instance via a local Docker network bridge. It translates raw distance measurements into a clean semantic markdown prompt detailing current directional trends and bounding clearances.
    • • Tree Mutation: The LLM responds with a single, highly deterministic navigation vector direction (north, east, south, west). Upon receiving this prediction, the master tree coordinator dynamically generates a new instance of DynamicWaypointAction, safely mutates the execution graph, and dispatches an asynchronous target vector request to the autopilot track manager.
2. High-Speed Geometric Obstacle Avoidance Mode (BT_Ollama_maze_navigator.py)
    • • The Problem: Safely routing targets inside stadium spaces cluttered with massive column pillar obstacles without incurring the latency or non-deterministic risk of network API queries.
    • • The Mathematical Mechanism: This engine runs a high-frequency line-segment to circular-cylinder interception pipeline. It continuously maps the drone's localized vector trajectory relative to localized pillar coordinates.
    • • Deterministic Avoidance: By taking the vector dot product, it calculates a clamped projection tracking step. If the shortest path clearance drops below the pillar's localized physical constraint radius, the system applies a dynamic perpendicular safety offset margin (robs + 6.0m), instantly generating structural detours on the fly while tracking target checkpoints with full state validation.
Package Structure
ai_agent_dbt/
├── CMakeLists.txt                  # Build system definition configuration
├── package.xml                     # C++/Python dependency configuration
├── launch/
│   └── rviz.launch.py              # Central layout deployment configuration
├── src/
│   ├── rviz_arena_publisher.cpp    # Enclosure boundary visualization manager
│   ├── rviz_custom_maze.cpp        # Structural hallway visualization generator
│   └── rviz_hallway_publisher.cpp  # Dual-mode terrain visualizer node
├── scripts/
│   ├── BT_maze_navigator.py        # LLM Behavior Tree Adaptor script
│   └── BT_Ollama_maze_navigator.py # Geometric Vector Obstacle Avoidance script
└── resource/
    └── maeserstatue_small.stl      # Goal target visual asset model mesh
Installation & Replication Setup
Prerequisites: Ensure your host target machine runs Ubuntu 22.04 LTS with ROS 2 Humble Geochelone and your local rosflight workspace configured.
1. Download Local LLM Engine
Execute the following to pull your background semantic modeling nodes:
curl -fsSL https://ollama.com/install.sh | sh
ollama run llama3
[NOTE] Verify that your local API interface binds correctly to port 11434. If running ROS 2 inside an isolated container environment, update the base_url parameter inside your python nodes to reference your system network bridge gateway.

2. Python SDK Dependencies
pip install --upgrade openai py-trees
3. Workspace Compilation
cd ~/rosflight_ws/src
git clone https://github.com/owarndahl/ai_agent_dbt.git
cd ~/rosflight_ws
colcon build --packages-select ai_agent_dbt --symlink-install
source install/setup.bash
Operational Execution Guide
Step 1: Launch the Virtual Environment
ros2 launch ai_agent_dbt rviz.launch.py
Step 2: Trigger Autonomous Execution Trees
Deterministic Vector Tracking Mode:
ros2 run ai_agent_dbt BT_Ollama_maze_navigator.py

Adaptive LLM Tree Mutation Mode:
ros2 run ai_agent_dbt BT_maze_navigator.py
