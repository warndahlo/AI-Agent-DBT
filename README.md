# Dynamic Behavior Trees for Autonomous UAV Navigation (Ollama-DBT)

An advanced autonomous multi-rotor navigation framework implementing Runtime Adaptive Behavior Trees (DBT) inside containerized environments. The system enables real-time tactical trajectory generation through bounded arenas and obstacle fields by combining high-frequency safety checks with local, non-blocking Large Language Model (LLM) semantic spatial reasoning.

---

## System Architecture & Core Concept

Standard robotic architectures handle autonomy using rigid, predefined state machines or static Behavior Trees. While predictable, they struggle with dynamically shifting obstacle environments.

This repository implements a **Mutator Pattern** using `py_trees`. The system instantiates a low-frequency supervisor node that tracks live positional telemetry from the drone's blackboard. When a nearby obstacle constraint threatens the path, the supervisor drops flight velocity, prunes the old execution pathway, and grafts a newly constructed local AI action detour sub-tree directly into the running master root structure at runtime—all without breaking the underlying multi-rotor flight stabilization loops.

---

## Operational Navigation Engine

### Stadium Obstacle Routing Mode (`BT_Ollama_maze_navigator.py`)
* **The Problem:** Safely routing targets inside bounded, box-like spaces cluttered with massive column pillar obstacles.
* **The AI Mechanism:** When the drone's proximity sensor flag triggers an early threshold check near an obstacle boundary, the system applies a brief stabilization dampener and queries a local containerized **Ollama Llama 3.2 (1-Billion Parameter)** instance over the internal docker network bridge bridge (`http://172.17.0.1:11434/v1`). 
* **The Prompt Pipeline:** Raw coordinates and object radii are compiled into a strict semantic navigation prompt detailing active tracking checkpoints and the blocking entity's bounding parameters.
* **Tree Mutation:** The `llama3.2:1b` engine returns a singular, highly deterministic spatial resolution vector (`north`, `east`, `south`, or `west`). Upon reading this token, the supervisor calculates a perpendicular safety offset margin ($r_{\text{obs}} + 12.0\text{m}$), dynamically generates a new instance of `DynamicWaypointAction`, safely mutates the running execution tree, and dispatches an asynchronous bypass path request to the flight autopilot.

---

## Package Structure

```text
ai_agent_dbt/
├── CMakeLists.txt                 # Build system definition configuration
├── package.xml                    # C++/Python dependency configuration
├── launch/
│   └── rviz.launch.py             # Central layout deployment configuration
├── src/
│   ├── rviz_arena_publisher.cpp   # Enclosure boundary visualization manager
│   ├── rviz_custom_maze.cpp       # Structural hallway visualization generator
│   └── rviz_hallway_publisher.cpp # Dual-mode terrain visualizer node
├── scripts/
│   ├── BT_maze_navigator.py       # LLM Behavior Tree Adaptor script
│   └── BT_Ollama_maze_navigator.py# Box-Arena Obstacle Avoidance router script
└── resource/
    └── maeserstatue_small.stl     # Goal target visual asset model mesh
