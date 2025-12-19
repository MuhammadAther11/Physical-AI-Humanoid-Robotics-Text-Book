# Tasks: Module 2 – The Digital Twin

This file outlines the implementation tasks for the Digital Twin module, ordered by dependency.

## Phase 1: Setup

-   [X] T001 Create a new ROS 2 package named `digital_twin_examples` in the `src` directory.
-   [X] T002 Initialize the package with a `package.xml`, `setup.py`, and standard directory structure (`launch`, `worlds`, `urdf`).

## Phase 2: Foundational Tasks (User Story 1 - Gazebo Physics)

**Goal**: A student can run a Gazebo simulation and see a robot interact with a physical world.
**Independent Test**: `ros2 launch digital_twin_examples simulation_us1.launch.py` starts Gazebo, and a robot model falls and rests on a ground plane.

-   [X] T003 [US1] Create a basic humanoid robot URDF file `src/digital_twin_examples/urdf/robot.urdf` with visual, collision, and inertial properties for a few links (e.g., torso, legs).
-   [X] T004 [US1] Create a Gazebo world file `src/digital_twin_examples/worlds/default.world` that includes a ground plane and basic lighting.
-   [X] T005 [US1] Create a ROS 2 launch file `src/digital_twin_examples/launch/simulation_us1.launch.py` to start Gazebo, spawn the robot from the URDF, and include a `robot_state_publisher`.

## Phase 3: User Story 2 - Interactive Unity Scene

**Goal**: A student can visualize the robot in Unity and see its movements mirrored from Gazebo.
**Independent Test**: Running the Unity scene shows a 3D robot model that accurately tracks the pose of the robot in the Gazebo simulation launched in Phase 2.

-   [X] T006 [P] [US2] Create a new Docusaurus markdown file `docs/module-2/02-unity-visualization.md` to document the Unity setup process.
-   [X] T007 [US2] In the new markdown file, add a section with screenshots and text explaining how to install the ROS-TCP-Connector in a new Unity project.
-   [X] T008 [US2] In the markdown file, add a section explaining how to import the robot's URDF into Unity using the `URDF-Importer` package.
-   [X] T009 [US2] In the markdown file, add a section with a C# code snippet for a script that subscribes to `/joint_states` and updates the robot's joint positions in Unity.
-   [X] T010 [US2] In the markdown file, add a section explaining how to configure the ROS connection settings and run the scene.

## Phase 4: User Story 3 - Sensor Simulation

**Goal**: A student can add simulated sensors to the robot and visualize their data.
**Independent Test**: After launching the simulation, `rviz2` can visualize a point cloud from the LiDAR and an image from the depth camera.

-   [X] T011 [US3] Copy the URDF from T003 to a new file `src/digital_twin_examples/urdf/robot_sensors.urdf`.
-   [X] T012 [P] [US3] In `robot_sensors.urdf`, add a Gazebo plugin for a simulated LiDAR sensor attached to the robot's torso.
-   [X] T013 [P] [US3] In `robot_sensors.urdf`, add a Gazebo plugin for a simulated depth camera sensor attached to the robot's head.
-   [X] T014 [US3] Create a new launch file `src/digital_twin_examples/launch/simulation_us3.launch.py` that is a copy of `simulation_us1.launch.py` but uses the `robot_sensors.urdf` file instead.

## Phase 5: Polish & Cross-Cutting Concerns

-   [X] T015 Create the main Docusaurus page `docs/module-2/01-digital-twins.md` explaining the core concepts of digital twins, Gazebo, and the overall architecture, linking to the other pages.
-   [X] T016 Create diagrams (using Mermaid.js) in the documentation to show the data flow between Gazebo, ROS 2, and Unity.
-   [X] T017 Record short, silent MP4 videos of the simulations (robot falling, sensor data) and embed them in the documentation.
-   [X] T018 Zip the completed `digital_twin_examples` package and add it to the `static` directory for users to download.
-   [X] T019 Review and edit all documentation for clarity, grammar, and technical accuracy.

## Dependencies

-   **US2** depends on **US1**.
-   **US3** depends on **US1**.
-   **US2** and **US3** can be worked on in parallel after **US1** is complete.
-   **Polish** phase depends on all user story phases being complete.

## Parallel Execution Examples

-   **After US1 is done**:
    -   One developer can start on the documentation for Unity (**T006 - T010**).
    -   Another developer can work on adding sensors to the URDF and creating the new launch file (**T011 - T014**).

## Implementation Strategy

The implementation will follow a story-based, incremental approach. The MVP is User Story 1, which provides the core value of a runnable physics simulation. Subsequent user stories add layers of functionality (visualization, sensors) that build on this foundation. The final phase will integrate all parts into a cohesive documentation module.
