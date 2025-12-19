# Research: Module 1 - ROS 2 Nervous System

**Purpose**: To document key decisions and resolve any ambiguities before content creation.

## Research Topics

### 1. Docusaurus Best Practices for Code Examples

- **Task**: Investigate the best way to present runnable Python/ROS 2 code examples within Docusaurus.
- **Decision**: Use standard Markdown code blocks with Python syntax highlighting. For more complex examples, link to files in a corresponding GitHub repository (`examples/` directory) to ensure they are runnable and testable.
- **Rationale**: This approach keeps the content clean and readable while providing access to the full, working code. It avoids overly complex Docusaurus plugins.
- **Alternatives Considered**: Interactive code playgrounds (e.g., via plugin), which were rejected due to the complexity of running ROS 2 nodes in a web-based environment.

### 2. Diagramming Tools for Docusaurus

- **Task**: Select a tool for creating and embedding diagrams (e.g., for ROS 2 architecture).
- **Decision**: Use Mermaid.js, which is natively supported by Docusaurus. Create diagrams directly in the Markdown files.
- **Rationale**: Native support means no extra dependencies and a seamless authoring workflow. Diagrams as code are version-controlled and easy to update.
- **Alternatives Considered**: Exporting images from tools like draw.io or Lucidchart. Rejected because it makes updates more difficult and decouples the diagram source from the content source.

### 3. URDF Example for a Humanoid Robot

- **Task**: Select a suitable, not-overly-complex URDF example of a humanoid robot.
- **Decision**: Use a simplified version of a well-known humanoid URDF, such as a stripped-down version of the TALOS or a similar bipedal robot model. The focus will be on illustrating the structure, not on a fully functional, complex model.
- **Rationale**: A simplified model allows the focus to remain on the URDF syntax and structure without overwhelming the reader with excessive detail.
- **Alternatives Considered**: Creating a URDF from scratch was rejected as too time-consuming and not central to the learning objective. Using a full, complex model was rejected as too difficult for a student to parse initially.
