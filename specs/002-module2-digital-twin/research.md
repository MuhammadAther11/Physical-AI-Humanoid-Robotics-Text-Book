# Research & Decisions: Module 2 Integration

This document outlines the decisions made for integrating Gazebo and Unity concepts into the Docusaurus-based textbook.

## 1. Presenting Simulation Concepts

**Decision**: Use a combination of embedded diagrams, code blocks, and short video clips (GIFs or silent MP4s) to illustrate simulation concepts.

**Rationale**:
- **Diagrams**: Essential for showing data flow (e.g., ROS 2 topics connecting Gazebo to Unity). Mermaid.js, supported by Docusaurus, is ideal for this.
- **Code Blocks**: For showing URDF snippets, launch files, and script examples. Docusaurus provides excellent syntax highlighting.
- **Video Clips**: Crucial for demonstrating physics and dynamic interactions (e.g., a robot falling, a sensor's view). Static images are insufficient. These can be embedded directly in the Markdown.

**Alternatives considered**:
- **Interactive WebGL embeds**: Considered too complex to develop and maintain for the scope of this module. The goal is to teach robotics concepts, not web development.
- **Text-only descriptions**: Insufficient for conveying the dynamic and visual nature of simulation.

## 2. Handling Runnable Examples

**Decision**: Package all runnable examples as a separate, downloadable ROS 2 package. The documentation will provide step-by-step instructions on how to download, build, and run the examples.

**Rationale**:
- **Reproducibility**: This is a core principle. A self-contained package ensures that students have the exact environment and files needed.
- **Separation of Concerns**: Keeps the Docusaurus site focused on documentation and the ROS 2 package focused on the code.
- **Versioning**: The ROS 2 package can be versioned alongside the documentation to ensure they stay in sync.

**Alternatives considered**:
- **Embedding all code in the docs**: This would be cumbersome and error-prone. It also makes it difficult for students to run the examples.
- **Linking to a git repository with scattered files**: Less organized and harder to maintain than a proper ROS 2 package.

## 3. Unity Integration Documentation

**Decision**: Document the Unity integration process using a series of screenshots and configuration file snippets. The primary focus will be on the ROS-TCP-Connector and how to set up subscriptions and publications to interface with the ROS 2 graph.

**Rationale**:
- **Clarity**: Unity is a highly visual tool. Screenshots are the clearest way to guide users through the editor interface.
- **Focus**: The key learning objective is the ROS 2 integration, not general Unity development. The documentation should stay focused on the relevant components and scripts.

**Alternatives considered**:
- **Providing a pre-built Unity project**: This is a good supplementary resource, but students still need to understand *how* it was built. The documentation must cover the setup process.
