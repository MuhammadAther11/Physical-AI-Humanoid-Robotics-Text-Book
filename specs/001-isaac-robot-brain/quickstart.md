# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

This guide provides instructions to quickly set up the local environment and access the content for Module 3.

## Prerequisites

Before you begin, ensure you have the following installed:

-   Node.js (LTS version recommended)
-   npm (Node Package Manager, usually installed with Node.js)
-   Git

## 1. Clone the Repository

First, clone the project repository from GitHub to your local machine:

```bash
git clone [repository-url]
cd new-Hackathon # Or your project root directory
```

## 2. Navigate to the Module Branch

Ensure you are on the correct feature branch for this module:

```bash
git checkout 001-isaac-robot-brain
```

## 3. Install Docusaurus Dependencies

From the project root directory, install the necessary Node.js dependencies for Docusaurus:

```bash
npm install
```

## 4. Start the Docusaurus Development Server

Once the dependencies are installed, you can start the local development server:

```bash
npm run start
```

This command will typically open a new browser tab at `http://localhost:3000` (or similar) where you can view the Docusaurus site.

## 5. Access Module 3 Content

Navigate through the Docusaurus site to find "Module 3 - The AI-Robot Brain (NVIDIA Isaac)" in the documentation sidebar. The content includes:

-   **Isaac Sim**: Photorealistic simulation and synthetic data.
-   **Isaac ROS**: VSLAM and perception acceleration.
-   **Nav2**: Path planning for humanoid robots.

## 6. Building the Static Site (Optional)

If you wish to build the static HTML, CSS, and JavaScript files for deployment, you can run:

```bash
npm run build
```

The generated static files will be located in the `build` directory.

## Contributing

For making changes or contributing to the content, edit the Markdown files located under `src/docs/module-3-ai-robot-brain/`. Remember to follow the project's contribution guidelines and Docusaurus Markdown conventions.
