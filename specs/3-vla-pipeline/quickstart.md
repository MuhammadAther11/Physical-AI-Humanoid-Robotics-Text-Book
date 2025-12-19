# Quickstart Guide: Module 4 - Vision-Language-Action (VLA)

This guide provides instructions to quickly set up the local environment and access the content for Module 4.

## Prerequisites

Before you begin, ensure you have the following installed:
- ROS 2 (Humble Hawksbill recommended)
- Python 3.11+
- Node.js (LTS version recommended) and npm
- Git

## 1. Clone the Repository

First, clone the project repository from GitHub to your local machine:

```bash
git clone [repository-url]
cd new-Hackathon # Or your project root directory
```

## 2. Navigate to the Feature Branch

Ensure you are on the correct feature branch for this module:

```bash
git checkout 3-vla-pipeline
```

## 3. Install Project Dependencies

Install Python and Node.js dependencies:

```bash
# For Python dependencies (assuming a virtual environment is recommended)
python -m venv .venv
source .venv/bin/activate # On Windows use: .venv\Scripts\activate
pip install -r requirements.txt # Assuming a requirements.txt will be created or exists

# For Node.js dependencies (for Docusaurus)
npm install
```

## 4. Start the Docusaurus Development Server

Once the dependencies are installed, you can start the local development server for the documentation:

```bash
npm run start
```

This command will typically open a new browser tab at `http://localhost:3000` (or similar) where you can view the Docusaurus site.

## 5. Access Module 4 Content

Navigate through the Docusaurus site to find "Module 4 - Vision-Language-Action (VLA)" in the documentation sidebar. The content includes:

-   **Voice-to-Action with Whisper**: Transcribing voice to robot commands.
-   **LLM-Based Task Planning to ROS 2**: Using LLMs for robot task planning.
-   **Capstone: Autonomous Humanoid Pipeline**: Integrating VLA components for end-to-end autonomy.

## 6. Building the Static Site (Optional)

If you wish to build the static HTML, CSS, and JavaScript files for deployment, you can run:

```bash
npm run build
```

The generated static files will be located in the `build` directory.

## Contributing

For making changes or contributing to the content, edit the Markdown files located under `docs/module-4-vla/`. Remember to follow the project's contribution guidelines and Docusaurus Markdown conventions.
