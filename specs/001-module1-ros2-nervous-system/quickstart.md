# Quickstart: Building and Viewing the Textbook

**Purpose**: To provide instructions for authors and contributors on how to build the Docusaurus site locally to preview content changes.

## Prerequisites

- **Node.js**: Version 18.x or later.
- **Yarn**: The project uses Yarn for package management.

## Local Development

1.  **Install Dependencies**:
    Navigate to the project's root directory and run the following command to install the necessary packages for Docusaurus:
    ```bash
    yarn install
    ```

2.  **Start the Development Server**:
    Once the dependencies are installed, you can start the local development server:
    ```bash
    yarn start
    ```
    This command will build the site and serve it from a local URL (usually `http://localhost:3000`). The server will automatically rebuild the site and refresh your browser whenever you make changes to the Markdown files in the `docs/` directory.

3.  **Preview Your Content**:
    Open your web browser and navigate to `http://localhost:3000`. You should see the Docusaurus site. The content from this module, "Module 1: The Robotic Nervous System (ROS 2)," will be available under the "Docs" section.

## Building the Site

To create a static build of the entire site (the same kind that would be deployed to GitHub Pages), run:

```bash
yarn build
```

This will generate the static HTML, CSS, and JavaScript files in the `build/` directory.
