# Data Model: Book Frontend Encapsulation

## Entities

### book-frontend directory
- **Description**: The new dedicated folder that will contain all Docusaurus-related code and assets
- **Attributes**: 
  - path: string (the directory path)
  - contents: list of files and subdirectories
- **Relationships**: Contains all other Docusaurus-related entities

### Docusaurus configuration
- **Description**: All files that configure the behavior of the Docusaurus documentation site
- **Attributes**:
  - docusaurus.config.js: main configuration file
  - sidebars.js: sidebar navigation configuration
  - babel.config.js: JavaScript transpilation configuration
  - package.json: Node.js package configuration
- **Relationships**: Configures the behavior of UI components, pages, and assets

### UI components
- **Description**: Reusable frontend elements that make up the user interface of the documentation site
- **Attributes**:
  - path: string (relative to src/components/)
  - props: object (input parameters)
  - state: object (internal state)
- **Relationships**: Used by pages to construct the user interface

### Static assets
- **Description**: Images, stylesheets, JavaScript files, and other resources used by the documentation site
- **Attributes**:
  - type: string (image, css, js, font, etc.)
  - path: string (relative to static/ directory)
  - size: number (file size in bytes)
- **Relationships**: Referenced by UI components and pages

### Internal references
- **Description**: Path references within the codebase that point to Docusaurus-related resources
- **Attributes**:
  - source: string (file containing the reference)
  - target: string (path being referenced)
  - type: string (relative, absolute, or module import)
- **Relationships**: Connects different parts of the Docusaurus application