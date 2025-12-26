# Quickstart: Book Frontend Encapsulation

## Overview
This guide provides a quick setup for developers to understand and work with the book-frontend encapsulation.

## Prerequisites
- Node.js 18+ LTS
- Git
- Basic understanding of Docusaurus

## Repository Structure
After the encapsulation, the repository will have the following structure:
```
book-frontend/                    # New dedicated directory for all Docusaurus-related code
├── blog/                         # Docusaurus blog content
├── docs/                         # Docusaurus documentation content
├── src/                          # Docusaurus custom React components
│   ├── components/               # Reusable UI components
│   ├── css/                      # Custom styles
│   └── pages/                    # Custom pages
├── static/                       # Static assets (images, files, etc.)
├── docusaurus.config.js          # Docusaurus configuration
├── sidebars.js                   # Docusaurus sidebar configuration
├── package.json                  # Node.js package file
├── yarn.lock or package-lock.json # Dependency lock file
└── babel.config.js               # Babel configuration
```

## Getting Started
1. Clone the repository
2. Navigate to the book-frontend directory: `cd book-frontend`
3. Install dependencies: `npm install` or `yarn install`
4. Start the development server: `npm run start` or `yarn start`
5. Open your browser to http://localhost:3000

## Development Commands
- `npm run start` - Start development server
- `npm run build` - Build static site
- `npm run serve` - Serve built site locally
- `npm run deploy` - Deploy to GitHub Pages (if configured)

## Key Configuration Files
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration
- `src/pages/` - Custom pages
- `src/components/` - Reusable components
- `static/` - Static assets