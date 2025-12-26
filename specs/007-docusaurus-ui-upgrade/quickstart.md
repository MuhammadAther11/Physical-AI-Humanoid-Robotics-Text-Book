# Quickstart Guide: Docusaurus UI Upgrade

## Prerequisites

- Node.js 18+ LTS
- npm or yarn package manager
- Git
- A modern web browser for testing

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd Physical-AI-Humanoid-Robotics-Text-Book
   ```

2. Navigate to the book frontend directory:
   ```bash
   cd book-frontend
   ```

3. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

## Local Development

1. Start the development server:
   ```bash
   npm start
   # or
   yarn start
   ```

2. Open your browser to http://localhost:3000 to see the site with the upgraded UI.

3. Edit components in the `src/` directory to make changes to the UI. The site will automatically reload when changes are saved.

## Building for Production

1. Build the static site:
   ```bash
   npm run build
   # or
   yarn build
   ```

2. The built site will be in the `build/` directory and can be deployed to any static hosting service.

## Testing the UI Changes

1. Run the test suite:
   ```bash
   npm test
   # or
   yarn test
   ```

2. For UI-specific testing, manually verify:
   - Responsive design on various screen sizes
   - Navigation functionality
   - Typography and color scheme
   - Accessibility features
   - Page load performance

## Key Configuration Files

- `docusaurus.config.js` - Main Docusaurus configuration
- `src/css/custom.css` - Custom CSS overrides
- `src/theme/` - Custom theme components
- `sidebars.js` - Navigation structure

## Customization

To customize the UI further:

1. Modify CSS in `src/css/custom.css`
2. Create custom components in `src/components/`
3. Override default theme components in `src/theme/`
4. Update configuration in `docusaurus.config.js`