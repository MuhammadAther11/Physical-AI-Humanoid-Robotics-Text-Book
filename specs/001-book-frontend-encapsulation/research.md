# Research: Book Frontend Encapsulation

## Decision: Use Git Move Operations for File Migration
**Rationale**: To preserve Git history during the migration of Docusaurus files to the new book-frontend directory, Git's built-in move operations (git mv) are the most appropriate solution. This ensures that the history of each file is maintained, which is important for tracking changes and debugging.

**Alternatives considered**:
- Simple file system move: Loses Git history
- Copy and delete: Creates duplicate history entries
- Git filter-branch: Overly complex for this use case

## Decision: Maintain Root Package.json for Vercel Deployment
**Rationale**: For Vercel deployment, it's important to have package.json at the root of the repository. Since the Docusaurus files are being moved to the book-frontend directory, we need to ensure the deployment process still works correctly. This can be achieved by either:
1. Using a monorepo structure with workspaces, or
2. Having Vercel build from the book-frontend directory

Option 2 is preferred as it's simpler and maintains the requirement that Docusaurus files are fully encapsulated in the book-frontend directory.

**Alternatives considered**:
- Keep package.json in the root and reference book-frontend: More complex configuration
- Monorepo with workspaces: Overkill for this use case

## Decision: Update Configuration Files for New Directory Structure
**Rationale**: All configuration files that reference Docusaurus assets (like paths to docs, static assets, etc.) need to be updated to reflect the new directory structure. This includes docusaurus.config.js, sidebars.js, and any custom components that reference static assets.

**Alternatives considered**:
- Use symbolic links: Would not properly encapsulate the files as required
- Keep old paths and create aliases: Would violate the requirement that no Docusaurus files exist outside the book-frontend directory

## Decision: Migration Strategy
**Rationale**: The migration process should be done in phases to minimize risk:
1. Create the new book-frontend directory structure
2. Move files using git mv to preserve history
3. Update configuration files to reflect new paths
4. Test the site functionality
5. Verify all links and assets work correctly

**Alternatives considered**:
- All-at-once migration: Higher risk of breaking the site
- Automated script migration: Risk of missing edge cases