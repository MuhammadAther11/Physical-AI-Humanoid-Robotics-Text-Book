@echo off
REM Script to verify all Docusaurus files have been moved to book-frontend directory

echo Verifying Docusaurus files migration...

REM Check if book-frontend directory exists
if not exist "book-frontend" (
    echo ERROR: book-frontend directory does not exist
    exit /b 1
)

REM Check for important Docusaurus files in the new location
if not exist "book-frontend\docusaurus.config.ts" (
    echo WARNING: docusaurus.config.ts not found in book-frontend
) else (
    echo OK: docusaurus.config.ts found in book-frontend
)

if not exist "book-frontend\package.json" (
    echo WARNING: package.json not found in book-frontend
) else (
    echo OK: package.json found in book-frontend
)

if not exist "book-frontend\sidebars.ts" (
    echo WARNING: sidebars.ts not found in book-frontend
) else (
    echo OK: sidebars.ts found in book-frontend
)

if not exist "book-frontend\src" (
    echo WARNING: src directory not found in book-frontend
) else (
    echo OK: src directory found in book-frontend
)

if not exist "book-frontend\docs" (
    echo WARNING: docs directory not found in book-frontend
) else (
    echo OK: docs directory found in book-frontend
)

if not exist "book-frontend\static" (
    echo WARNING: static directory not found in book-frontend
) else (
    echo OK: static directory found in book-frontend
)

if not exist "book-frontend\blog" (
    echo WARNING: blog directory not found in book-frontend
) else (
    echo OK: blog directory found in book-frontend
)

echo Migration verification complete.