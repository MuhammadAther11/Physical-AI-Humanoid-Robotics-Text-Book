# UI Component Contract

## Custom Docusaurus Components

### Theme Override Components

#### Navbar Component
- **Purpose**: Custom navigation bar with enhanced UI
- **Props Interface**:
  ```typescript
  interface NavbarProps {
    logo?: {
      alt: string;
      src: string;
      href?: string;
    };
    title: string;
    items: Array<{
      to?: string;
      href?: string;
      label: string;
      position: 'left' | 'right';
      activeBasePath?: string;
    }>;
  }
  ```

#### Footer Component
- **Purpose**: Custom footer with improved design
- **Props Interface**:
  ```typescript
  interface FooterProps {
    copyright: string;
    links: Array<{
      title: string;
      items: Array<{
        label: string;
        to?: string;
        href?: string;
      }>;
    }>;
  }
  ```

#### Layout Component
- **Purpose**: Custom page layout with enhanced spacing and typography
- **Props Interface**:
  ```typescript
  interface LayoutProps {
    title?: string;
    description?: string;
    wrapperClassName?: string;
    children: React.ReactNode;
  }
  ```

### Content Display Components

#### DocRoot Component
- **Purpose**: Enhanced documentation root layout
- **Props Interface**:
  ```typescript
  interface DocRootProps {
    activeDoc: {
      frontMatter: Record<string, any>;
      metadata: {
        id: string;
        title: string;
        description: string;
      };
    };
    activeVersion: {
      name: string;
      label: string;
    };
  }
  ```

#### MDX Components
- **Purpose**: Custom MDX components for enhanced content display
- **Props Interface**:
  ```typescript
  interface MDXComponents {
    wrapper: React.ComponentType;
    h1: React.ComponentType;
    h2: React.ComponentType;
    h3: React.ComponentType;
    p: React.ComponentType;
    a: React.ComponentType;
    img: React.ComponentType;
    code: React.ComponentType;
    pre: React.ComponentType;
  }
  ```