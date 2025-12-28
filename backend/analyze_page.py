#!/usr/bin/env python3
"""
Test script to analyze the structure of the documentation pages
"""
import requests
from bs4 import BeautifulSoup

def analyze_page_structure():
    """
    Analyze the structure of a documentation page to understand how to extract content
    """
    # Test with a specific documentation page
    url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app/docs/module-1-ros2-nervous-system/ros2-fundamentals"
    
    print(f"Analyzing page structure: {url}")
    
    try:
        response = requests.get(url)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Print the page title
        title = soup.find('title')
        print(f"Page title: {title.text if title else 'No title found'}")
        
        # Look for common content containers in Docusaurus sites
        content_selectors = [
            'article',
            '.markdown',
            '.theme-doc-markdown',
            '.main-wrapper',
            'main',
            '.container',
            '.docItemContainer',
            '.theme-doc-item',
            '.markdown',
            '[class*="docItem"]',
            '[class*="markdown"]',
            '.content',
            '.docs-content',
            '.theme-content'
        ]
        
        print("\nLooking for content in common Docusaurus selectors:")
        for selector in content_selectors:
            elements = soup.select(selector)
            if elements:
                print(f"  Found {len(elements)} element(s) with selector '{selector}':")
                for i, elem in enumerate(elements[:2]):  # Show first 2 elements
                    text_content = elem.get_text(strip=True)[:200]  # First 200 chars
                    print(f"    {i+1}. '{text_content}...'")
            else:
                print(f"  No elements found with selector '{selector}'")
        
        # Also look for content in other common places
        print("\nOther common content areas:")
        body_content = soup.find('body')
        if body_content:
            # Look for content divs with common patterns
            content_divs = body_content.find_all('div', class_=lambda x: x and ('content' in x or 'doc' in x or 'markdown' in x))
            print(f"  Found {len(content_divs)} divs with content-related classes")
            for i, div in enumerate(content_divs[:3]):  # Show first 3
                text_content = div.get_text(strip=True)[:200]
                print(f"    {i+1}. Class: {div.get('class', [])}, Content: '{text_content}...'")
        
        # Print the first 1000 characters of the entire page for reference
        print(f"\nFirst 1000 characters of page content:")
        print(soup.get_text()[:1000] + "...")
        
    except requests.RequestException as e:
        print(f"Error fetching page: {e}")
    except Exception as e:
        print(f"Error analyzing page: {e}")

if __name__ == "__main__":
    analyze_page_structure()