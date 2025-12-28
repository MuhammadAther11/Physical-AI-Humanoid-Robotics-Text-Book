#!/usr/bin/env python3
"""
Test script to verify content extraction from a documentation page
"""
import requests
from bs4 import BeautifulSoup
from urllib.parse import urlparse

def test_content_extraction():
    """
    Test content extraction from a documentation page
    """
    # Test with a specific documentation page
    url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app/docs/module-1-ros2-nervous-system/ros2-fundamentals"
    
    print(f"Testing content extraction from: {url}")
    
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()
        
        # Try to find the main content area
        content_selectors = [
            'article',
            '.markdown',
            '.theme-doc-markdown',
            '.main-wrapper',
            'main',
            '.container',
            '.docItemContainer',
            '.theme-doc-item',
            '.theme-content',
            '.docs-content',
            '[role="main"]',
            '.content',
        ]
        
        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break
        
        # If no specific content area found, try to find content divs with common patterns
        if not content_element:
            content_divs = soup.find_all('div', class_=lambda x: x and any(keyword in x.lower() for keyword in ['content', 'doc', 'markdown', 'main', 'article']))
            if content_divs:
                # Use the div with the most text content
                content_element = max(content_divs, key=lambda x: len(x.get_text(strip=True)))
        
        # If still no content element found, use the body
        if not content_element:
            content_element = soup.find('body')
        
        if content_element:
            text_content = content_element.get_text(separator=' ')
            
            # Clean up the text
            lines = (line.strip() for line in text_content.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text_content = ' '.join(chunk for chunk in chunks if chunk)
            
            # Extract title
            title = soup.find('title')
            title_text = title.get_text().strip() if title else urlparse(url).path.split('/')[-1]
            
            # Also try to find H1 or H2 as a secondary title source
            if not title_text or len(title_text.strip()) < 5:
                h1 = soup.find('h1')
                if h1:
                    title_text = h1.get_text().strip()
                elif not title_text or len(title_text.strip()) < 5:
                    h2 = soup.find('h2')
                    if h2:
                        title_text = h2.get_text().strip()
            
            print(f"Title: {title_text}")
            print(f"Content length: {len(text_content)} characters")
            print(f"Content preview: {text_content[:500]}...")
            
            if len(text_content) > 0:
                print("\n✓ Content extraction successful!")
                return True
            else:
                print("\n✗ No content extracted")
                return False
        else:
            print("No content found")
            return False
            
    except requests.RequestException as e:
        print(f"Error fetching page: {e}")
        return False
    except Exception as e:
        print(f"Error processing page: {e}")
        return False

if __name__ == "__main__":
    success = test_content_extraction()
    if success:
        print("\nContent extraction is working correctly!")
    else:
        print("\nContent extraction needs further debugging.")