#!/usr/bin/env python3
"""
Test script to verify sitemap parsing and content extraction
"""
import requests
import xml.etree.ElementTree as ET
from urllib.parse import urlparse
from bs4 import BeautifulSoup

def test_sitemap_and_content_extraction():
    """
    Test both sitemap parsing and content extraction
    """
    # Test sitemap parsing
    sitemap_url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app/sitemap.xml"
    print(f"Testing sitemap parsing from: {sitemap_url}")
    
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()
        
        # Parse the XML content
        root = ET.fromstring(response.content)
        
        # Define the namespace used in sitemaps
        namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        
        # Extract all URLs from the sitemap
        urls = []
        for url_element in root.findall('sitemap:url', namespace):
            loc_element = url_element.find('sitemap:loc', namespace)
            if loc_element is not None:
                url = loc_element.text
                urls.append(url)
        
        print(f"Found {len(urls)} URLs in sitemap")
        
        # Test content extraction on a few URLs
        test_urls = urls[:3]  # Test first 3 URLs
        print(f"\nTesting content extraction on {len(test_urls)} URLs:")
        
        for i, url in enumerate(test_urls, 1):
            print(f"\n{i}. Testing: {url}")
            
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
                    
                    print(f"   Title: {title_text}")
                    print(f"   Content length: {len(text_content)} characters")
                    print(f"   Content preview: {text_content[:200]}...")
                else:
                    print("   No content found")
                    
            except Exception as e:
                print(f"   Error extracting content: {e}")
        
    except requests.RequestException as e:
        print(f"Error fetching sitemap: {e}")
    except ET.ParseError as e:
        print(f"Error parsing sitemap XML: {e}")

if __name__ == "__main__":
    test_sitemap_and_content_extraction()