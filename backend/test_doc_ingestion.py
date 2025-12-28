#!/usr/bin/env python3
"""
Test script to specifically test documentation page content extraction
"""
import os
import sys
import logging
from urllib.parse import urlparse
import requests
import xml.etree.ElementTree as ET
from bs4 import BeautifulSoup

# Add the backend directory to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
docusaurus_site_url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app"

def get_all_urls_from_sitemap(sitemap_url: str) -> list:
    """
    Parse sitemap.xml to extract all documentation URLs
    """
    logger.info(f"Fetching sitemap from: {sitemap_url}")
    
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
                # Fix placeholder domain in sitemap - replace with actual domain
                if url and 'your-docusaurus-site.example.com' in url:
                    url = url.replace('your-docusaurus-site.example.com', urlparse(docusaurus_site_url).netloc)
                
                # Only include URLs that are part of the Docusaurus site
                if url and url.startswith(docusaurus_site_url):
                    urls.append(url)
        
        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls
        
    except requests.RequestException as e:
        logger.error(f"Error fetching sitemap: {e}")
        return []
    except ET.ParseError as e:
        logger.error(f"Error parsing sitemap XML: {e}")
        return []

def extract_text_from_url(url: str) -> dict:
    """
    Extract clean text content from a given URL
    """
    logger.info(f"Extracting content from: {url}")
    
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
        
        # Try to find the main content area of the Docusaurus page
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
            # Extract text content
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
            
            return {
                'url': url,
                'title': title_text,
                'content': text_content,
                'word_count': len(text_content.split())
            }
        else:
            logger.warning(f"No content found for URL: {url}")
            return {
                'url': url,
                'title': urlparse(url).path.split('/')[-1],
                'content': '',
                'word_count': 0
            }
            
    except requests.RequestException as e:
        logger.error(f"Error fetching content from {url}: {e}")
        return {
            'url': url,
            'title': urlparse(url).path.split('/')[-1],
            'content': '',
            'word_count': 0
        }
    except Exception as e:
        logger.error(f"Error processing content from {url}: {e}")
        return {
            'url': url,
            'title': urlparse(url).path.split('/')[-1],
            'content': '',
            'word_count': 0
        }

def main():
    """
    Main function to test documentation content extraction
    """
    logger.info("Starting documentation content extraction test...")
    
    # Step 1: Get all URLs from sitemap
    sitemap_url = f"{docusaurus_site_url}/sitemap.xml"
    urls = get_all_urls_from_sitemap(sitemap_url)
    
    if not urls:
        logger.error("No URLs found in sitemap. Please verify the sitemap URL.")
        return False
    
    # Filter for documentation URLs
    doc_urls = [url for url in urls if '/docs/' in url]
    logger.info(f"Found {len(doc_urls)} documentation URLs out of {len(urls)} total URLs")
    
    if not doc_urls:
        logger.error("No documentation URLs found in sitemap.")
        return False
    
    logger.info(f"Testing first 3 documentation URLs:")
    
    # Process first 3 documentation URLs
    test_doc_urls = doc_urls[:3]
    total_content_chars = 0
    total_chunks = 0
    
    for i, url in enumerate(test_doc_urls, 1):
        logger.info(f"\nProcessing documentation URL {i}/{len(test_doc_urls)}: {url}")
        
        # Extract content from URL
        content_data = extract_text_from_url(url)
        
        if not content_data['content']:
            logger.warning(f"No content extracted from {url}, skipping...")
            continue
        
        logger.info(f"  - Title: {content_data['title']}")
        logger.info(f"  - Content length: {len(content_data['content'])} characters")
        logger.info(f"  - Word count: {content_data['word_count']}")
        
        # Check if this is a substantial content page
        if len(content_data['content']) > 100:  # More than 100 chars indicates real content
            logger.info(f"  ✓ Substantial content found!")
        else:
            logger.info(f"  ⚠ Minimal content found - might be a placeholder or index page")
        
        total_content_chars += len(content_data['content'])
        # For this test, we're just counting that we processed the page
        total_chunks += 1
    
    logger.info(f"\nDocumentation test completed!")
    logger.info(f"- Processed {len(test_doc_urls)} documentation URLs")
    logger.info(f"- Total content characters: {total_content_chars}")
    logger.info(f"- Documentation pages processed: {total_chunks}")
    
    if total_content_chars > 0:
        logger.info("✓ Documentation content extraction is working correctly!")
        return True
    else:
        logger.info("✗ Documentation content extraction needs debugging.")
        return False

if __name__ == "__main__":
    success = main()
    if success:
        print("\nDocumentation content extraction test PASSED!")
        print("The pipeline is correctly ingesting documentation content from the sitemap.")
    else:
        print("\nDocumentation content extraction test FAILED!")