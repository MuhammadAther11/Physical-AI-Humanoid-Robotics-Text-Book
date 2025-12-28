#!/usr/bin/env python3
"""
Test script to verify sitemap parsing with domain fix
"""
import requests
import xml.etree.ElementTree as ET
from urllib.parse import urlparse

def test_sitemap_parsing_with_domain_fix():
    """
    Test sitemap parsing with domain replacement
    """
    sitemap_url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app/sitemap.xml"
    docusaurus_site_url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app"
    
    print(f"Testing sitemap parsing from: {sitemap_url}")
    print(f"Expected domain: {docusaurus_site_url}")
    
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
        
        print(f"Found {len(urls)} valid URLs in sitemap after domain fix")
        
        # Show first 10 URLs
        for i, url in enumerate(urls[:10], 1):
            print(f"  {i}. {url}")
        
        if len(urls) > 10:
            print(f"  ... and {len(urls) - 10} more URLs")
        
        # Test a specific documentation URL
        doc_url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app/docs/module-1-ros2-nervous-system/ros2-fundamentals"
        if doc_url in urls:
            print(f"\n✓ Found target documentation URL: {doc_url}")
        else:
            print(f"\n✗ Target documentation URL not found: {doc_url}")
            # Check if similar URLs exist
            doc_urls = [url for url in urls if 'docs' in url]
            print(f"Found {len(doc_urls)} documentation URLs:")
            for i, url in enumerate(doc_urls[:5], 1):
                print(f"  {i}. {url}")
            if len(doc_urls) > 5:
                print(f"  ... and {len(doc_urls) - 5} more")
        
        return urls
        
    except requests.RequestException as e:
        print(f"Error fetching sitemap: {e}")
        return []
    except ET.ParseError as e:
        print(f"Error parsing sitemap XML: {e}")
        return []

if __name__ == "__main__":
    test_sitemap_parsing_with_domain_fix()