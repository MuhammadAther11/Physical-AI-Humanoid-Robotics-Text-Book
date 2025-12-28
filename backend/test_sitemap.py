#!/usr/bin/env python3
"""
Test script to verify sitemap parsing functionality
"""
import requests
import xml.etree.ElementTree as ET
from urllib.parse import urljoin, urlparse

def test_sitemap_parsing():
    """
    Test the sitemap parsing functionality with the provided URL
    """
    sitemap_url = "https://physical-ai-humanoid-robotics-text-eosin.vercel.app/sitemap.xml"
    
    print(f"Fetching sitemap from: {sitemap_url}")
    
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
        
        print(f"Found {len(urls)} URLs in sitemap:")
        for i, url in enumerate(urls[:10], 1):  # Print first 10 URLs
            print(f"  {i}. {url}")
        
        if len(urls) > 10:
            print(f"  ... and {len(urls) - 10} more URLs")
        
        return urls
        
    except requests.RequestException as e:
        print(f"Error fetching sitemap: {e}")
        return []
    except ET.ParseError as e:
        print(f"Error parsing sitemap XML: {e}")
        return []

if __name__ == "__main__":
    test_sitemap_parsing()