"""
Httpx compatibility patch for openai SDK.

This patch fixes the 'proxies' argument issue between openai SDK and httpx.
The httpx.Client no longer accepts 'proxies' argument (it was renamed to 'proxy'),
but some code (including openai internals) still passes 'proxies'.
"""

import httpx

# Store the original __init__ method
_original_init = httpx.Client.__init__

def _patched_init(self, *args, **kwargs):
    """Patched __init__ that handles 'proxies' argument."""
    # Handle the 'proxies' -> 'proxy' conversion for backwards compatibility
    if 'proxies' in kwargs:
        proxies = kwargs.pop('proxies')
        if proxies is not None and 'proxy' not in kwargs:
            # If only one proxy is provided, use it
            if isinstance(proxies, str):
                kwargs['proxy'] = proxies
            elif isinstance(proxies, dict):
                # For dict with multiple proxies, httpx 0.x doesn't support this directly
                # Take the first non-None value
                for key in ['http', 'https']:
                    if key in proxies and proxies[key]:
                        kwargs['proxy'] = proxies[key]
                        break
    return _original_init(self, *args, **kwargs)

# Apply the patch
httpx.Client.__init__ = _patched_init

# Also patch AsyncClient for async operations
_original_async_init = httpx.AsyncClient.__init__

def _patched_async_init(self, *args, **kwargs):
    """Patched AsyncClient __init__ that handles 'proxies' argument."""
    if 'proxies' in kwargs:
        proxies = kwargs.pop('proxies')
        if proxies is not None and 'proxy' not in kwargs:
            if isinstance(proxies, str):
                kwargs['proxy'] = proxies
            elif isinstance(proxies, dict):
                for key in ['http', 'https']:
                    if key in proxies and proxies[key]:
                        kwargs['proxy'] = proxies[key]
                        break
    return _original_async_init(self, *args, **kwargs)

httpx.AsyncClient.__init__ = _patched_async_init
