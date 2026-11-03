"""Load the console page and serve its three explicitly allowed static resources."""
from pathlib import Path
from urllib.parse import urlsplit

STATIC_ROOT = Path(__file__).resolve().parent / 'static'
ASSETS = {
    '/static/style.css': ('style.css', 'text/css; charset=utf-8'),
    '/static/app.js': ('app.js', 'application/javascript; charset=utf-8'),
}

def load_index():
    """Read the submitted page; the backend continues serving it at /."""
    return (STATIC_ROOT / 'index.html').read_text(encoding='utf-8')

def serve_static(handler, path):
    """A fixed URL-to-file map prevents arbitrary filesystem reads."""
    target = ASSETS.get(urlsplit(path).path)
    if target is None:
        handler.send_error(404, 'Unknown static resource')
        return
    name, content_type = target
    data = (STATIC_ROOT / name).read_bytes()
    handler.send_response(200)
    handler.send_header('Content-Type', content_type)
    handler.send_header('Content-Length', str(len(data)))
    handler.send_header('Cache-Control', 'no-store')
    handler.end_headers()
    handler.wfile.write(data)
