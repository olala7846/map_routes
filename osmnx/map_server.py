"""
Python map server

Usage:
  python map_server.py --port=8080 --api_key=<google_maps_api_key>

  and then visit localhost:8080/maps.html
"""

from http.server import BaseHTTPRequestHandler, HTTPServer, SimpleHTTPRequestHandler
import logging
import socketserver
from absl import app
from absl import flags

FLAGS = flags.FLAGS
flags.DEFINE_integer('port', 8080, 'Port to server HTTP server')
flags.DEFINE_string('api_key', 'fake_api_key', 'Valid Google Maps JavaScript API key')

HOST_NAME = 'localhost'


DirectorySimpleHandler = SimpleHTTPRequestHandler

class RouteServerHandler(SimpleHTTPRequestHandler):
  def do_GET(self):
    if self.path == '/hello':
      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      self.wfile.write(bytes("<h1>Hello World</h1>", "utf-8"))

    if self.path == '/maps.html':
      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      with open('maps.html') as html_file:
        html_template = html_file.read()
        # TODO(olala7846): find a real template engine instead of using string replace.
        response_html = html_template.replace('{API_KEY}', FLAGS.api_key)
        self.wfile.write(bytes(response_html, "utf-8"))

    else:
      super().do_GET()


def main(argv):
  web_server = HTTPServer((HOST_NAME, FLAGS.port), RouteServerHandler)

  try:
    logging.info('Web server starting at localhost:%d', FLAGS.port)
    web_server.serve_forever()
  except KeyboardInterrupt:
    logging.info('Turning down server ...')

  web_server.server_close()
  logging.info('Server stopped')


if __name__ == '__main__':
  app.run(main)
