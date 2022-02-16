"""
Python map server

Usage:
  python map_server.py --port=8080 --api_key=<google_maps_api_key>

  and then visit localhost:8080/maps.html
"""

from absl import app
from absl import flags
from http.server import BaseHTTPRequestHandler, HTTPServer, SimpleHTTPRequestHandler
from pathfinder import AStarPathFinder
from urllib.parse import urlparse, parse_qs
import json
import logging
import socketserver
import utils
import osmnx as ox

FLAGS = flags.FLAGS
flags.DEFINE_integer('port', 8080, 'Port to server HTTP server')
flags.DEFINE_string(
  'region', 'Greater Taipei', 'Region name of road network to use')

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
        response_html = html_template.replace(
          '{API_KEY}', self.server.secrets['google_api_key'])
        self.wfile.write(bytes(response_html, "utf-8"))

    if self.path.startswith('/route?'):
      # Example http://localhost:8080/route?orig=1,2&dest=4,5
      parsed_url = urlparse(self.path)
      query_dict = parse_qs(parsed_url.query)
      print(query_dict)
      source = [float(x) for x in query_dict['orig'][0].split(',')]
      target = [float(x) for x in query_dict['dest'][0].split(',')]
      route = self.server.pathfinder.route(orig=source, dest=target)

      self.send_response(200)
      self.send_header("Content-Type", "application/json")
      self.end_headers()
      self.wfile.write(bytes(json.dumps({"route": route}), "utf-8"))

    else:
      super().do_GET()


def main(argv):
  secrets = None
  with open('./secrets.json', 'r') as secret_file:
    secrets = json.loads(secret_file.read())
  if secrets is None:
    raise ValueError("Failed to load ./secrets.json")

  web_server = HTTPServer((HOST_NAME, FLAGS.port), RouteServerHandler)
  web_server.secrets = secrets

  road_network = utils.load_road_network(FLAGS.region, extension='graphml')
  web_server.pathfinder = AStarPathFinder(road_network)

  try:
    logging.info('Web server starting at localhost:%d', FLAGS.port)
    web_server.serve_forever()
  except KeyboardInterrupt:
    logging.info('Turning down server ...')

  web_server.server_close()
  logging.info('Server stopped')


if __name__ == '__main__':
  app.run(main)
