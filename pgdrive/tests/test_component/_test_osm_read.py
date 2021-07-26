from pyrosm import OSM, get_data
import osmnx as ox

# Initialize the reader
osm = OSM("cpii.osm.pbf")

# Get all walkable roads and the nodes
nodes, edges = osm.get_network(nodes=True, network_type="driving")
# Create NetworkX graph
G = osm.to_graph(nodes, edges, graph_type="networkx")
ox.plot_graph(G)
ccc = edges.head(40)
print(ccc.geometry)