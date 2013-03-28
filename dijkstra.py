import nhpn
import heap_id
import time

# ----------------------- Auxiliary Functions -----------------------

def node_by_name(nodes, city, state):
    """Returns the first node that matches specified location.
    The description of the node should include description,
    and the state of the node should match the state.
    If there is no such node, returns None."""

    for node in nodes:
        if node.state == state:
            if city in node.description:
                return node

    return None

def equal(node1, node2):
    """Returns True if node1 and node2 are equal, False otherwise."""
    return node1.latitude == node2.latitude and \
           node1.longitude == node2.longitude

def distance(node1, node2):
    """Returns the distance between node1 and node2, ignoring the
    earth's curvature."""

    latitude_diff = node1.latitude - node2.latitude
    longitude_diff = node1.longitude - node2.longitude
    return (latitude_diff**2 + longitude_diff**2)**.5

def _heap_insert(node):
    """Assumes key is already in node.distance."""
    node.ID = heap.insert(node.distance)
    ID_to_node[node.ID] = node

def _heap_decrease_key(node):
    """Assumes new key is already in node.distance."""
    heap.decrease_key_using_id(node.ID, node.distance)

def _heap_extract_min():
    """Returns the node associated with the minimum key."""
    ID = heap.extract_min_with_id()[1]
    node = ID_to_node[ID]
    return node

def reconstruct_path(source, destination):
    """Returns list of nodes in path from source to destination. Uses
    node.parent pointers."""
    node = destination
    p = [destination]
    while node != source:
        node = node.parent
        p.append(node)
    p.reverse()
    return p

def shortest_path(nodes, edges, weight, source, destination):
    """Returns list of nodes in path from source to destination. Uses
    node.parent pointers."""
    dijkstra(nodes, edges, weight, source)
    return reconstruct_path(source, destination)

# ------------------------- Shortest Paths -------------------------

def dijkstra(nodes, edges, weight, source):
    """Finds the shortest path from source to all nodes in the
    graph (nodes and edges), where the weight on edge (u, v) is
    given by weight(u, v). Assumes that all weights are
    non-negative.

    At the end of the algorithm:
    - node.visited is True if the node is visited, False otherwise.
    (Note: node is visited if shortest path to it is computed.)
    - node.distance is set to the shortest path length from source
        to node if node is visited, or not present otherwise.
    - node.parent is set to the previous node on the shortest path
        from source to node if node is visited, or not present otherwise.        

    Returns the number of visited nodes.
    """

    global heap, ID_to_node
    
    # Initialize.
    for node in nodes:
        node.marked = False # node is marked if it has been added to the heap.
        node.visited = False # node is visited if the shortest path from source
                             # to node is found (i.e., if removed from heap).
    num_visited = 0

    # Now run dijkstra's algorithm.

    ID_to_node = {}
    heap = heap_id.heap_id()
    source.distance = 0
    _heap_insert(source)
    source.marked = True

    while(heap.heapsize > 0):
        current = _heap_extract_min()
        current.visited = True
        num_visited = num_visited + 1
        for node in current.adj: # Relax nodes adjacent to current.
            if not node.visited:
                new_distance = weight(current, node) + current.distance
                if node.marked:
                    if new_distance < node.distance:
                        node.distance = new_distance
                        _heap_decrease_key(node)
                        node.parent = current
                else:
                    node.distance = new_distance
                    _heap_insert(node)
                    node.marked = True
                    node.parent = current

    return num_visited

def dijkstra_early_stop(nodes, edges, weight, source, destination):
    """Performs Dijkstra's algorithm until it finds the shortest
    path from source to destination in the graph (nodes and edges),
    where the weight on edge (u, v) is given by weight(u, v).
    Assumes that all weights are non-negative.

    At the end of the algorithm:
    - node.visited is True if the node is visited, False otherwise.
    (Note: node is visited if shortest path to it is computed.)
    - node.distance is set to the shortest path length from source
        to node if node is visited, or not present otherwise.
    - node.parent is set to the previous not on the shortest path
        from source to node if node is visited, or not present otherwise.        

    Returns the number of visited nodes.
    """
    
    global heap, ID_to_node
    
    # Initialize.
    for node in nodes:
        node.marked = False # node is marked if it has been added to the heap.
        node.visited = False # node is visited if the shortest path from source
                             # to node is found (i.e., if removed from heap).
    num_visited = 0

    # Now run dijkstra's algorithm.

    ID_to_node = {}
    heap = heap_id.heap_id()
    source.distance = 0
    _heap_insert(source)
    source.marked = True

    while(heap.heapsize > 0):
        current = _heap_extract_min()
        current.visited = True
        num_visited = num_visited + 1
        if current == destination:
            return num_visited
        for node in current.adj: # Relax nodes adjacent to current.
            if not node.visited:
                new_distance = weight(current, node) + current.distance
                if node.marked:
                    if new_distance < node.distance:
                        node.distance = new_distance
                        _heap_decrease_key(node)
                        node.parent = current
                else:
                    node.distance = new_distance
                    _heap_insert(node)
                    node.marked = True
                    node.parent = current

    return num_visited

def compute_landmark_distances(nodes, edges, weight, landmark):
    """Computes shortest path length to landmark for all nodes
    in the graph (nodes and edges), where the weight on edge (u, v)
    is given by weight(u, v). Assumes that all weights are non-negative.

    At the end of the algorithm, for each node, node.land_distance is
    set to the shortest path length from node to landmark, or to 10^9
    if no such path exist. 
    """

    dijkstra(nodes, edges, weight, landmark)
    for n in nodes:
        if n.visited == True:
            n.land_distance = n.distance
        else:
            n.land_distance = 10**9

    return # NOT IMPLEMENTED YET

def dijkstra_with_potentials(nodes, edges, weight, source, destination):
    """Performs Dijkstra's algorithm on a graph with weights modified
    according to the potentials method, until it finds the shortest
    path from source to destination in the graph (nodes and edges),
    where the weight on edge (u, v) is given by weight(u, v).
    Assumes that all weights are non-negative.

    Assumes that node.land_distance is already computed for each node.

    At the end of the algorithm:
    - node.visited is True if the node is visited, False otherwise.
    (Note: node is visited if shortest path to it is computed.)
    - node.distance is set to the shortest path length from source
        to node if node is visited, or not present otherwise.
    - node.parent is set to the previous not on the shortest path
        from source to node if node is visited, or not present otherwise.        

    Returns the number of visited nodes.
    """

    global heap, ID_to_node
    
    # Initialize.
    for node in nodes:
        node.marked = False # node is marked if it has been added to the heap.
        node.visited = False # node is visited if the shortest path from source
                             # to node is found (i.e., if removed from heap).
    num_visited = 0

    # Now run dijkstra's algorithm.

    ID_to_node = {}
    heap = heap_id.heap_id()
    source.distance = 0
    _heap_insert(source)
    source.marked = True

    while(heap.heapsize > 0):
        current = _heap_extract_min()
        current.visited = True
        num_visited = num_visited + 1
        if current == destination:
            return num_visited
        for node in current.adj: # Relax nodes adjacent to current.
            if not node.visited:
                new_distance = weight(current, node) - current.land_distance + node.land_distance + current.distance
                if node.marked:
                    if new_distance < node.distance:
                        node.distance = new_distance
                        _heap_decrease_key(node)
                        node.parent = current
                else:
                    node.distance = new_distance
                    _heap_insert(node)
                    node.marked = True
                    node.parent = current

    return num_visited


def compute_multi_landmark_distances(nodes, edges, weight, landmarks):
    """Computes shortest path lengths to landmarks for all nodes.
    in the graph (nodes and edges), where the weight on edge (u, v)
    is given by weight(u, v). Assumes that all weights are non-negative.

    At the end of the algorithm, for each node, node.land_distances[i]
    is set to the shortest path length from node to landmarks[i],
    or to 10^9 if no such path exist. 
    """

    for n in nodes:
        n.land_distance = []
    for l in landmarks:
        dijkstra(nodes, edges, weight, l)
        for n in nodes:
            if n.visited == True:
                n.land_distance.append(node.distance)
            else:
                n.land_distance.append(10**9)

def dijkstra_with_max_potentials(nodes, edges, weight, source, destination):
    """Performs Dijkstra's algorithm on a graph with weights modified
    according to the max-potentials method (with multiple landmarks),
    until it finds the shortest path from source to destination in the
    graph (nodes and edges), where the weight on edge (u, v) is given
    by weight(u, v). Assumes that all weights are non-negative.

    Assumes that node.land_distances is already computed for each node.

    At the end of the algorithm:
    - node.visited is True if the node is visited, False otherwise.
    (Note: node is visited if shortest path to it is computed.)
    - node.distance is set to the shortest path length from source
        to node if node is visited, or not present otherwise.
    - node.parent is set to the previous not on the shortest path
        from source to node if node is visited, or not present otherwise.        

    Returns the number of visited nodes.
    """

    global heap, ID_to_node
    
    # Initialize.
    for node in nodes:
        node.marked = False # node is marked if it has been added to the heap.
        node.visited = False # node is visited if the shortest path from source
                             # to node is found (i.e., if removed from heap).
    num_visited = 0

    # Now run dijkstra's algorithm.

    ID_to_node = {}
    heap = heap_id.heap_id()
    source.distance = 0
    _heap_insert(source)
    source.marked = True

    while(heap.heapsize > 0):
        current = _heap_extract_min()
        current.visited = True
        num_visited = num_visited + 1
        if current == destination:
            return num_visited
        for node in current.adj: # Relax nodes adjacent to current.
            if not node.visited:
                new_distance = weight(current, node) - max(current.land_distance) + max(node.land_distance) + current.distance
                if node.marked:
                    if new_distance < node.distance:
                        node.distance = new_distance
                        _heap_decrease_key(node)
                        node.parent = current
                else:
                    node.distance = new_distance
                    _heap_insert(node)
                    node.marked = True
                    node.parent = current

    return num_visited

# ------------------------- Data Processing ----------------------------

def load_data():
    print "Loading data..."
    start_time = time.clock()
    loader = nhpn.Loader() # Loading the data takes a few seconds.
    lnodes = loader.nodes()
    llinks = loader.links()
    print "Loading data finished after", round(time.clock() - start_time, 3), "seconds"
    return lnodes, llinks

def create_adjacency_lists(nodes, links):
    # Use an adjacency list representation, by putting all vertices
    # adjacent to node in node.adj.
    for node in nodes:
        node.adj = []
    for link in links:
        link.begin.adj.append(link.end)
        link.end.adj.append(link.begin)

def create_edge_set(links):
    # Put edges in a set for faster lookup.
    edge_set = set()
    for link in links:
        edge_set.add( (link.begin, link.end) )
    return edge_set

# --------------------------- Testing ----------------------------------

def test_graph_size():
    num_nodes = len(nodes)
    num_edges = 0
    for node in nodes:
        num_edges += len(node.adj)
    print "Graph size:", num_nodes, "nodes,", num_edges, "edges"

def verifyPath(path, edge_set, source, destination):
    """Verify that path is a valid path from source to destination
    (it's valid if it uses only edges in the edge set)."""
    if source != path[0]:
        print "First node on a path is different form the source"
        return False
    if destination != path[-1]:
        print "Last node on a path is different form the destination"
        return False
    for i in range(len(path)-1):
        if (path[i], path[i+1]) not in edge_set and \
            (path[i+1], path[i]) not in edge_set:
            print "Adjacent nodes in path have no edge between them"
            return False
    return True

def sumPath(path, weight):
    """Compute the sum of weights along a path.  Requires path to
    be valid (see verifyPath)."""
    sum = 0
    for i in range(len(path)-1):
        sum += weight(path[i], path[i+1])
    return sum

def test_dijkstra(source, destination):
    print "\nDijkstra:", source.description, ",", source.state, "->", \
          destination.description, ",", destination.state
    start_time = time.clock()
    num_visited = dijkstra(nodes, links, distance, source)
    print "Finished after", round(time.clock() - start_time, 3), "seconds,", \
          "visited", num_visited, "nodes"
    print "destination.distance =", destination.distance
    path = reconstruct_path(source, destination)
    if verifyPath(path, edge_set, source, destination):
        print "Path has length", sumPath(path, distance), "and consists of", \
              len(path) - 1, "roads"
    else:
        print "Invalid path"

def test_dijkstra_early_stop(source, destination):
    print "\nDijkstra_early_stop:", source.description, ",", source.state, "->", \
          destination.description, ",", destination.state
    start_time = time.clock()
    num_visited = dijkstra_early_stop(nodes, links, distance, source, destination)
    print "Finished after", round(time.clock() - start_time, 3), "seconds,", \
          "visited", num_visited, "nodes"
    print "destination.distance =", destination.distance
    path = reconstruct_path(source, destination)
    if verifyPath(path, edge_set, source, destination):
        print "Path has length", sumPath(path, distance), "and consists of", \
              len(path) - 1, "roads"
    else:
        print "Invalid path"

def test_dijkstra_with_potentials(source, destination):
    print "\nDijkstra_with_potentials:", source.description, ",", source.state, "->", \
          destination.description, ",", destination.state
    start_time = time.clock()
    num_visited = dijkstra_with_potentials(nodes, links, distance, source, destination)
    print "Finished after", round(time.clock() - start_time, 3), "seconds,", \
          "visited", num_visited, "nodes"
    print "destination.distance = ", destination.distance
    path = reconstruct_path(source, destination)
    if verifyPath(path, edge_set, source, destination):
        print "Path has length", sumPath(path, distance), "and consists of", \
              len(path) - 1, "roads"
    else:
        print "Invalid path"

def test_dijkstra_with_max_potentials(source, destination):
    print "\nDijkstra_with_max_potentials:", source.description, ",", source.state, "->", \
          destination.description, ",", destination.state
    start_time = time.clock()
    num_visited = dijkstra_with_max_potentials(nodes, links, distance, source, destination)
    print "Finished after", round(time.clock() - start_time, 3), "seconds,", \
          "visited", num_visited, "nodes"
    print "destination.distance = ", destination.distance
    path = reconstruct_path(source, destination)
    if verifyPath(path, edge_set, source, destination):
        print "Path has length", sumPath(path, distance), "and consists of", \
              len(path) - 1, "roads"
    else:
        print "Invalid path"

# ------------------------ Testing Scenarios ---------------------------

def define_locations():
    global cambridge, pasadena, portland_maine, portland_oregon, miami, new_haven, \
           el_dorado, watertown, san_francisco, chicago, bellevue
    cambridge = node_by_name(nodes, 'CAMBRIDGE', 'MA')
    pasadena = node_by_name(nodes, 'PASADENA', 'CA')
    portland_maine = node_by_name(nodes, 'PORTLAND', 'ME')
    portland_oregon = node_by_name(nodes, 'PORTLAND', 'OR')
    new_haven = node_by_name(nodes, 'NEW HAVEN', 'CT')
    watertown = node_by_name(nodes, 'WATERTOWN', 'MA')
    san_francisco = node_by_name(nodes, 'SAN FRANCISCO', 'CA')
    chicago = node_by_name(nodes, 'CHICAGO', 'IL')
    bellevue = node_by_name(nodes, 'BELLEVUE', 'WA')

def test_a():
    print "\n\n################################################"
    print "### Testing Part (a) -- Dijkstra's Algorithm ###"
    print "################################################"    
    test_dijkstra(pasadena, cambridge)
    test_dijkstra(cambridge, pasadena)
    test_dijkstra(pasadena, bellevue)
    test_dijkstra(portland_maine, cambridge)
    test_dijkstra(new_haven, cambridge)
    test_dijkstra(portland_maine, san_francisco)
    test_dijkstra(cambridge, watertown)
    test_dijkstra(chicago, portland_maine)
    test_dijkstra(portland_oregon, chicago)

def test_b():
    print "\n\n################################################################"
    print "### Testing Part (b) -- Dijkstra's Algorithm With Early Stop ###"
    print "################################################################"
    test_dijkstra_early_stop(pasadena, cambridge)
    test_dijkstra_early_stop(cambridge, pasadena)
    test_dijkstra_early_stop(pasadena, bellevue)
    test_dijkstra_early_stop(portland_maine, cambridge)
    test_dijkstra_early_stop(new_haven, cambridge)
    test_dijkstra_early_stop(portland_maine, san_francisco)
    test_dijkstra_early_stop(cambridge, watertown)
    test_dijkstra_early_stop(chicago, portland_maine)
    test_dijkstra_early_stop(portland_oregon, chicago)

def test_d():
    print "\n\n################################################################"
    print "### Testing Part (d) -- Dijkstra's Algorithm With 1 Landmark ###"
    print "################################################################"

    print "\nLandmark:", san_francisco.description, san_francisco.state
    print "Computing landmark distances...",
    compute_landmark_distances(nodes, links, distance, san_francisco)
    print "done"
    test_dijkstra_with_potentials(pasadena, cambridge)
    test_dijkstra_with_potentials(cambridge, pasadena)
    test_dijkstra_with_potentials(pasadena, bellevue)
    test_dijkstra_with_potentials(portland_maine, cambridge)
    test_dijkstra_with_potentials(new_haven, cambridge)
    test_dijkstra_with_potentials(portland_maine, san_francisco)
    test_dijkstra_with_potentials(cambridge, watertown)
    test_dijkstra_with_potentials(chicago, portland_maine)
    test_dijkstra_with_potentials(portland_oregon, chicago)

    print "\nLandmark:", portland_oregon.description, portland_oregon.state
    print "Computing landmark distances...",
    compute_landmark_distances(nodes, links, distance, portland_oregon)
    print "done"
    test_dijkstra_with_potentials(pasadena, cambridge)
    test_dijkstra_with_potentials(cambridge, pasadena)
    test_dijkstra_with_potentials(pasadena, bellevue)
    test_dijkstra_with_potentials(portland_maine, cambridge)
    test_dijkstra_with_potentials(new_haven, cambridge)
    test_dijkstra_with_potentials(portland_maine, san_francisco)
    test_dijkstra_with_potentials(cambridge, watertown)
    test_dijkstra_with_potentials(chicago, portland_maine)
    test_dijkstra_with_potentials(portland_oregon, chicago)

    print "\nLandmark:", chicago.description, chicago.state
    print "Computing landmark distances...",
    compute_landmark_distances(nodes, links, distance, chicago)
    print "done"
    test_dijkstra_with_potentials(pasadena, cambridge)
    test_dijkstra_with_potentials(cambridge, pasadena)
    test_dijkstra_with_potentials(pasadena, bellevue)
    test_dijkstra_with_potentials(portland_maine, cambridge)
    test_dijkstra_with_potentials(new_haven, cambridge)
    test_dijkstra_with_potentials(portland_maine, san_francisco)
    test_dijkstra_with_potentials(cambridge, watertown)
    test_dijkstra_with_potentials(chicago, portland_maine)
    test_dijkstra_with_potentials(portland_oregon, chicago)

    print "\nLandmark:", new_haven.description, new_haven.state
    print "Computing landmark distances...",
    test_dijkstra_with_potentials(pasadena, cambridge)
    test_dijkstra_with_potentials(cambridge, pasadena)
    test_dijkstra_with_potentials(pasadena, bellevue)
    test_dijkstra_with_potentials(portland_maine, cambridge)
    test_dijkstra_with_potentials(new_haven, cambridge)
    test_dijkstra_with_potentials(portland_maine, san_francisco)
    test_dijkstra_with_potentials(cambridge, watertown)
    test_dijkstra_with_potentials(chicago, portland_maine)
    test_dijkstra_with_potentials(portland_oregon, chicago)

    print "\nLandmark:", watertown.description, watertown.state
    print "Computing landmark distances...",
    compute_landmark_distances(nodes, links, distance, watertown)
    print "done"
    test_dijkstra_with_potentials(pasadena, cambridge)
    test_dijkstra_with_potentials(cambridge, pasadena)
    test_dijkstra_with_potentials(pasadena, bellevue)
    test_dijkstra_with_potentials(portland_maine, cambridge)
    test_dijkstra_with_potentials(new_haven, cambridge)
    test_dijkstra_with_potentials(portland_maine, san_francisco)
    test_dijkstra_with_potentials(cambridge, watertown)
    test_dijkstra_with_potentials(chicago, portland_maine)
    test_dijkstra_with_potentials(portland_oregon, chicago)

    print "\nLandmark:", portland_maine.description, portland_maine.state
    print "Computing landmark distances...",
    compute_landmark_distances(nodes, links, distance, portland_maine)
    print "done"
    test_dijkstra_with_potentials(pasadena, cambridge)
    test_dijkstra_with_potentials(cambridge, pasadena)
    test_dijkstra_with_potentials(pasadena, bellevue)
    test_dijkstra_with_potentials(portland_maine, cambridge)
    test_dijkstra_with_potentials(new_haven, cambridge)
    test_dijkstra_with_potentials(portland_maine, san_francisco)
    test_dijkstra_with_potentials(cambridge, watertown)
    test_dijkstra_with_potentials(chicago, portland_maine)
    test_dijkstra_with_potentials(portland_oregon, chicago)

def test_f():
    print "\n\n########################################################################"
    print "### Testing Part (f) -- Dijkstra's Algorithm With Multiple Landmarks ###"
    print "########################################################################"

    print "\nLandmarks: {", watertown.description, watertown.state, ",", \
          portland_maine.description, portland_maine.state, ",", \
          san_francisco.description, san_francisco.state, ",", \
          portland_oregon.description, portland_oregon.state, ",", \
          chicago.description, chicago.state, ",", \
          new_haven.description, new_haven.state, "}"
    print "Computing landmarks distances...",
    compute_multi_landmark_distances(nodes, links, distance, \
                                     [watertown, portland_maine, san_francisco, \
                                      portland_oregon, chicago, new_haven])
    print "done"
    test_dijkstra_with_max_potentials(pasadena, cambridge)
    test_dijkstra_with_max_potentials(cambridge, pasadena)
    test_dijkstra_with_max_potentials(pasadena, bellevue)
    test_dijkstra_with_max_potentials(portland_maine, cambridge)
    test_dijkstra_with_max_potentials(new_haven, cambridge)
    test_dijkstra_with_max_potentials(portland_maine, san_francisco)
    test_dijkstra_with_max_potentials(cambridge, watertown)
    test_dijkstra_with_max_potentials(chicago, portland_maine)
    test_dijkstra_with_max_potentials(portland_oregon, chicago)

    print "\nLandmarks: {", watertown.description, watertown.state, ",", \
          portland_maine.description, portland_maine.state, ",", \
          new_haven.description, new_haven.state, "}"
    print "Computing landmarks distances...",
    compute_multi_landmark_distances(nodes, links, distance, \
                                     [watertown, portland_maine, new_haven])
    print "done"
    test_dijkstra_with_max_potentials(pasadena, cambridge)
    test_dijkstra_with_max_potentials(cambridge, pasadena)
    test_dijkstra_with_max_potentials(pasadena, bellevue)
    test_dijkstra_with_max_potentials(portland_maine, cambridge)
    test_dijkstra_with_max_potentials(new_haven, cambridge)
    test_dijkstra_with_max_potentials(portland_maine, san_francisco)
    test_dijkstra_with_max_potentials(cambridge, watertown)
    test_dijkstra_with_max_potentials(chicago, portland_maine)
    test_dijkstra_with_max_potentials(portland_oregon, chicago)

# ------------------------------ Main ----------------------------------

if __name__ == '__main__':
    global nodes, links, edge_set
    nodes, links = load_data()
    create_adjacency_lists(nodes, links)
    edge_set = create_edge_set(links)
    test_graph_size()

    define_locations()
    #test_a()
    #test_b()
    #test_d()
    test_f()
