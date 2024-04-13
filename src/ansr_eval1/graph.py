# 1. Read a graph (vertices, edges) and keep out zones.
# 2. Calculate all possible acylic paths in the graph for each pair of vertices and sort them by their manhattan distance.
# 3. Repeat (2) but while avoiding keep out zones.

from mission import Mission

import sys
sys.path.append('..')
from Controllers.minigrid_controller import MiniGridController

from collections import namedtuple

Point = namedtuple("Point", "x y")

def manhattan_distance(p1: Point, p2: Point):
    return abs(p1.x - p2.x) + abs(p1.y - p2.y)

# class Point:
#     def __init__(self, x=0, y=0):
#         self._xy = (x, y)

#     def x(self):
#         return self._xy[0]

#     def y(self):
#         return self._xy[1]

#     def print(self):
#         print(f"({self.x()}, {self.y()})")
    
#     def manhattan_distance(self, other):
#         return abs(self.x() - other.x()) + abs(self.y() - other.y())


class Node:
    def __init__(self, id="", point=None):
        if point is None:
            point = Point(x=0, y=0)
        self._id = id
        self._point = point

    def __init__(self, infile):
        self._id, x, y = infile.readline().split()
        self._point = Point(int(x), int(y))

    def get_id(self):
        return self._id

    def get_point(self):
        return self._point

    def print(self):
        print(f"{self.get_id()}: ", end="")
        print(self.get_point())

    def manhattan_distance(self, other):
        return manhattan_distance(self.get_point(), other.get_point())


class Edge:
    def __init__(self, n1: Node, n2: Node):
        self.n1, self.n2 = n1, n2
        self.mh_dist = self.n1.manhattan_distance(self.n2)
    
    @classmethod
    def from_file(cls, infile, nodes):
        id1, id2 = infile.readline().split()
        return cls(nodes[id1], nodes[id2])

    def get_node1(self):
        return self.n1

    def get_node2(self):
        return self.n2

    def print(self):
        print(f"({self.n1.get_id()},{self.n2.get_id()}), len: {self.manhattan_distance()}")

    def manhattan_distance(self):
        return self.mh_dist
    
    def reverse(self):
        return Edge(self.n2, self.n1)


class Koz:
    # Keep out zone;
    # TODO: assuming convex polygon with edges listed cpointkwise
    def __init__(self, infile, n_vertices):
        self.n_vertices = n_vertices
        self.vertices = []
        for i in range(n_vertices):
            x, y = map(int, infile.readline().split())
            self.vertices.append(Point(x, y))

    def get_vertices(self):
        return self.vertices

    def intersects(self, e, p):
        # TODO:
        pass

    def get_intersection(self, edges):
        for i in range(len(edges)):
            # if (
            pass

    def print(self):
        print("koz:")
        print(f"\t# vertices: {self.n_vertices}")
        for vertex in self.vertices:
            print("\t", end="")
            vertex.print()


class Path:
    def __init__(self):
        self.nodes = ""
        self.mh = 0
    
    def __init__(self, path, mh):
        self.nodes = path
        self.mh = mh

    def print(self):
        print(f"\t{self.nodes}, mh: {self.mh}")  


class Graph:
    def __init__(self):
        self.n_nodes = 0
        self.n_edges = 0
        self.n_koz_vertices = 0
        self.nodes = []
        self.edges = []
        self.id_to_node = {}
        self.coord_to_node = {}
        self.coord_to_edge = {}
        self.id_to_edges = {}
        self.id_to_paths = {}
        self.keep_out_edges = [] # T/F for each edge; T=> keep-out edge
    
    def clear(self):
        self.n_nodes = 0
        self.n_edges = 0
        self.n_koz_vertices = 0
        self.nodes.clear()
        self.edges.clear()
        self.coord_to_node.clear()
        self.coord_to_edge.clear()
        self.id_to_node.clear()
        self.id_to_edges.clear()
        self.id_to_paths.clear()
        self.keep_out_edges.clear()

    def read_node(self, infile):
        return Node(infile)

    def read_edge(self, infile):
        return Edge.from_file(infile, self.id_to_node)

    def read_koz(self, infile, n_vertices):
        return Koz(infile, n_vertices)

    def get_node(self, node: str):
        return self.id_to_node[node]
    
    def get_node_from_point(self, p : Point):
        return self.coord_to_node[p]
        
    def is_keep_out_edge(self, index: int) -> bool:
        return self.keep_out_edges[index]

    def manhattan_distance(self, n1: str, n2: str):
        n1_node = self.id_to_node[n1]
        n2_node = self.id_to_node[n2]
        return n1_node.manhattan_distance(n2_node)
    
    def get_neighboring_nodes(self, node: str):
        return [n.get_node2().get_id() for n in self.id_to_edges[node]]
    
    # TODO: Integrate and test.
    def set_data(self, mission: Mission, controllers: list[MiniGridController]):
        """
        Read graph from Mission object and list of controllers
        and populate nodes, edges and keep_out_zone.
        There are two controllers per cell:
        Cell i corresponds to Controller 2i and 2i+1
        """
        # Mission defines cells (i.e. edges)
        # Controllers are associated with initial and final states
        
        # Get vertices either from initial states
        initial_states = set()
        final_states = set()
        for controller in controllers:
            init_state = controller.get_init_states()
            fin_state = controller.get_final_states()
            assert(len(init_state) == 1)
            assert(len(fin_state) == 1)
            initial_states.add(init_state)
            final_states.add(fin_state)
            #TODO: check data types

        # Populate nodes
        self.n_nodes = len(initial_states)
        for i, pt in enumerate(initial_states):
            n = Node(str(i), pt) #TODO: check type
            self.nodes.append(n)
            self.id_to_node[n.get_id()] = n
            self.coord_to_node[n.get_point().x, n.get_point().y] = n

        for key, value in self.id_to_node.items():
            value.print()                    
        for key, value in self.coord_to_node.items():
            print(key, end=": ")
            value.print()
        
        # Populate edges
        self.n_edges = len(controllers)
        print("\n\n# edges:", self.n_edges)
        for i, controller in enumerate(controllers):
            init_state = controller.get_init_states()
            fin_state = controller.get_final_states()
            assert(len(init_state) == 1)
            assert(len(fin_state) == 1)
            w, x = init_state[0].x, init_state[0].y
            y, z = fin_state[0].x, fin_state[0].y
            n1 = self.coord_to_node[w, x]
            n2 = self.coord_to_node[y, z]
            e = Edge(n1, n2)
            # No need to generate reverse edge since its a separate controller
            self.edges.append(e)
            self.id_to_edges.setdefault(n1.get_id(), []).append(e)
            self.coord_to_edge[w, x, y, z] = (e, i)
        
        for i in range(self.n_edges):
            self.edges[i].print()
        for k, v in self.coord_to_edge.items():
            print(f"{v[1]}: {k}, ", end="")
            v[0].print()

        # Determine keep out zones cells (i.e. edges)
        self.keep_out_edges = []
        assert(len(mission.cells) == (len(self.n_edges)/2))
        for c in mission.cells:
            self.keep_out_edges.append(c.in_keep_out_zone)
            self.keep_out_edges.append(c.in_keep_out_zone)

    def initialize_from_file(self, file_name):
        """
        Read graph from a file and populate nodes, edges, and keep_out_zone.
        
        Args:
        file_name (str): Name of the file containing graph information.
        
        Returns:
        int: 0 if successful, 1 if unable to open file.
        """
        with open(file_name, 'r') as infile:
            self.clear()
            
            if infile:
                line = infile.readline() 
                while line == '':
                    line = infile.readline()        
                self.n_nodes = int(line.split()[1])
                print("# nodes:", self.n_nodes)
                
                for i in range(self.n_nodes):
                    n = self.read_node(infile)
                    self.nodes.append(n)
                    self.id_to_node[n.get_id()] = n
                    self.coord_to_node[n.get_point().x, n.get_point().y] = n
                
                for key, value in self.id_to_node.items():
                    value.print()                    

                for key, value in self.coord_to_node.items():
                    print(key, end=": ")
                    value.print()
                
                line = infile.readline()
                while line == '':
                    line = infile.readline()         
                self.n_edges = int(line.split()[1])
                print("\n\n# edges:", self.n_edges)
                
                for i in range(self.n_edges):
                    e = self.read_edge(infile)
                    e_rev = e.reverse()
                    self.edges.append(e)
                    self.id_to_edges.setdefault(e.get_node1().get_id(), []).append(e)
                    self.id_to_edges.setdefault(e.get_node2().get_id(), []).append(e_rev)
                    w, x = e.get_node1().get_point()
                    y, z = e.get_node2().get_point()
                    self.coord_to_edge[w, x, y, z] = (e, 2*i)
                    self.coord_to_edge[y, z, w, x] = (e_rev, 2*i+1)
                
                for i in range(self.n_edges):
                    self.edges[i].print()
                
                for k, v in self.coord_to_edge.items():
                    print(f"{v[1]}: {k}, ", end="")
                    v[0].print()
                
                self.keep_out_edges = [False] * (self.n_edges*2)

                # print("\n\nOutgoing edges:")
                # for k, v in self.id_to_edges.items():
                #     print(f"\tNode: {k}\n\tEdges: ")
                #     for e in v:
                #         print("\t\t", end='')
                #         e.print()
                #     print()
                
                return 0
            else:
                print("Unable to open file")
                return 1
            

    def find_all_paths(self):
        """
        Find all paths between nodes and print them sorted by Manhattan distance.
        """
        print("\n\nPaths:")
        self.id_to_paths = {}
        
        for i in range(len(self.nodes)):
            for j in range(i+1, len(self.nodes)):
                if i != j:
                    src = self.nodes[i]
                    dst = self.nodes[j]
                    paths = self.find_paths(src.get_id(), dst.get_id())
                    self.id_to_paths[src.get_id() + " " + dst.get_id()] = paths
                    reverse_paths = []
                    for path, mh in paths:
                        reverse_path = list(reversed(path))
                        reverse_paths.append((reverse_path, mh))
                    self.id_to_paths[dst.get_id() + " " + src.get_id()] = reverse_paths
        
        return self.id_to_paths

    def find_paths_to_cell(self, src, dst1, dst2, exclude=[]):
        """
        Find all paths between source and destination nodes
        such that the path ends in dst1, dst2 or dst2, dst1.
        
        Args:
        src (str): Source node.
        dst1 (str): Destination node 1.
        dst2 (str): Destination node 2.
        exclude ([(str1, str2)]): Cells to exclude from paths.
        """

        paths = self.find_paths(src, dst1)
        # print(f"Paths from {src} to {dst1}: {len(paths)}")
        paths += self.find_paths(src, dst2)
        # print(f"Paths from {src} to {dst1, dst2}: {len(paths)}")
        
        pruned_paths = [(path, mh) for path, mh in paths
                        if (len(path) >= 2 and
                            ((path[-1]==dst1 and path[-2]==dst2)
                             or (path[-1]==dst2 and path[-2]==dst1)))]        
        pruned_paths.sort(key=lambda x: x[1])
        return pruned_paths
    
    def convert_to_edges(self, paths):
        """Paths: [(['6', '7', '3', '2'], 34), ...]"""
        edge_paths = []
        for p in paths:
            edge_path = []
            for i in range(len(p[0])-1):
                (w, x) = self.id_to_node[p[0][i]].get_point()
                (y, z) = self.id_to_node[p[0][i+1]].get_point()
                edge_path.append((w, x, y, z))
            edge_paths.append(edge_path)
        return edge_paths

    def is_valid_controller_path(self, controller_path):
        """ True if none of the controllers belong to a cell in a keep out zone """
        for c in controller_path:
            if self.is_keep_out_edge(c):
                return False 
        return True

    def convert_to_controllers(self, paths):
        """Paths: [(['6', '7', '3', '2'], 34), ...]"""
        controller_paths = []
        for p in paths:
            controller_path = []
            for i in range(len(p[0])-1):
                (w, x) = self.id_to_node[p[0][i]].get_point()
                (y, z) = self.id_to_node[p[0][i+1]].get_point()
                (e, i) = self.coord_to_edge[w, x, y, z]
                controller_path.append(i)
            if self.is_valid_controller_path(controller_path):
                controller_paths.append(controller_path)
        return controller_paths

    def find_paths(self, src, dst):
        """
        Find all paths between source and destination nodes.
        
        Args:
        src (str): Source node.
        dst (str): Destination node.
        paths (set): Set to store paths.
        """
        paths = []
        path_from_src = [src]
        self.find_paths_dfs(path_from_src, dst, paths)
        sorted_paths = []
        for path in paths:
            nodes_list = path
            mh = len(nodes_list) # use length of path as tie-breaker in mh sort
            for i in range(len(nodes_list) - 1):
                mh += self.manhattan_distance(nodes_list[i], nodes_list[i+1])
            sorted_paths.append((path, mh))
        sorted_paths.sort(key=lambda x: x[1])
        return sorted_paths

    def find_paths_dfs(self, path_from_src, dst, paths):
        """
        Depth-first search to find paths between nodes.
        
        Args:
        path_from_src ([str]): Path from source node.
        dst (str): Destination node ID.
        paths (set): Set to store paths.
        """
        src = path_from_src[-1]

        if src == dst:
            paths.append(path_from_src.copy())
            return

        outgoing = self.id_to_edges.get(src, [])
        for out_edge in outgoing:
            out = out_edge.get_node2().get_id()
            if out not in path_from_src: # avoid loops in paths
                path_from_src.append(out)
                self.find_paths_dfs(path_from_src, dst, paths)
                path_from_src.pop()

    def locate_nearest_node(self, p: Point):
        """
        Locate the nearest node to a given point based on Manhattan distance.
        
        Args:
        p (point): Point to find the nearest node to.
        
        Returns:
        node: Nearest node to the given point.
        """
        assert(len(self.nodes) > 0)

        nearest_node = 0
        min_mh = manhattan_distance(p, self.nodes[nearest_node].get_point())
        
        # print("0: \tPoint:", end=" ")
        # p.print()
        # print("\t Current Node:", end=" ")
        # self.nodes[nearest_node].print()
        # print("\t Nearest Node:", end=" ")
        # self.nodes[nearest_node].print()
        # print()
        
        for i in range(1, self.n_nodes):
            mh = manhattan_distance(p, self.nodes[i].get_point())
            if mh < min_mh:
                nearest_node = i
                min_mh = mh
            
            # print(i, ": \tPoint:", end=" ")
            # p.print()
            # print("\t Current Node:", end=" ")
            # self.nodes[i].print()
            # print("\t Nearest Node:", end=" ")
            # self.nodes[nearest_node].print()
            # print()
        
        return self.nodes[nearest_node]



if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] == "-f" and len(sys.argv) > 2:
            print(f"Reading graph file: {sys.argv[2]}")
            a_graph = Graph()
            a_graph.initialize_from_file(sys.argv[2])

            # Test: find all paths in the graph
            print("\n\nTest: find all paths in the graph")
            id_to_paths = a_graph.find_all_paths()
            for node_id, paths in id_to_paths.items():
                print(node_id + ":", len(paths))
                for path, mh in paths:
                    print("\t", mh, ":", path)
        
            # Test: find all paths from one vertex to another
            print("\n\nTest: find all paths from one vertex to another")
            sorted_paths = a_graph.find_paths('0', '1')
            print(f"\nPaths from 0 to 1: {len(sorted_paths)}")
            for path, mh in sorted_paths:
                print("\t", mh, ":", path)
            sorted_paths = a_graph.find_paths('0', '2')
            print(f"\nPaths from 0 to 2: {len(sorted_paths)}")
            for path, mh in sorted_paths:
                print("\t", mh, ":", path)

            # Test: find all paths from a vertex to a cell
            print("\n\nTest: find all paths from a vertex to a cell")
            sorted_paths = a_graph.find_paths_to_cell('0', '1', '2')
            print(f"\nPaths from 0 ending with 1,2 or 2,1: {len(sorted_paths)}")
            for path, mh in sorted_paths:
                print("\t", mh, ":", path)

            # Test: find the nearest vertex in the graph to the input coordinates
            # print("Enter point x y to check for nearest intersection: ", end="")
            # x, y = map(int, input().split())
            print("\n\nTest: find the nearest vertex in the graph to the input coordinates")
            x, y = 10, 10
            p = Point(x, y)
            nearest_node = a_graph.locate_nearest_node(p)
            print(f"Nearest node to ({x},{y}) is ", end="")
            nearest_node.print()
            print()

            # Test: find paths between two arbitrary points after finding nodes closest to them
            print("\n\nTest: find paths between two arbitrary points after finding nodes closest to them")
            p1 = Point(10, 10)
            n1 = a_graph.locate_nearest_node(p1)
            p2 = Point(20, 20)
            n2 = a_graph.locate_nearest_node(p2)
            n1.print()
            n2.print()

            paths = a_graph.find_paths(n1.get_id(), n2.get_id())
            for p in paths:
                print(p)
            
            # Test: find neighbors of an arbitrary node
            print("\n\nTest: find neighbors of an arbitrary node")
            n1.print()
            neighbors = a_graph.get_neighboring_nodes(n1.get_id())
            print(neighbors)

            # Test: find all paths between a pair of neighboring nodes
            print("\n\nTest: find all paths between a pair of neighboring nodes")
            n1.print()
            n2 = a_graph.get_node(neighbors[0])
            n2.print()
            paths = a_graph.find_paths(n1.get_id(), n2.get_id())
            print(paths)
            for p in paths:
                print(p)

            # Test: Convert paths of nodes to a path of controllers
            print("\n\nTest: Convert paths of nodes to a path of controllers")
            for p in a_graph.convert_to_controllers(paths):
                print(p)
            
            print("\n\nTest: Adding controllers 16 and 17 to the list of keep out edges")
            a_graph.keep_out_edges[16]=True
            a_graph.keep_out_edges[17]=True
                
            print("\n\nTest: Paths after pruning paths with keep out edges")
            for p in a_graph.convert_to_controllers(paths):
                print(p)
            
            # for p in a_graph.convert_to_edges(paths):
            #     print(p)
