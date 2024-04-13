# 1. Read a graph (vertices, edges) and keep out zones.
# 2. Calculate all possible acylic paths in the graph for each pair of vertices and sort them by their manhattan distance.
# 3. Repeat (2) but while avoiding keep out zones.

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def print(self):
        print(f"({self.x}, {self.y})")

    def manhattan_distance(self, other):
        return abs(self.x - other.x) + abs(self.y - other.y)


class Node:
    def __init__(self, id="", loc=None):
        if loc is None:
            loc = Point(0, 0)
        self.id = id
        self.loc = loc

    def __init__(self, infile):
        self.id, x, y = infile.readline().split()
        self.loc = Point(int(x), int(y))

    def get_id(self):
        return self.id

    def get_point(self):
        return self.loc

    def print(self):
        print(f"{self.id}: ", end="")
        self.loc.print()

    def manhattan_distance(self, other):
        return self.loc.manhattan_distance(other.loc)



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
    # TODO: assuming convex polygon with edges listed clockwise
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
        self.id_to_edges = {}
        self.id_to_paths = {} 
        # self.keep_out_zone = Koz()
    
    def clear(self):
        self.n_nodes = 0
        self.n_edges = 0
        self.n_koz_vertices = 0
        self.nodes.clear()
        self.edges.clear()
        self.id_to_node.clear()
        self.id_to_edges.clear()
        self.id_to_paths.clear()
        # self.keep_out_zone = Koz()

    def read_node(self, infile):
        return Node(infile)

    def read_edge(self, infile):
        return Edge.from_file(infile, self.id_to_node)

    def read_koz(self, infile, n_vertices):
        return Koz(infile, n_vertices)

    def get_node(self, node: str):
        return self.id_to_node[node]

    def manhattan_distance(self, n1: str, n2: str):
        n1_node = self.id_to_node[n1]
        n2_node = self.id_to_node[n2]
        return n1_node.manhattan_distance(n2_node)
        
    def read_graph(self, file_name):
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
                
                for key, value in self.id_to_node.items():
                    value.print()                    
                
                line = infile.readline()
                while line == '':
                    line = infile.readline()         
                self.n_edges = int(line.split()[1])
                print("\n\n# edges:", self.n_edges)
                
                for i in range(self.n_edges):
                    e = self.read_edge(infile)
                    self.edges.append(e)
                    self.id_to_edges.setdefault(e.get_node1().get_id(), []).append(e)
                    self.id_to_edges.setdefault(e.get_node2().get_id(), []).append(e.reverse())
                
                for i in range(self.n_edges):
                    self.edges[i].print()
                
                # print("\n\nOutgoing edges:")
                # for k, v in self.id_to_edges.items():
                #     print(f"\tNode: {k}\n\tEdges: ")
                #     for e in v:
                #         print("\t\t", end='')
                #         e.print()
                #     print()

                line = ""
                while line == "":
                    line = infile.readline()     
                self.n_koz_vertices = int(line.split()[1])
                print("\n")
                self.keep_out_zone = self.read_koz(infile, self.n_koz_vertices)
                self.keep_out_zone.print()
                
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
                    paths = set()
                    self.find_paths(src, dst, paths)
                    self.id_to_paths[src.get_id() + " " + dst.get_id()] = paths
                    reverse_paths = set()
                    for path in paths:
                        reverse_paths.add(' '.join(reversed(path.split())))
                    self.id_to_paths[dst.get_id() + " " + src.get_id()] = reverse_paths
        
        for node_id, paths in self.id_to_paths.items():
            sorted_paths = []
            print(node_id + ":", len(paths))
            for path in paths:
                nodes_list = path.split()
                mh = len(nodes_list) # use length of path as tie-breaker in mh sort
                for i in range(len(nodes_list) - 1):
                    mh += self.manhattan_distance(nodes_list[i], nodes_list[i+1])
                sorted_paths.append((path, mh))
            sorted_paths.sort(key=lambda x: x[1])
            for path, mh in sorted_paths:
                print("\t", mh, ":", path)
        
        return 0

    def find_paths(self, src, dst, paths):
        """
        Find all paths between source and destination nodes.
        
        Args:
        src (node): Source node.
        dst (node): Destination node.
        paths (set): Set to store paths.
        """
        path_from_src = [src.get_id()]
        self.find_paths_dfs(path_from_src, dst.get_id(), paths)

    def find_paths_dfs(self, path_from_src, dst, paths):
        """
        Depth-first search to find paths between nodes.
        
        Args:
        path_from_src ([str]): Path from source node.
        dst (str): Destination node ID.
        paths (set): Set to store paths.
        """
        src = path_from_src[-1]

        # print(f"{path_from_src=}, {src=}, {dst=}")
        
        if src == dst:
            # print(f"Reached {dst}")
            paths.add(' '.join(path_from_src))
            return

        outgoing = self.id_to_edges.get(src, [])
        for out_edge in outgoing:
            out = out_edge.get_node2().get_id()
            # print(f"{out=}, ")
            # print(' '.join(path_from_src))
            if not Graph.node_in_path(out, path_from_src): # avoid loops in paths
                path_from_src.append(out)
                self.find_paths_dfs(path_from_src, dst, paths)
                path_from_src.pop()

    @staticmethod
    def node_in_path(node : str, path : list[str]):
        """Search for string in a list of strings"""
        for s in path:
            if node == s:
                return True
        return False

    def locate_nearest_node(self, p: Point):
        """
        Locate the nearest node to a given point based on Manhattan distance.
        
        Args:
        p (point): Point to find the nearest node to.
        
        Returns:
        node: Nearest node to the given point.
        """
        nearest_node = 0
        min_mh = p.manhattan_distance(self.nodes[nearest_node].get_point())
        
        print("0: \tPoint:", end=" ")
        p.print()
        print("\t Current Node:", end=" ")
        self.nodes[nearest_node].print()
        print("\t Nearest Node:", end=" ")
        self.nodes[nearest_node].print()
        print()
        
        for i in range(1, self.n_nodes):
            mh = p.manhattan_distance(self.nodes[i].get_point())
            if mh < min_mh:
                nearest_node = i
                min_mh = mh
            
            print(i, ": \tPoint:", end=" ")
            p.print()
            print("\t Current Node:", end=" ")
            self.nodes[i].print()
            print("\t Nearest Node:", end=" ")
            self.nodes[nearest_node].print()
            print()
        
        return self.nodes[nearest_node]



if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] == "-f" and len(sys.argv) > 2:
            print(f"Reading graph file: {sys.argv[2]}")
            a_graph = Graph()
            a_graph.read_graph(sys.argv[2])

            # Test find all paths from one vertex to another
            # paths = set()
            # a_graph.find_paths_dfs(["a"], 'b', paths)
            # a_graph.find_paths_dfs(["0"], '1', paths)
            # a_graph.get_node("0").print()
            # a_graph.get_node("1").print()
            # print(f"{len(paths)=}")
            # a_graph.find_all_paths()

            # Test nearest node
            print("Enter point x y to check for nearest intersection: ", end="")
            x, y = map(int, input().split())
            p = Point(x, y)
            nearest_node = a_graph.locate_nearest_node(p)
            print("Nearest node is ", end="")
            nearest_node.print()
            print()

