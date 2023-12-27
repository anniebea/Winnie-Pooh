class Vertex:
    idx: int
    visited: bool
    weight: int  # stores the max value needed to connect to self

    def __init__(self, idx):
        self.idx = idx
        self.visited = False
        self.weight = -20000

    def print(self):
        print(self.idx)


class Edge:
    u: Vertex  # smaller order number
    v: Vertex  # larger order number
    weight: int

    def __init__(self, u, v, weight):
        if u.idx < v.idx:
            self.u = u
            self.v = v
        else:
            self.u = v
            self.v = u
        self.weight = weight

    def print(self):
        print(f"{self.u.idx}-{self.v.idx} ; w={self.weight}")


class Graph:
    vertexCount: int
    vertices: [Vertex]
    edges: [Edge]
    adjacencyMatrix: [[Vertex]]
    honeyEdges: [Edge]
    honeySum = int
    maxSpanTree: [(int, int, int)]  # maximum spanning tree

    def __init__(self, vertices, edges, vCount):
        self.vertexCount = vCount
        self.vertices = [v for v in vertices]
        self.edges = [e for e in edges]
        self.honeyEdges = []
        self.honeySum = 0
        self.maxSpanTree = []

    def preprocess(self):
        """
        Prepare graph for further actions.
        Time complexity: O(vve)
        (O(e) + O(e) + O(v)*O(ve) + O(e))
        """
        self.honeyPotNonPositiveEdges()
        self.edges = [edge for edge in self.edges if edge not in self.honeyEdges]
        for _ in self.vertices:
            finished = self.removeBoringVertices()
            if finished is True:
                break
        self.getAdjacencyMatrix()

    def removeBoringVertices(self):
        """
        Remove all vertices and their associated edges from graph if not part of cyclic paths.
        Time complexity: O(ve)

        (O(v)*O(e) + O(v) + O(e) + O(e))
        :return: whether no changes were made
        """
        deletable = []
        for vertex in self.vertices:
            neighbors = self.findNeighbors(vertex)
            if len(neighbors) <= 2:  # the vertex itself is the first neighbor
                deletable.append(vertex)
        self.vertices = [vertex for vertex in self.vertices if vertex not in deletable]

        remEdges = []
        for edge in self.edges:
            if edge.u in deletable or edge.v in deletable:
                remEdges.append(edge)
        self.edges = [edge for edge in self.edges if edge not in remEdges]

        if deletable == [] and remEdges == []:
            return True
        else:
            return False

    def findNeighbors(self, vertex):
        """
        Finds all neighbors of one vertex.
        NB: first elem in list is vertex itself
        Time complexity: O(e)

        :param vertex: vertex for which neighbors should be found
        :return: list of all neighbors of vertex v
        """
        neighbors = [vertex]
        for edge in self.edges:
            if edge.u == vertex:
                neighbors.append(edge.v)
            elif edge.v == vertex:
                neighbors.append(edge.u)
        return neighbors

    def getAdjacencyMatrix(self):
        """
        Creates adjacency matrix.
        Time complexity: O(e)
        """
        self.adjacencyMatrix = [[None for _ in range(self.vertexCount + 1)] for _ in range(self.vertexCount + 1)]
        for edge in self.edges:
            self.adjacencyMatrix[edge.u.idx][edge.v.idx] = edge.weight
            self.adjacencyMatrix[edge.v.idx][edge.u.idx] = edge.weight

    def honeyPotNonPositiveEdges(self):
        """
        Automatically adds all edges with non-positive weight to the HoneyPot edge collection.
        Time complexity: O(e)
        """
        for edge in self.edges:
            if edge.weight is not None and edge.weight <= 0:
                self.honeyEdges.append(edge)

    def maximumSpanningTree(self):
        """
        Computes the maximum spanning tree
        Time complexity: O(v^3)
        """
        # include first vertex
        self.vertices[0].weight = None
        self.vertices[0].visited = True

        for i in range(1, len(self.vertices)):
            # find max-weight unvisited neighbor of a visited vertex
            maxSourceIdx = 0
            maxNeighbor = Vertex(0)
            maxNeighbor.weight = -20001
            for source in self.vertices:
                if source.visited is True:
                    for vertex in self.vertices:
                        if vertex.visited is False:
                            if self.adjacencyMatrix[source.idx][vertex.idx] is not None:  # there is an edge
                                if self.adjacencyMatrix[source.idx][vertex.idx] > maxNeighbor.weight:
                                    maxNeighbor = vertex
                                    maxSourceIdx = source.idx
                                    maxNeighbor.weight = self.adjacencyMatrix[source.idx][vertex.idx]
            if maxNeighbor.weight != -20001:
                maxNeighbor.visited = True
                maxNeighbor.weight = self.adjacencyMatrix[maxSourceIdx][maxNeighbor.idx]
                if maxSourceIdx < maxNeighbor.idx:
                    self.maxSpanTree.append((maxSourceIdx, maxNeighbor.idx, maxNeighbor.weight))
                else:
                    self.maxSpanTree.append((maxNeighbor.idx, maxSourceIdx, maxNeighbor.weight))

    def getHoneyFromMST(self):
        """
        Calculates the Honeypot edge total difficulty.
        Time complexity: O(e)
        """
        non_mst = [edge for edge in self.edges
                   if (edge.u.idx, edge.v.idx, edge.weight) not in self.maxSpanTree
                   and (edge.v.idx, edge.u.idx, edge.weight) not in self.maxSpanTree]
        self.honeyEdges.extend([edge for edge in non_mst if edge not in self.honeyEdges])
        for edge in self.honeyEdges:
            self.honeySum += edge.weight

    def print(self):
        """
        Print graph data in easily readable format.

        Time complexity: irrelevant - this function is not necessary for the algorithm,
        it is only called for debugging purposes.
        """
        print("-----------")
        print("GRAPH DATA")

        # print("Vertices:")
        # print(', '.join(map(str, [vertex.idx for vertex in self.vertices])))

        # print("\nEdges:")
        # for edge in self.edges:
        #     edge.print()

        print("\nAdjacency matrix:")
        self.adjacencyMatrix.pop(0)
        npk = [str(i) for i in range(1, len(self.adjacencyMatrix) + 1)]
        conv = lambda i: i or '-'
        for i in range(len(self.adjacencyMatrix)):
            self.adjacencyMatrix[i].pop(0)
            self.adjacencyMatrix[i].insert(0, i + 1)
            self.adjacencyMatrix[i] = [conv(i) for i in self.adjacencyMatrix[i]]
        s = [[str(e) for e in row] for row in self.adjacencyMatrix]
        npk.insert(0, "u\\v")
        s.insert(0, npk)
        lens = [max(map(len, col)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        print('\n'.join(table))

        print("\nMaxSpanTree")
        for entry in self.maxSpanTree:
            print(f"{entry[0]}-{entry[1]} ; w={entry[2]}")

        print("\nHoney Edges:")
        for edge in self.honeyEdges:
            edge.print()

        print("\nHoney Difficulty:", self.honeySum)

        print("-----------")
