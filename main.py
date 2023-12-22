from classDefinitions import *


def loadData(file):
    with open(file, 'r') as f:
        data = [int(i) for i in f.read().split()]
        n = data.pop(0)
        G = Graph([Vertex(i) for i in range(1, n + 1)], [], n)
        for i in range(0, len(data), 3):
            G.edges.append(Edge(G.vertices[data[i] - 1], G.vertices[data[i + 1] - 1], data[i + 2]))
    return n, G


def main(file):
    start_time = time.time()
    n, G = loadData(file)
    G.preprocess()
    G.maximumSpanningTree()
    G.getHoneyFromMST()
    G.print()
    end_time = time.time()
    print("Total time:", end_time - start_time)


if __name__ == '__main__':
    filename = 'Tests/test2.txt'
    main(filename)
