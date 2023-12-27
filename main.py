from classDefinitions import *
import time


def loadData(file):
    with open(file, 'r') as f:
        data = [int(i) for i in f.read().split()]
        n = data.pop(0)
        G = Graph([Vertex(i) for i in range(1, n + 1)], [], n)
        for i in range(0, len(data), 3):
            G.edges.append(Edge(G.vertices[data[i] - 1], G.vertices[data[i + 1] - 1], data[i + 2]))
    return n, G


def outputHoneyData(file, G):
    lines = [str(len(G.honeyEdges)) + "\t" + str(G.honeySum)]
    for i, edge in enumerate(G.honeyEdges):
        if i % 5 == 0:
            lines.append("\n" + str(edge.u.idx) + "\t" + str(edge.v.idx))
        else:
            lines.append("\t\t" + str(edge.u.idx) + "\t" + str(edge.v.idx))
    with open(file, 'w') as f:
        f.writelines(lines)


def main(infile, outfile):
    start_time = time.time()
    n, G = loadData(infile)
    G.preprocess()
    G.maximumSpanningTree()
    G.getHoneyFromMST()
    # G.print()
    outputHoneyData(outfile, G)
    print(f"Time for file {infile}:", time.time() - start_time)


def run3tests(outfiles):
    for fileset in outfiles:
        main(fileset[0], fileset[1])


if __name__ == '__main__':
    print("Testing Sample Graphs")
    testFiles = [
        ['Tests/test1.txt', 'Tests/outfile1.txt'],
        ['Tests/test2.txt', 'Tests/outfile2.txt'],
        ['Tests/test3.txt', 'Tests/outfile3.txt']
    ]
    run3tests(testFiles)

    print("Testing self-made graph")
    inf = 'Tests/test0.txt'
    outf = 'Tests/outfile.txt'
    main(inf, outf)
