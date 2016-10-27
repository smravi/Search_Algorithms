
# --------------------------------Class Definition----------------------------------------------------------
class Graph:
    def __init__(self, searchType, isDirected, hops, graphDict):
        self.searchType = searchType
        self.isDirected = isDirected
        self.nodeCount = hops
        self.graphDict = graphDict

    def getGraph(self):
        return self.graphDict

    def getNodeCount(self):
        return self.nodeCount
