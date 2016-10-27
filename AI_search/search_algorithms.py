import AI_search.helpers.helper.acostFunction as acostFunction
import AI_search.helpers.helper.ucsCostFunction as ucsCostFunction
import AI_search.helpers.helper.writeOutput as writeOutput
import AI_search.search.bfs.bfs_search as bfs_search
import AI_search.search.bfs.bfs_search as dfs_search
import AI_search.search.helpers.helper.Edge as Edge
import AI_search.helpers.Graph as Graph
import AI_search.helpers.Node as Node
import AI_search.search.informed_search as informed_search

# graphDict
# [
# Node1: [(Node, weight of the edge (Node1->Node), order of the edge)]
# Node2: [(Node, weight of the edge (Node2->Node), order of the edge)]
# ]

# childlist:[(Node, weight of the edge (Node1->Node), order of the edge)]

# priority in the edge is required for the maintaining the input order in case of ties on same path cost

# minPathToNode dictionary 
#  A:[A] 
#  B:[A,B] 
#  C:[A,C] 
#  D:[A,B,D]
#  E:[A,B,E]
#  I:[A,B,D,I]
#  F:[A,B,D,F]



# --------------------------------Main Function----------------------------------------------------------
# gets the input from the file and normalizes the input
def main():
    inputSpec = []
    graphDict = dict()
    nameToNodeMap = dict()
    with open('input.txt', 'r') as file:
        for line in file:
            inputSpec.append(line.strip())
    if len(inputSpec) > 0:
        searchType = inputSpec[0]
        startNode = inputSpec[1].strip()
        goalNode = inputSpec[2].strip()
        hops = int(inputSpec[3].strip())
        index = 4  # this is the start index where the list of routes are specified
        order = 1  # this order is used to break the tie for BFS if multiple paths of same length exists
        hopList = []  # edge with pathcost list

        for hopIterator in range(4, hops + index, 1):
            hopList.append(inputSpec[hopIterator].strip())
        trafficIterator = hopIterator
        # update the childlist and edge and the pathcost
        for hop in hopList:
            parent, child, cost = hop.split(' ')
            parent = parent.strip()
            child = child.strip()
            cost = cost.strip()
            if searchType == 'BFS' or searchType == 'DFS':
                cost = 1
            else:
                cost = int(cost)
            if not parent in nameToNodeMap:
                nodeObj = Node(parent)
                nameToNodeMap[parent] = nodeObj
                graphDict[nodeObj] = []
            if not child in nameToNodeMap:
                nodeObj = Node(child)
                nameToNodeMap[child] = nodeObj
                graphDict[nodeObj] = []
            # create an edge for this parent and child
            edge = Edge(nameToNodeMap[child], cost, order)
            graphDict[nameToNodeMap[parent]].append(edge)
            order += 1

        # sunday traffic
        sundayTrafficIndex = trafficIterator + 1
        sundayTrafficLines = int(inputSpec[sundayTrafficIndex])
        for heuristic in range(sundayTrafficIndex + 1, sundayTrafficIndex + sundayTrafficLines + 1, 1):
            # the sunday traffic gives us detail about the number of nodes
            nodeName, traffic = inputSpec[heuristic].split(' ')
            nodeName = nodeName.strip()
            traffic = traffic.strip()
            if nodeName in nameToNodeMap:
                nameToNodeMap[nodeName].sundayTraffic = int(traffic)

        graphObj = Graph(searchType, True, sundayTrafficLines, graphDict)

        if searchType == 'BFS':
            totalPathCost = bfs_search(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode])

        if searchType == 'UCS':
            #  def informedSearch(graphObj, startNodeObj, goalNodeObj, startNode_gcost, startNode_hcost, costFunction):
            totalPathCost = informed_search(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode], 0, 0,
                                            ucsCostFunction)
        if searchType == 'A*':
            totalPathCost = informed_search(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode],
                                            0, nameToNodeMap[startNode].sundayTraffic, acostFunction)
        if searchType == 'DFS':
            totalPathCost = dfs_search(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode])

        writeOutput('output.txt', totalPathCost)


if __name__ == '__main__':
    main()
