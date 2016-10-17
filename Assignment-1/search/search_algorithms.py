import sys
import queue
from collections import namedtuple
import heapq
from collections import deque

algo = ['BFS', 'DFS', 'UCS', 'A*']


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

# create a namesTuple to store the edge details
Edge = namedtuple('Edge', 'child edgecost order')
# create a namedtuple priority
Priority = namedtuple('Priority', 'fcost edgeorder node gcost, parent')
# create the result route
Route = namedtuple('Route', 'location time')


# Edge = namedtuple('Edge', 'child edgecost order')
# -------------------------------- Common Helpers----------------------------------------------------------

def writeOutput(fileName, routePath):
    with open(fileName, 'w') as f:
        for hop in routePath:
            f.write('{} {}\n'.format(hop.location, str(hop.time)))
        f.truncate(f.tell() - 1)


# -------------------------------- InformedSearch Helpers-------------------------------------------------------
def findNodeInQueue(queue, node):
    for priority in queue:
        if priority.node == node:
            return priority
    return None


# childPriority - Priority(pathCost=5, node=Node(D), parent=Node(B)]
# currentParent- Node(C)
# edge-Edge(child=Node(D), edgecost=1, order=3)

def ucsCostFunction(currentParent, edge):
    return currentParent.gcost + edge.edgecost


def acostFunction(currentParent, edge):
    return currentParent.gcost + edge.edgecost + edge.child.sundayTraffic


def fetchgcost(currentParent, edge):
    return currentParent.gcost + edge.edgecost


def createPath(node_tuple, closed, totalPathCost):
    if not node_tuple.parent in closed:
        return totalPathCost[::-1]
    else:
        route = Route(closed[node_tuple.parent].node.name, closed[node_tuple.parent].gcost)
        totalPathCost.append(route)
        return createPath(closed[node_tuple.parent], closed, totalPathCost)


def informedSearch(graphObj, startNodeObj, goalNodeObj, startNode_gcost, startNode_hcost, costFunction):
    nodeDict = graphObj.graphDict
    open_q = []
    # closed dictionary
    closed = dict()
    indexValue = None
    # insert the start node
    # Priority(fcost=5, edgeorder=1 node=Node(D), gcost, hcost, parent=Node(B)]
    # edgeorder  is of zero for the startNode . thats y initalizing to zero below
    order = 1  # For breaking ties, we maintain the order in which the node is inserted in priority queue. Only works for UCS and A*
    prtuple = Priority(startNode_gcost + startNode_hcost, order, startNodeObj, startNode_gcost, None)
    heapq.heappush(open_q, prtuple)

    while open_q:
        node_tuple = heapq.heappop(open_q)
        if node_tuple.node == goalNodeObj:
            totalPathCost = []
            totalPathCost.append(Route(node_tuple.node.name, node_tuple.gcost))
            return createPath(node_tuple, closed, totalPathCost)
        children = nodeDict[node_tuple.node]
        for edge in children:
            childFromOpenQueue = findNodeInQueue(open_q, edge.child)
            # currentParentFromOpenQueue = findNodeInQueue(open_q, node_tuple.node)
            if not childFromOpenQueue and not edge.child in closed:
                # cost calculation from closed
                order += 1
                prtuple = Priority(costFunction(node_tuple, edge), order, edge.child, fetchgcost(node_tuple, edge),
                                   node_tuple.node)
                heapq.heappush(open_q, prtuple)

            elif childFromOpenQueue:
                # the cost is new path is less so replace the child in queue with this new pathcost
                currentCost = costFunction(node_tuple, edge)
                order += 1
                if childFromOpenQueue.fcost > currentCost:
                    newPriority = Priority(currentCost, order, edge.child, fetchgcost(node_tuple, edge),
                                           node_tuple.node)
                    # **** open_q.index(childFromOpenQueue) will gove index.. no need for loop
                    for i in open_q:
                        if newPriority.node == i.node:
                            indexValue = open_q.index(i)
                            break
                    if indexValue != None:
                        open_q[indexValue] = open_q[-1]
                        open_q.pop()
                        heapq.heapify(open_q)
                        heapq.heappush(open_q, newPriority)

            else:
                if edge.child in closed:
                    childFromClosedQueue = closed[edge.child]
                    currentCost = costFunction(node_tuple, edge)
                    if childFromClosedQueue.fcost > currentCost:
                        order += 1
                        newPriority = Priority(currentCost, order, edge.child, fetchgcost(node_tuple, edge),
                                               node_tuple.node)

                        del closed[edge.child]
                        heapq.heappush(open_q, newPriority)
        closed[node_tuple.node] = node_tuple


# --------------------------------DFS Helpers----------------------------------------------------------
def dfsIterator(graphObj, startNodeObj, goalNodeObj):
    dfs_queue = deque()
    closed = []
    nodeDict = graphObj.graphDict
    # closed dictionary
    closed = dict()
    # insert the start node
    # Priority(fcost=0, edgeorder=0 node=Node(A), 0, parent=Node(B)]
    # edgeorder  is of zero for the startNode . thats y initalizing to zero below
    prtuple = Priority(0, 0, startNodeObj, 0, None)
    dfs_queue.appendleft(prtuple)
    while dfs_queue:
        node_tuple = dfs_queue.popleft()
        if node_tuple.node == goalNodeObj:
            totalPathCost = []
            totalPathCost.append(Route(node_tuple.node.name, node_tuple.gcost))
            return createPath(node_tuple, closed, totalPathCost)
        children = nodeDict[node_tuple.node]
        for edge in reversed(children):
            childFromDequeue = findNodeInQueue(dfs_queue, edge.child)
            if not childFromDequeue and not edge.child in closed:
                # cost calculation from closed
                # for dfs gcost = fcost so resuse this method to calculate teh cost
                gcost = fetchgcost(node_tuple, edge)
                prtuple = Priority(gcost, edge.order, edge.child, gcost, node_tuple.node)
                dfs_queue.appendleft(prtuple)
            elif childFromDequeue:
                # the cost is new path is less so replace the child in queue with this new pathcost
                currentCost = fetchgcost(node_tuple, edge)
                if childFromDequeue.fcost > currentCost:
                    newPriority = Priority(currentCost, edge.order, edge.child, currentCost,
                                           node_tuple.node)

                    for i in dfs_queue:
                        if newPriority.node == i.node:
                            dfs_queue.remove(i)
                            break
                    dfs_queue.appendleft(newPriority)
            else:
                if edge.child in closed:
                    childFromClosedQueue = closed[edge.child]
                    currentCost = fetchgcost(node_tuple, edge)
                    if childFromClosedQueue.fcost > currentCost:
                        newPriority = Priority(currentCost, edge.order, edge.child,
                                               currentCost, node_tuple.node)
                        del closed[edge.child]
                        dfs_queue.appendleft(newPriority)
        closed[node_tuple.node] = node_tuple


# --------------------------------BFS Helpers----------------------------------------------------------
# Returns priority from (Node, weight of the edge (Node1->Node), priority of the edge) of the child
def getEdgePriority(parentchildList, child):
    edgeDetails = [tuple for tuple in parentchildList if tuple.child == child]
    return edgeDetails[0].order


# Returns weight cost from (Node, weight of the edge (Node1->Node), priority of the edge) of the child
def getEdgeCost(parentchildList, child):
    edgeDetails = [tuple for tuple in parentchildList if tuple.child == child]
    return edgeDetails[0].edgecost


def decidePath(graphDict, oldPath, newPath):
    differingIndex = [i for i, x in enumerate(zip(oldPath, newPath)) if x[0] != x[1]][0]
    differingEdge1 = oldPath[differingIndex - 1: differingIndex + 1]
    differingEdge2 = newPath[differingIndex - 1: differingIndex + 1]
    if getEdgePriority(graphDict[differingEdge1[0]], differingEdge1[1]) < getEdgePriority(graphDict[differingEdge2[0]],
                                                                                          differingEdge2[1]):
        return oldPath
    return newPath


def getBFSAccumulatedCost(graphDict, minPathToGoal):
    edgeCost = 0
    totalPathCost = []
    initialState = Route(minPathToGoal[0].name, edgeCost)
    totalPathCost.append(initialState)
    for i in range(len(minPathToGoal) - 1):
        edgeCost += getEdgeCost(graphDict[minPathToGoal[i]], minPathToGoal[i + 1])
        route = Route(minPathToGoal[i + 1].name, edgeCost)
        totalPathCost.append(route)
    return totalPathCost


def bfsIterator(graphObj, startNodeObj, goalNodeObj):
    bfsqueue = queue.Queue()
    minPathToNode = dict()
    visited = dict()
    resultMinPath = []

    bfsqueue.put(startNodeObj)
    minPathToNode[startNodeObj] = [startNodeObj]
    while not bfsqueue.empty():
        # deque
        nodeObj = bfsqueue.get()
        visited[nodeObj] = True
        if nodeObj == goalNodeObj:
            resultMinPath = minPathToNode[nodeObj]
            return getBFSAccumulatedCost(graphObj.graphDict, resultMinPath)
        else:
            for edge in graphObj.graphDict[nodeObj]:
                if not edge.child in bfsqueue.queue and not edge.child in visited:
                    bfsqueue.put(edge.child)

                # compare and update path
                newPathList = minPathToNode[nodeObj] + [edge.child]  # append two list
                if not edge.child in minPathToNode or len(minPathToNode[edge.child]) > len(newPathList):
                    minPathToNode[edge.child] = newPathList
                else:
                    if len(minPathToNode[edge.child]) == len(newPathList):
                        minPathToNode[edge.child] = decidePath(graphObj.graphDict, minPathToNode[edge.child],
                                                               newPathList)

    return resultMinPath  # this contains only path now. I have to fit in the cost


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


class Node(object):
    def __init__(self, name):
        self.name = name
        self.sundayTraffic = None

    def getName(self):
        return self.name

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)


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

        # initialize the Graph
        # def informedSearch(graphObj, startNodeObj, goalNodeObj, startNode_gcost, startNode_hcost, costFunction):
        graphObj = Graph(searchType, True, sundayTrafficLines, graphDict)

        if searchType == 'BFS':
            totalPathCost = bfsIterator(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode])
        if searchType == 'UCS':
            totalPathCost = informedSearch(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode], 0, 0,
                                           ucsCostFunction)
        if searchType == 'A*':
            totalPathCost = informedSearch(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode],
                                           0, nameToNodeMap[startNode].sundayTraffic, acostFunction)
        if searchType == 'DFS':
            totalPathCost = dfsIterator(graphObj, nameToNodeMap[startNode], nameToNodeMap[goalNode])

        writeOutput('output.txt', totalPathCost)

if __name__ == '__main__':
    main()