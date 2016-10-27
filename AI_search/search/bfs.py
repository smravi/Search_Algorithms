import queue
import AI_search.helpers.helper.Route as Route
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


def bfs_search(graphObj, startNodeObj, goalNodeObj):
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
