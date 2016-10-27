from collections import deque
import AI_search.helpers.helper.Priority as Priority
import AI_search.helpers.helper.fetchgcost as fetchgcost
import AI_search.helpers.helper.Route as Route
import AI_search.helpers.helper.findNodeInQueue as findNodeInQueue
import AI_search.helpers.helper.createPath as createPath
# --------------------------------DFS Helpers----------------------------------------------------------
def dfs_search(graphObj, startNodeObj, goalNodeObj):
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
