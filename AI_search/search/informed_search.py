import heapq
import AI_search.helpers.helper.Priority as Priority
import AI_search.helpers.helper.createPath as createPath
import AI_search.helpers.helper.Route as Route
import AI_search.helpers.helper.findNodeInQueue as findNodeInQueue
import AI_search.helpers.helper.fetchgcost as fetchgcost

# A* search and Uniform cost search
def informed_search(graphObj, startNodeObj, goalNodeObj, startNode_gcost, startNode_hcost, costFunction):
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

