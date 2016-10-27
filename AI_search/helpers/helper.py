
from collections import namedtuple
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





