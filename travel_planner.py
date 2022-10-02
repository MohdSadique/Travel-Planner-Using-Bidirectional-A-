import heapq
from collections import defaultdict
import math


pos_to_node = {}
graph = defaultdict(list)
nodes = {}
edges = defaultdict(lambda: float('inf'))
h2 = defaultdict(lambda: float(0))

def addnodes(nodes, nodefile):
    with open(nodefile, "r") as rf:
        for line in rf.readlines():
            node, x, y = map(float, line.split())
            nodes[int(node)] = x, y
            pos_to_node[(x, y)] = int(node)



def addedges(edges, edgesfile):
    with open(edgesfile, "r") as rf:
        for line in rf.readlines():
            v1, v2, dist = map(float, line.split())
            if edges[int(v1), int(v2)] > dist:
                edges[int(v1), int(v2)] = dist


def addheurestic(h2, hfile):
    with open(hfile, "r") as rf:
        for line in rf.readlines():
            node, hval = map(float, line.split())
            h2[int(node)] = hval
            

# bidriectional astar implimentation

class Heap:

    def __init__(self):

        self.Discovered = []

    # method for adding the givenItem to the heap
    def addToHeap(self, givenItem):

        self.Discovered.append(givenItem)
        self.heapify()

    # method for getting the deep copy of the heap list
    def getHeap(self):

        return self.Discovered

    # function for updating the cost of a node in the heap
    def updateNodeCostInHeap(self, currentNode, fValue, gValue, sourceNode):

        for node in self.Discovered:

            if node.id == currentNode.id:

                node.g1 = gValue
                node.f1  = fValue
                node.parent = sourceNode
                self.heapify()

    # method to change the position of the elements in the heap in order to satisfy the heap property
    def heapify(self):

        for item in range(len(self.Discovered)):

            # if the index is greater than or equal to 1 and the parent is greater than children, then swap
            while item >= 1 and self.Discovered[item].f1  <= self.Discovered[item//2].f1 :

                if self.Discovered[item].f1  < self.Discovered[item//2].f1 :

                    self.swap(self.Discovered, item, item // 2)

                elif self.Discovered[item].f1  == self.Discovered[item//2].f1 :

                    if self.Discovered[item].h1  < self.Discovered[item//2].h1 :

                        self.swap(self.Discovered, item, item // 2)

                item = item // 2

    # method to get the minimum item from the heap
    def minItemInHeap(self):

        result = self.Discovered.pop(0)
        self.heapify()
        return result

    # method to get the value of the root element at the heap
    def rootItemAtHeap(self):

        return self.Discovered[0]

    # method for swapping the values in the heap
    def swap(self, heap, firstIndex, secondIndex):

        tempVal = heap[firstIndex]
        heap[firstIndex] = heap[secondIndex]
        heap[secondIndex] = tempVal


class Node:

    def __init__(self, parent, id):

        self.parent = parent
        self.secondParent = None
        self.id = id

        self.g1 = 0
        self.h1  = 0
        self.f1  = 0
        self.g2 = 0
        self.h2  = 0
        self.f2  = 0
        # print(f"id = {id} ")



class Graph:

    def __init__(self):

        self.map = []
        self.height = 0
        self.width = 0

    # method for building the map in the required format
    def buildGraph(self, filename):
        pass
 
    def heuristic(self, currentNode, targetNode):
            # -------------Euclidean Heuristic-------------------------
            return math.sqrt((nodes[currentNode.id][0] - nodes[targetNode.id][0]) ** 2 + (nodes[currentNode.id][1] - nodes[targetNode.id][1]) ** 2)
    
    def heuristic2(self, currentNode):
            # -------------Euclidean Heuristic-------------------------
            return h2[currentNode.id]

    # method for getting the node with minimum value from the either direction
    def minValueNodeFromEitherDirection(self, forwardDiscovered, backwardDiscovered, forwardFinalized, backwardFinalised):

        forwardMinItem = forwardDiscovered.rootItemAtHeap()
        backwardMinItem = backwardDiscovered.rootItemAtHeap()
        
        if forwardMinItem.f1  < backwardMinItem.f1 :
        
            result = forwardDiscovered.minItemInHeap()
            forwardFinalized.append(result)
            return result, "forward"
        
        elif forwardMinItem.f1  > backwardMinItem.f1 :
        
            result = backwardDiscovered.minItemInHeap()
            backwardFinalised.append(result)
            return result, "backward"
        
        elif forwardMinItem.f1  == backwardMinItem.f1 :
        
            if forwardMinItem.h1  > backwardMinItem.h1 :
        
                result = forwardDiscovered.minItemInHeap()
                forwardFinalized.append(result)
                return result, "forward"
        
            else:
        
                result = backwardDiscovered.minItemInHeap()
                backwardFinalised.append(result)
                return result, "backward"

    def minItemFromList(self, node, givenList):

        for item in givenList:

            if item.id == node.id:

                return True

        return False

    # method implementing bi-directional search algorithm
    def biDirectionalSearch(self, source, target,graph):

        # if the source and the target, are the same, then return an empty path with a cost of 0
        if source == target:
            return ([], 0)
        
        sourceNode = Node(None, source)
        targetNode = Node(None, target)

        # initialising the forwardDiscovered list as a heap and the finalized list as a normal list
        forwardDiscovered = Heap()
        forwardFinalized = []

        backwardDiscovered = Heap()
        backwardFinalised = []

        forwardDiscovered.addToHeap(sourceNode)
        backwardDiscovered.addToHeap(targetNode)

        numberOfNodes = 0

        while len(forwardDiscovered.getHeap()) != 0 and len(backwardDiscovered.getHeap()) != 0:

            # get the node with the smallest value from the either direction (forward or backward)
            smallestValueNode = self.minValueNodeFromEitherDirection(forwardDiscovered, backwardDiscovered, forwardFinalized, backwardFinalised)

            # if the smallest value node happens to be from the forward frontier
            if smallestValueNode[1] == "forward":

                # if the smallest value node happens to be from the forward frontier and it happens to be in the finalised list of the backward frontier
                if self.minItemFromList(smallestValueNode[0], backwardFinalised):

                    shortestPath = []

                    currentNode = smallestValueNode[0]
                    totalCost = currentNode.f1 

                    # tracing back to the source node in order to retrieve the path
                    while currentNode is not None:
                        shortestPath = [currentNode.id] + shortestPath
                        currentNode = currentNode.parent

                    return shortestPath, totalCost, numberOfNodes

                if True:
                    # get neighbours of current node
                    neighborNode = []
                    for val in graph[smallestValueNode[0].id]:
                        neighborNode.append(val[0]) 
                    
                    for nodeid in neighborNode:

                        childNodeParent = smallestValueNode[0]
                        # print("parent = ", childNodeParent.id)
                        childNode = Node(childNodeParent, nodeid)

                        firstFlagChecker = False

                        # if child node is in the finalised list, then skip to the next neighbouring node
                        for forwardFinalizedNode in forwardFinalized:

                            if forwardFinalizedNode.id == nodeid:

                                firstFlagChecker = True

                                break

                        if firstFlagChecker:

                            continue

                        # if the child node is not in the finalized list, then search the forwardDiscovered list
                        secondFlagChecker = True

                        forwardDiscoveredList = forwardDiscovered.getHeap()

                        # checking if the node is in the forwardDiscovered list, if the node is in the forwardDiscovered list then skip to the next node
                        for forwardDiscoveredNode in forwardDiscoveredList:

                            if forwardDiscoveredNode.id == nodeid:

                                secondFlagChecker = False

                                break

                        if secondFlagChecker:
                            numberOfNodes += 1
                            childNode.g1 = childNodeParent.g1 + edges(childNodeParent.id,nodeid)
                            childNode.h1  = self.heuristic(childNode, targetNode) +self.heuristic2(childNode)
                            childNode.f1  = childNode.g1 + childNode.h1 
                            forwardDiscovered.addToHeap(childNode)

                        # if the child node is not in the finalized list but it's in the forwardDiscovered list, then check whether we are getting a better value for that node
                        else:
                            childNodeCurrentGVal = childNodeParent.g1 + edges(childNodeParent.id,nodeid)
                            childNodeCurrentHVal = self.heuristic(childNode, targetNode) +self.heuristic2(childNode)
                            childNodeCurrentFVal = childNodeCurrentHVal + childNodeCurrentGVal

                            for nodeIndex in forwardDiscovered.getHeap():

                                if nodeIndex.id == childNode.id and nodeIndex.f1  <= childNodeCurrentFVal:

                                    break
                                # if we are getting a better g value for the current child node, then update that value accordingly for that node in the forwardDiscovered list
                            else:
                                forwardDiscovered.updateNodeCostInHeap(childNode, childNodeCurrentFVal, childNodeCurrentGVal, smallestValueNode[0])

            # if the smallest value node happens to be from the backward frontier
            elif smallestValueNode[1] == "backward":

                if self.minItemFromList(smallestValueNode[0], forwardFinalized):

                    shortestPath = []

                    currentNode = smallestValueNode[0]
                    totalCost = currentNode.f1 

                    # tracing back to the source node in order to retrieve the path
                    while currentNode is not None:

                        shortestPath = [currentNode.id] + shortestPath
                        currentNode = currentNode.parent

                    return shortestPath, totalCost, numberOfNodes

                if True:

                    neighborNode =[]
                    for e in edges.keys():
                        # print("e = ",e)
                        if e[1] == smallestValueNode[0].id:
                            neighborNode.append(e[1])
                        
                    for nodeid in neighborNode:

                        childNodeParent = smallestValueNode[0]
                        print("parent = ", childNodeParent.id)
                        childNode = Node(childNodeParent, nodeid)

                        firstFlagChecker = False

                        # if child node is in the finalised list, then skip to the next neighbouring node
                        for backwardFinalizedNode in backwardFinalised:

                            if backwardFinalizedNode.id == nodeid:

                                firstFlagChecker = True

                                break

                        if firstFlagChecker:

                            continue

                        # if the child node is not in the finalized list, then search the backward frontier's discovered list
                        secondFlagChecker = True

                        backwardDiscoveredList = backwardDiscovered.getHeap()

                        # checking if the node is in the forwardDiscovered list, if the node is in the forwardDiscovered list then skip to the next node
                        for backwardDiscoveredNode in backwardDiscoveredList:

                            if backwardDiscoveredNode.id == nodeid:

                                secondFlagChecker = False

                                break

                        if secondFlagChecker:
                                childNode.g1 = childNodeParent.g1 + edges[(childNodeParent.id,nodeid)]
                                childNode.h1  = self.heuristic(childNode, sourceNode) +self.heuristic2(childNode)
                                childNode.f1  = childNode.g1 + childNode.h1
                                backwardDiscovered.addToHeap(childNode)

                        # if the child node is not in the finalized list but it's in the forwardDiscovered list, then check whether we are getting a better value for that node
                        else:

                            childNodeCurrentGVal = childNodeParent.g1 + 1
                            childNodeCurrentHVal = self.heuristic(childNode, targetNode) +self.heuristic2(childNode)
                            childNodeCurrentFVal = childNodeCurrentHVal + childNodeCurrentGVal

                            for nodeIndex in forwardDiscovered.getHeap():

                                if nodeIndex.position == childNode.position and nodeIndex.f1  <= childNodeCurrentFVal:
                                    break
                            # if we are getting a better g value for the current child node, then update that value accordingly for that node in the forwardDiscovered list
                                else:
                                    backwardDiscovered.updateNodeCostInHeap(childNode, childNodeCurrentFVal, childNodeCurrentGVal, smallestValueNode[0])
        print("No possible path exist from the given source to given target.")
        return ([], 0)





if __name__ == "__main__":
    
    addnodes(nodes, "sample_nodes.txt")

    addedges(edges, "sample_edges.txt")

    addheurestic(h2,"heuristic.txt")
    
    # graph = defaultdict(list)
    for (u, v), wt in edges.items():
        graph[u].append((v, wt))

    x = Graph()
    
    ch = 1
    while ch != 0:
        ch = int(input("\nenter choice\n1.find the optimal path \n0.exit\nChoice:"))
        if ch == 1:
            source_node = int(input("source node:"))
            dest_node = int(input("destination node:"))
            result = x.biDirectionalSearch(source_node,dest_node,graph)
            # print(result)  

