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
