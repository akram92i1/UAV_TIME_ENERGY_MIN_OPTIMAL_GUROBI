class graph:
    def __init__(self , NodeId , predecessor:list , sucessor:list):
        self.Node = NodeId
        self.predecessor = predecessor 
        self.sucessor = sucessor
        
    def predecessors(self):
        return slef.predecessor