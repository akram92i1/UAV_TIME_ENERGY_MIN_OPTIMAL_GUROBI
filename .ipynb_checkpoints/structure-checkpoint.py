class node:
    def __init__(self,Nodes:list):
        self.Nodes = Nodes
        
    def getsucessors(self , nodeID):
        for node in self.Nodes:
            if node.Node == nodeID:
                return node.sucessor       
    def getpredecessors(self, nodeID):
        for node in self.Nodes:
            if node.Node == nodeID:
                return node.predecessor
            
            