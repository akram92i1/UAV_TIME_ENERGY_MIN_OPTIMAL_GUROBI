import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from graphStructure import graph
from structure import node
import gurobipy as gp
from gurobipy import GRB
import random
from gurobipy import quicksum


def add_subtour_elimination_constraints(model, n, x):
    # Create an empty list to store the subtour elimination constraints
    subtour_constraints = []
    for i in range(1, n):
        for j in range(i + 1, n):
            subtour_constraints.append(model.addConstr(
                quicksum(x[i, k] for k in range(n) if k != i) +
                quicksum(x[k, j] for k in range(n) if k != j) <=
                quicksum(x[i, j] for i in range(n)) - 1, "subtour_" + str(i) + "_" + str(j)))
    return subtour_constraints


def solve(Lambda):
    # Read data of sensors
    sensorData = pd.read_excel('Data/Environement/K=9/SensorInformation.xls')
    sensorCoordinate = []
    # Sensors coordinate
    for row in sensorData.index:
        coordinate = (sensorData['X'][row], sensorData['Y'][row], sensorData['Cluster_Index'][row])
        sensorCoordinate.append(coordinate)
    sensorInformations = sensorCoordinate
    # Cluster Information
    clusterData = pd.read_excel('Data/Environement/K=9/ClustersInformations.xls')
    ClusterInformation = []
    AllClusterIndex = []
    # Cluster coordinate
    for row in clusterData.index:
        coordinate = (clusterData['X'][row], clusterData['Y'][row], clusterData['Cluster_index'][row])
        ClusterInformation.append(coordinate)
        AllClusterIndex.append(clusterData['Cluster_index'][row])
    C = AllClusterIndex[0:-1]
    # Create a graph structure
    predecessor = []
    sucessor = []
    Nodesss = []
    for i in range(len(AllClusterIndex)):
        predecessor = []
        sucessor = []
        for j in range(len(AllClusterIndex)):
            if i == len(AllClusterIndex) - 1 and i != j:
                sucessor.append(j)
            if i != j and j != len(AllClusterIndex) - 1 and i != len(AllClusterIndex) - 1:
                predecessor.append(j)
                sucessor.append(j)
        g = graph(i, predecessor, sucessor)
        Nodesss.append(g)
    N = node(Nodesss)
    AllClusterIndex
    # Normlized Data of the Mission
    nomlizedMissionData = pd.read_excel("NormlizedData.xls")
    missionInformationSubPath = []
    missionInformationScores = []
    for row in nomlizedMissionData.index:
        subPath = (nomlizedMissionData['Cluster_ID'][row], nomlizedMissionData['Cluster_DESTINATION'][row])
        scores = (
        float(nomlizedMissionData['TIME_TRAVEL_CONSUMPTION'][row]), float(nomlizedMissionData['HOVERING_TIME'][row]),
        float(nomlizedMissionData['ENERGY_HOVER'][row]), float(nomlizedMissionData['TRAVEL_ENERGY_COST'][row]))
        missionInformationSubPath.append(subPath)
        missionInformationScores.append(scores)



    for coor in sensorInformations:
        plt.scatter(coor[0], coor[1], c='blue')
        plt.text(coor[0], coor[1], coor[2])
    for coor in ClusterInformation:
        if coor[0] == 0 and coor[1] == 0:
            plt.text(0, 0, 'Base Station')
        plt.scatter(coor[0], coor[1], c='yellow')
        plt.text(coor[0], coor[1], coor[2])
    #plt.show()
    print(N.getpredecessors(2))
    dataDict = {}
    for cluster in range(len(missionInformationSubPath)):
        dataDict.update({missionInformationSubPath[cluster]: missionInformationScores[cluster]})
    # Match subpath with there scores
    combinations, timeTravel, hoveringTime, energyTravel, hoveringEnergy = gp.multidict(dataDict)
    print(combinations)
    #The number of cluster
    numberOfCluster = 5
    # The startingPosition Should be the base station  (C1) --> keep
    startingPosition = 4
    # Initialization of the model
    model = gp.Model('UAV_TimeEnergy_Minimization')
    # SOLVER PARAMETER
    model.params.TimeLimit = 3000
    model.params.NodeLimit = 10000
    model.params.FeasibilityTol = 1e-6  # Tolerance
    # combinations contain the cluster i and the cluster j i--j or a subPath
    x = model.addVars(combinations, vtype=GRB.BINARY, name='SubPath')
    TimeTravel = model.addVars(timeTravel, vtype=GRB.CONTINUOUS, name='timeTravel')
    HoveringTime = model.addVars(hoveringTime, vtype=GRB.CONTINUOUS, name='hoveringTime')
    EnergyTravel = model.addVars(energyTravel, vtype=GRB.CONTINUOUS, name='energyTravel')
    HoveringEnergy = model.addVars(hoveringEnergy, vtype=GRB.CONTINUOUS, name='hoveringEnergy')
    end = model.addVars(C, vtype=GRB.BINARY, name='end')
    # CONSTRAINT ABOUT ending position
    #model.setObjective(0.8 * quicksum(TimeTravel[i, j] + HoveringTime[i, j] for i, j in x) + 0.2 * quicksum(
     #   EnergyTravel[i, j] + HoveringEnergy[i, j] for i, j in x), GRB.MINIMIZE)
    model.setObjective( gp.quicksum( HoveringTime[i,j]+TimeTravel[i,j] * x[i,j] for i,j in combinations ), GRB.MINIMIZE )
    #CONSTRAINTS

    # Constraint leave the starting position once
    model.addConstr(quicksum(x[startingPosition,j] for j in C) == 1 , name="start_%d" % j)
    model.addConstr(quicksum(x[startingPosition, j] for j in range(numberOfCluster) if j!= startingPosition) == 1, name="start_from_starting_position_relaxed")
    model.addConstr(quicksum(end[j] for j in C) == 1, name="end_position")
    # enter  each cluster once
    model.addConstrs(quicksum(x[i, j] for i in N.getpredecessors(j)) == 1 for j in range(numberOfCluster)  if j!= startingPosition)
    # leave each cluster once
    #model.addConstrs(quicksum(x[i, j] for j in N.getsucessors(i)) == 1 for i in range(numberOfCluster) if j!= startingPosition)
    # The size of set solution should equal the numberOfCluster
    model.addConstr(quicksum(x[i, j] for i in range(numberOfCluster) for j in range(numberOfCluster) if
                             i != j and j != startingPosition) == numberOfCluster, name="limit_solution_size")
    # Ending position
    M = len(C)


    def subtour_eilimination(m , where):
        # check if the LP relaxation at this branch-and-bound has an integer solution
        if where == GRB.Callback.MIPSOL:
            #retreive the LP relaxation solution at the B&B node
            xval = m.cbGetSolution(m._x)
            tour_edges = [e for e in m._G.egdes if xval[e] > 0.5]
            # for each subtour add a constarint
            for component in nx.connected_components(m._G.egde_subgraph(tour_edges)):
                if len(component) < m._G.number_of_nodes():
                    inner_edges = [(i,j) for (i,j) in m._G.edges if i in component and j in component]
                    m.cbLazy(gp.quicksum(m._x[e] for e in inner_edges)<= len(component)-1)

    #model.addConstrs(end[j] <= x[startingPosition, j] for j in C)
    #model.addConstrs(x[startingPosition, j] <= end[j] * M for j in C)
    model.update()
    # RUN OPTIMIZATION
    print(model.optimize())
    status = model.status
    if status == GRB.OPTIMAL:
        print("The solution is optimal")
    elif status == GRB.SUBOPTIMAL:
        print("The solution is suboptimal")

    active_arcs = [i for i in x if x[i].x > 0.99]
    def getCoordinateFromClusterIndex(index):
        for coor in ClusterInformation:
            if coor[2] == index:
                return [coor[0], coor[1]]

    for coor in sensorInformations:
        plt.scatter(coor[0], coor[1], c='blue')
        plt.text(coor[0], coor[1], coor[2])

    for coor in ClusterInformation:
        if coor[0] == 0 and coor[1] == 0:
            plt.text(0, 0, 'Base Station')
        plt.scatter(coor[0], coor[1], c='yellow')
        plt.text(coor[0], coor[1], coor[2])

    for subPath in active_arcs:
        cor1 = getCoordinateFromClusterIndex(subPath[0])
        cor2 = getCoordinateFromClusterIndex(subPath[1])
        X = [cor1[0], cor2[0]]
        Y = [cor1[1], cor2[1]]
        plt.plot(X, Y)

    # Print the subPath
    plt.show()
    print(active_arcs)
    edges = []
    for edge in missionInformationSubPath:
        edges.append(edge)
    edges = tuple(edges)
    print(edges)
    # Add the Miller-Tucker-Zemlin variables and constraints and resolve
    allNodes = [i for i in range(0, numberOfCluster+1)]
    allNodes = tuple(allNodes)
    print(allNodes)
    u = model.addVars(allNodes)
    model.addConstrs(u[i] - u[j] + (numberOfCluster*x[i , j]) <= numberOfCluster-1 for i,j in edges if j != startingPosition)
    model.params.TimeLimit = 300
    print("################################################  MTZ ###################################################")
    model.update()
    model.optimize()
    print("Active arcs after  MTZ ",active_arcs)

    def getCoordinateFromClusterIndex(index):
        for coor in ClusterInformation:
            if coor[2] == index:
                return [coor[0], coor[1]]

    for coor in sensorInformations:
        plt.scatter(coor[0], coor[1], c='blue')
        plt.text(coor[0], coor[1], coor[2])

    for coor in ClusterInformation:
        if coor[0] == 0 and coor[1] == 0:
            plt.text(0, 0, 'Base Station')
        plt.scatter(coor[0], coor[1], c='yellow')
        plt.text(coor[0], coor[1], coor[2])

    for subPath in active_arcs:
        cor1 = getCoordinateFromClusterIndex(subPath[0])
        cor2 = getCoordinateFromClusterIndex(subPath[1])
        X = [cor1[0], cor2[0]]
        Y = [cor1[1], cor2[1]]
        plt.plot(X, Y)

        # Print the subPath
    plt.show()

if __name__ == '__main__':
    Lambda = 0.7
    solve(Lambda)