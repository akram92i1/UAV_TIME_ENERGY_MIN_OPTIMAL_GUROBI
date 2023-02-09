import datetime
from itertools import groupby

from pulp import *
import numpy as np
from math import *

from util import City

#data of problem
rnd = np.random
n = 14
rr = 6
H = 100
Beta0 = 5
v = 10
deltat = 5
Pp = 39
p_trans = 37
k = 3 #nbr depots
d_UAV = 0
eta = 1
t_H = 5
eMax = 7300


dd = datetime.datetime.now()
#Liste des tours/sous trajectoires
R = [r for r in range(1, rr + 1)]

path_to_cplex = r'/Applications/CPLEX_Studio221/cplex/bin/x86-64_osx/cplex'
solver = CPLEX_CMD(path=path_to_cplex, msg=0, gapRel=0.05, threads=6)

#Liste des capteurs
N = [i for i in range(1, n+1)]
#Liste des depots
K = [i for i in range(n+1, n + k + 1)]
#Liste dummy nodes
D = [i for i in range(n + k + 1, n + k + rr + 1)]

#Liste des capteurs + les dépôts + les duplicats des dépôts
NK = N + K

print(NK)

#Emax de l'UAV
Emax = [eMax] #[rnd.randint(900, 1000)] #
#Puissance de transmission de l'UAV
Pu = [p_trans]

#generation des coordonnées des capteurs + dépôts

#loc_x = [967.0298390136767,972.6843599648844, 697.7288245972708,976.2744547762418,252.98236238344396,779.3829217937524,862.9932355992222,163.8422414046987,8.986097667554983,44.16005793149957, 436.14664687979763, 786.3059859350609]


#loc_x = [221.9931710897395, 870.7323061773764, 206.7191553394264, 918.6109079379215, 488.4111887948291, 611.7438629026457, 765.9078564803156, 518.4179878729433, 296.80050157622196, 187.72122866125162, 80.74126876487486, 738.44029619897]
loc_x = rnd.rand(n) * 1000

kx = [250,750]
loc_kx = rnd.choice(kx,k)
#loc_kx = [750,250,750]
#loc_kx = [250,750,750]

print("loc_kx     ", loc_kx)

for ll in range(k):
    loc_x = np.append(loc_x, loc_kx[ll])

loc_y = rnd.rand(n) * 1000
#loc_y = [547.2322491757224, 714.8159936743647, 216.08949558037637, 6.230255204589863, 434.7915324044458, 197.68507460025307, 983.4006771753128, 597.3339439328593, 386.5712826436294, 956.652967714236, 948.9773067815628, 866.2892985816984]
#loc_y = [764.3726086611824, 110.90076173906371, 204.15474783059219, 119.09535747826038, 877.9030712603621, 523.6752895998791, 492.1359986061542, 731.8711002604598, 14.58075109635204, 93.36303363433629, 826.5542486873562, 833.492742062839]
ky = [100, 300, 500, 700, 900]
loc_ky = rnd.choice(ky,k)
#loc_ky = [100, 700, 700]
#loc_ky = [900.0, 300.0, 700.0]

print("loc_ky     ", loc_ky)

for ll in range(k):
    loc_y = np.append(loc_y, loc_ky[ll])

print(loc_x)

reseau_ok = 1
lists = []
for i in range(n):
    lists.append(City(loc_x[i], loc_y[i], 0))
listd = []
for i in range(n,n+k):
    listd.append(City(loc_x[i], loc_y[i], 0))

for s in lists:
    id_nearest_depot_to_sensor, nearest_depot_to_sensor = min(enumerate(listd), key=lambda item: item[1].distance(s))
    energie = nearest_depot_to_sensor.energie_F(s)*2 + s.energie_H()
    if energie > eMax :
        #print("Alerte !! Il existe des capteurs n'appartiennent pas au reseau")
        reseau_ok = 0
        break
print("reseau_ok   ", reseau_ok)

#channel power gain
# g = [ceil(Beta0 / (H ** 2)) for i in N]
#
# #the harvested energy
# EH = {i: deltat * Pu[0] * g[i-1] for i in N}

EH = {i: t_H * ((eta * Pu[0]) / (d_UAV + (H ** 2))) for i in N}

#Arc
A = [(i, j) for i in NK for j in NK if i != j]

#Time i --> j
T = {(i, j): (np.hypot(loc_x[i-1] - loc_x[j-1], loc_y[i-1] - loc_y[j-1]))/v for i, j in A}


# Setting the problem
prob = LpProblem("minimize_the_mission_total_time_of_UAV", LpMinimize)

# Description Variables

alpha = LpVariable.dicts("alpha", [(i, j, r) for i in NK for j in NK for r in R], 0, 1,
                         cat='Binary')
beta = LpVariable.dicts("beta", [(i, r) for i in NK for r in R], 0, 1, cat='Binary')
u = LpVariable.dicts("u", [(i, r) for i in N for r in R], 1, n + rr - 1, cat='Integer')

# Objective function
prob += lpSum(T[i, j] * alpha[i, j, r] for i, j in A for r in R)

# Constraints
# Each node in N must be visited exactly once
for i in N:
    prob += lpSum(beta[i, r] for r in R) == 1

# the trajectory must start with a depot : the first element in r=1 is the first depot n+1
prob += lpSum(alpha[n+1, j, 1] for j in N) == 1

# each sub trajectory r must start with a depot and finish with a depot
# end depot for r must be the start depot for r+1
for r in range(1,rr):
    for k in K :
        prob += lpSum(alpha[i, k, r] for i in NK if k != i) == lpSum(alpha[k, j, r+1] for j in NK if j != k)
        #prob += alpha[0, k, r] == lpSum(alpha[k, j, r + 1] for j in N)


# If a node (including the depot replicas and dummy node) is visited during tour r then we must activate exactly
# one entering edge and one leaving edge
for i in N:
    for r in R:
        prob += lpSum(alpha[i, j, r] for j in NK if j != i) == beta[i, r]
        prob += lpSum(alpha[j, i, r] for j in NK if j != i) == beta[i, r]


# The total energy consumed in a tour must be lower than the maximum energy
for r in R:
    prob += Pp * lpSum(T[(i, j)] * alpha[i, j, r] for i, j in A) + lpSum(EH[i] * beta[i, r] for i in N) <= Emax[0]


# Constraints eliminating sub-tours (Miller–Tucker–Zemlin formulation of TSP)
for r in R:
    for i in N:
        for j in N:
            if i != j:
                prob += u[i, r] - u[j, r] + (n + 2) * alpha[i, j, r] <= n + 1

solution = prob.solve(solver)
print("Solution Status = ", LpStatus[prob.status])

# Each of the variables is printed with it's resolved optimum value
for v in prob.variables():
    if v.varValue == 1 and v.name.startswith("alpha"):
        print(v.name, "=", v.varValue)

# Each of the variables is printed with it's resolved optimum value
count = 0
chemin = []
tempList=[]
for r in range(1,rr+1):
    chemin.append("r_"+str(r))
print(chemin)
for v in prob.variables():
    if v.varValue == 1 and v.name.startswith("alpha"):
        count += 1
        x = v.name[7:]
        values = x.split("_")
        values[0] = int(values[0][0:-1])
        if values[0] in K : values[0] = "D_"+ str(values[0])
        values[1] = int(values[1][0:-1])
        if values[1] in K: values[1] = "D_" + str(values[1])
        values[2] = int(values[2][0:-1])
        print(values)

        for r in range(1,rr+1):
            if values[2] == r:
                idr = chemin.index("r_"+str(r))
                idr1 = 0
                if "r_"+str(r+1) in chemin : idr1 = chemin.index("r_"+str(r+1))
                if idr1 == 0 and idr == len(chemin)-1 :
                    chemin.insert(idr + 1, values[0])
                    chemin.insert(idr + 2, values[1])
                elif idr1-idr == 1 :
                    chemin.insert(idr + 1,values[0])
                    chemin.insert(idr + 2, values[1])
                else :
                    if str(values[1]).startswith('D') :
                        if r == rr :
                            chemin.insert(len(chemin), values[1])
                            idr1=len(chemin)-1
                        else:
                            chemin.insert(idr1,values[1])
                        if values[0] not in chemin : chemin.insert(idr1, values[0])
                    elif str(values[0]).startswith('D') :
                        chemin.insert(idr + 1, values[0])
                        if values[1] not in chemin : chemin.insert(idr + 2, values[1])
                    else:
                        id_val0 = 0
                        id_val1 = 0
                        if values[0] in chemin: id_val0 = chemin.index(values[0])
                        if values[1] in chemin: id_val1 = chemin.index(values[1])
                        if id_val0 == 0 and id_val1 != 0 :
                            chemin.insert(id_val1,values[0])
                        elif id_val1 == 0 and id_val0 != 0 :
                            chemin.insert(id_val0+1,values[1])
                        elif id_val0 == 0 and id_val1 == 0 :
                            tempList.append(values)
                break

if len(tempList)>0:
    for v in tempList:
        id_val0 = 0
        id_val1 = 0
        if v[0] in chemin: id_val0 = chemin.index(v[0])
        if v[1] in chemin: id_val1 = chemin.index(v[1])
        if id_val0 == 0 and id_val1 != 0:
            chemin.insert(id_val1, v[0])
        elif id_val1 == 0 and id_val0 != 0:
            chemin.insert(id_val0 + 1, v[1])
        else:
            break
print(tempList)
print(chemin)
chemin_new = [x for x in chemin if not str(x).startswith('r')]
print(chemin_new)
res = [i[0] for i in groupby(chemin_new)]
print(res)

objective_value = value(prob.objective)
print("objective_value", objective_value)
print(datetime.datetime.now()-dd)



