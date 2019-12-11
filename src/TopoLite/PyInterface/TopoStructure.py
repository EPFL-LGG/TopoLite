import os
import sys
import cvxpy as cp
import numpy as np
import compas
import compas.files
from compas.datastructures import Mesh
from compas.datastructures import mesh_planarize_faces

print(os.path.realpath(__file__))
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path +  '/../../../Release')
print(sys.path)

from pyTopo import *

def check_interlock(check):
    A = check.getInterlockingMat(True)
    m = A.shape[0]
    n = A.shape[1]
    c = np.ones(m)

    x = cp.Variable(n)
    t = cp.Variable(m)
    prob = cp.Problem(cp.Maximize(c.T@t),
                    [A@x >= t,
                    t >= 0,
                    t <= c])

    prob.solve(solver=cp.ECOS)
    print(prob.value)
    if abs(prob.value) < 1e-8:
        print("Interlocking")
    else:
        print("Non Interlocking")


if __name__ == "__main__":
    meshes = []
    atBoundary = []
    for id in range(0, 61):
        filename = dir_path + "/../../../data/origin_data/PartGeometry/Part_" + str(id).zfill(2) + ".obj"
        atBoundary.append(False)
        meshes.append(Mesh.from_obj(filename))

    filename = dir_path + "/../../../data/origin_data/PartGeometry/Boundary.obj"
    meshes.append(Mesh.from_obj(filename))
    atBoundary.append(True)

    graph = PyContactGraph(meshes, atBoundary, True)
    check = PyInterlockCheck(graph, PyInterlockCheck.CVXOPT)
    check_interlock(check)