import os
import sys
import cvxpy as cp
import numpy as np
import compas
import compas.files
from compas.datastructures import Mesh
from compas.datastructures import mesh_planarize_faces
from compas_plotters import MeshPlotter
import puzIO
import puzOBJ

print(os.path.realpath(__file__))
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path +  '/../../../Release')
print(sys.path)

from pyTopo import *


def check_interlock(check):
    A = check.getInterlockingMat(False)
    np.set_printoptions(threshold=sys.maxsize)
    print(A)
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
    for id in range(int(n / 3)):
        print(x.value[id * 3], x.value[id * 3 + 1], x.value[id* 3 + 2])
    print(prob.value)
    if abs(prob.value) < 1e-6:
        print("Interlocking")
    else:
        print("Non Interlocking")


if __name__ == "__main__":
    filename = dir_path + "/../../../data/Cube_4x4x4_K9.puz"
    puzMat = puzIO.read_puzfile_into_numpy(filename)
    print(np.amax(puzMat))
    meshes = []
    atBoundary = []
    for partID in range(1, int(np.amax(puzMat)) + 1):
        vfs = puzOBJ.compute_puz_partOBJ(puzMat, partID)
        meshes.append(Mesh.from_vertices_and_faces(vfs["vertices"], vfs["faces"]))
        atBoundary.append(False)
    
    atBoundary[0] = True;
    atBoundary[1] = True;

    graph = PyContactGraph(meshes, atBoundary, True)
    check = PyInterlockCheck(graph, PyInterlockCheck.CVXOPT)

    mergeMeshes = graph.mergeFaces(meshes)
    id = 0;
    for mesh in mergeMeshes:
        filename = dir_path + "/../../../data/puz/" + str(id) + ".obj"
        mesh.to_obj(filename)
        id = id + 1

    check_interlock(check)

