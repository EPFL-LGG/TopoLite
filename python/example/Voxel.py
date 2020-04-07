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
import timeit

print(os.path.realpath(__file__))
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path +  '/../../cmake-build-release/python')
print(sys.path)

from pyTopo import *

def check_interlock(check, isPrint = False):
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

    if(isPrint):
        for id in range(int(n / 3)):
            print(x.value[id * 3], x.value[id * 3 + 1], x.value[id* 3 + 2])
        print(prob.value)
    if abs(prob.value) < 1e-6:
        print("Interlocking")
    else:
        print("Non Interlocking")

def outputContacts(graph, filename):
    contact_mesh = graph.getContacts();
    contact_mesh.mergeFaces(1e-4)
    contact_mesh.getCompasMesh().to_obj(filename)

def outputPuzzles(meshes, dirname):
    for id in range(0, len(meshes)):
        filename = os.path.join(dirname, "Part" + str(id).zfill(2) + ".obj")
        meshes[id].to_obj(filename)

if __name__ == "__main__":
    filename = dir_path + "/../../data/Voxel/bunny_15x14x11_K80.puz"
    puzMat = puzIO.read_puzfile_into_numpy(filename)
    meshes = []
    varList = PyParamList();
    for partID in range(1, int(np.amax(puzMat)) + 1):
        vfs = puzOBJ.compute_puz_partOBJ(puzMat, partID)
        compas_mesh = Mesh.from_vertices_and_faces(vfs["vertices"], vfs["faces"])
        if partID < 3:
            poly_mesh = PyPolyMesh(compas_mesh, True, varList)
            meshes.append(poly_mesh)
        else:
            poly_mesh = PyPolyMesh(compas_mesh, False, varList)
            meshes.append(poly_mesh)
        poly_mesh.getCompasMesh().to_obj(dir_path + "/../../data/Voxel/bunny/part_{}.obj".format(partID))
        
    graph = PyContactGraph(meshes, 0.001)
    outputContacts(graph, dir_path + "/../../data/Voxel/bunny/contact_remesh.obj")

    check = PyInterlockCheck(graph, PyInterlockCheck.CLP)

    start = timeit.default_timer()
    # Your statements here
    print(check.checkInterlocking(True))
    stop = timeit.default_timer()
    print('Time: ', stop - start)

    start = timeit.default_timer()
    check_interlock(check)
    stop = timeit.default_timer()
    print('Time: ', stop - start)

