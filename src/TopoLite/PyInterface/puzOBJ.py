import puzIO
import sys
import numpy as np

def compute_puz_partOBJ(puzMat, partID):

    try:
        [Nx, Ny, Nz] = list(puzMat.shape)
    except:
        print("Error: the shape of the voxel matrix is wrong")

    v2i = lambda v: v[0] + v[1] * (Nx + 1) + v[2] * (Nx + 1) * (Ny + 1)

    part_voxel = []
    for iz in range(Nz):
        for iy in range(Ny):
            for ix in range(Nx):
                if(puzMat[ix][iy][iz] == partID):
                    part_voxel.append([ix, iy, iz])

    cube_vertice = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 1, 1]
    ]
    cube_vertice = [np.array(v) for v in cube_vertice]

    dV =[
        [1, 0, 0],
        [-1, 0, 0],
        [0, 1, 0],
        [0, -1, 0],
        [0, 0, 1],
        [0, 0, -1]
    ]
    dV = [np.array(dv) for dv in dV]

    cube_face = [
        [1, 3, 7, 5], # +x
        [0, 4, 6, 2], # -x
        [2, 6, 7, 3], # +y
        [0, 1, 5, 4], # -y
        [4, 5, 7, 6], # +z
        [0, 2, 3, 1], # -z
    ]

    size = np.array([Nx, Ny, Nz])
    Nmax = max(size) + 1
    faces = []
    vertices = []
    vmap = {}
    for voxel in part_voxel:
        pos = np.array(voxel)
        for i, dv in enumerate(dV):
            npos = pos + dv
            #face is between two voxels in the same part
            if npos.min() >= 0 and (size - npos).min() > 0:
                if puzMat.item(tuple(npos)) == puzMat.item(tuple(pos)):
                    continue
            face = [list(cube_vertice[vid] + pos) for vid in cube_face[i]]
            for fv in face:
                if v2i(fv) not in vmap:
                    vmap[v2i(fv)] = len(vmap)
                    #vertices.append([fv[0] / (Nx + 1) - 0.5, fv[1] / (Ny + 1) - 0.5, fv[2] / (Nz + 1) - 0.5])
                    vertices.append([fv[0] / Nmax - 0.5, fv[2] / Nmax - 0.5, -(fv[1] / Nmax - 0.5)])
            faces.append([vmap[v2i(fv)] for fv in face])

    return {"vertices" : vertices, "faces" : faces}

def main():
    if len(sys.argv) < 2:
        print("Error: no input .puz file is indicated.")
        exit();

    if len(sys.argv) == 2:
        puzMat = puzIO.read_puzfile_into_numpy(sys.argv[1])
        print(compute_puz_partOBJ(puzMat, partID = 1))
    #plt.show()

if __name__ == "__main__":
    main()
