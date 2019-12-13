import numpy as np
import sys


#input format:
    #readpuz.py puzzle.puz
    #readpuz.py puzzle.puz out.xml

def compute_puz_contactinfo (puzMat):
    try:
        [Nx, Ny, Nz] = list(puzMat.shape)
    except:
        print("Error: the shape of the voxel matrix is wrong")

    #Contact dictionary
    contact = []
    dV = [[-1, 0, 0],
          [1, 0, 0],
          [0, -1, 0],
          [0, 1, 0],
          [0, 0, -1],
          [0, 0, 1]]
    dV = [np.array(dv) for dv in dV]
    size = np.array([Nx, Ny, Nz])

    for iz in range(Nz):
        for iy in range(Ny):
            for ix in range(Nx):
                pos = np.array([ix, iy, iz])
                id = int(puzMat[ix, iy, iz])
                if puzMat[ix, iy, iz] > 0:
                    for dv in dV:
                        npos = pos + dv
                        if(npos.min() >=0 and (size - npos).min()> 0):
                            nid = int(puzMat.item(tuple(npos)))
                            if nid != id and nid != 0:
                                #id -> nid
                                contact.append([id, nid, *list(dv)])
                                #nid -> id
                                ndv = dv *(-1)
                                contact.append([nid, id, *list(ndv)])
    return contact

#read .puz file for 3d
def read_puzstring_into_numpy(num_data):
    [Nx, Ny, Nz] = [num_data[0], num_data[1], num_data[2]]
    puzMat = np.zeros((Nx, Ny, Nz))
    it_data = iter(num_data[3:])
    # read reset matrix
    for iz in range(Nz):
        for iy in range(Ny):
            for ix in range(Nx):
                puzMat[ix, iy, iz] = int(next(it_data))
    return puzMat

def read_puzfile_into_numpy (file_path):
    with open(file_path, "r") as puz_file:
        str_data = puz_file.read().replace("\n", " ")
        num_data = [int(x) for x in str_data.split()]
        puzMat = read_puzstring_into_numpy(num_data)
        # dimension in x, y, z direction
    return puzMat

#DEBUG
def main():
    if len(sys.argv) < 2:
        print("Error: no input .puz file is indicated.")
        exit();

    if len(sys.argv) == 2:
        puzMat = read_puzfile_into_numpy(sys.argv[1])
        print(puzMat)
    else:
        print("Error: command parameters are too long")
        exit()

if __name__ == "__main__":
    main()
