//
// Created by ziqwang on 25.05.20.
//

#ifndef TOPOLITE_XMLIO_BACKWARD_H
#define TOPOLITE_XMLIO_BACKWARD_H

#include <iostream>
#include <unordered_map>
#include <string>
#include <pugixml.hpp>
#include "IOData.h"

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesysten>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
using std::string;
#endif


class XMLIO_backward {
public:
    bool XMLReader(const std::string xmlFileName, IOData &data);
    void XMLReader_GUISettings(pugi::xml_node &xml_root, IOData &data);

    bool XMLReader_ReferenceSurface(pugi::xml_node &xml_root, const std::string xmlFileName_path, IOData &data);
    bool XMLReader_CrossMesh(pugi::xml_node &xml_root, const std::string xmlFileName_path, IOData &data);
    void XMLReader_Boundary(pugi::xml_node &xml_root, IOData &data);

private:
    Vector3d readXYZ(std::string xyz_str);

    vector<std::string> split(const std::string str_text, const char separator) const;

    Eigen::Matrix4d toEigenMatrix(double *interactMatrix){
        Eigen::Matrix4d interactMat;
        interactMat << interactMatrix[0], interactMatrix[4], interactMatrix[8], interactMatrix[12],
                interactMatrix[1], interactMatrix[5], interactMatrix[9], interactMatrix[13],
                interactMatrix[2], interactMatrix[6], interactMatrix[10], interactMatrix[14],
                interactMatrix[3], interactMatrix[7], interactMatrix[11], interactMatrix[15];
        return interactMat;
    }
};


#endif //TOPOLITE_XMLIO_BACKWARD_H
