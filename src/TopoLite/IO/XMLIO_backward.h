//
// Created by ziqwang on 25.05.20.
//

#ifndef TOPOLITE_XMLIO_BACKWARD_H
#define TOPOLITE_XMLIO_BACKWARD_H

#include <iostream>
#include <unordered_map>
#include <string>
#include <pugixml.hpp>
#include <filesystem>
#include "IOData.h"


class XMLIO_backward {
public:
    bool XMLReader(const std::string xmlFileName, XMLData &data);
    void XMLReader_GUISettings(pugi::xml_node &xml_root, XMLData &data);

    bool XMLReader_ReferenceSurface(pugi::xml_node &xml_root, const string xmlFileName_path, XMLData &data);
    bool XMLReader_CrossMesh(pugi::xml_node &xml_root, const string xmlFileName_path, XMLData &data);
    void XMLReader_Boundary(pugi::xml_node &xml_root, XMLData &data);

private:
    Vector3d readXYZ(std::string xyz_str);

    vector<std::string> split(const std::string str_text, const char separator) const;
};


#endif //TOPOLITE_XMLIO_BACKWARD_H
