//
// Created by ziqwang on 29.10.18.
//

#ifndef TOPOLOCKCREATOR_GLUIXML_H
#define TOPOLOCKCREATOR_GLUIXML_H

#include "Mesh/PolyMesh.h"
#include "Utility/Controls.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Structure/StrucCreator.h"
#include "IO/gluiVar.h"
#include "CrossMesh/CrossMeshCreator.h"
#include "Mesh/CrossMesh.h"
#include "Mesh/Cross.h"
#include "Structure/Struc.h"

#include <iostream>
#include <unordered_map>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include "pugixml.hpp"

// TI Assembly creator


using IgnoreList = std::unordered_map<int, bool> ;

class XMLIO
{
public:

    gluiVarList varList;
    shared_ptr<StrucCreator> myStrucCreator;
    vector<int> pickPartIDs;
    double interactMatrix[16];
    pugi::xml_document xmldoc;

public:


public:
    void XMLWriter_Mitsuba(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path);
    void XMLWriter_GUISettings(pugi::xml_node &xml_root);
    void XMLWriter_Output(pugi::xml_node &xmlroot, boost::filesystem::path &xmlFileName_path);
    void XMLWriter_PartGeoData(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path);
    void XMLWriter_Animation(string data_folder);
    bool SaveXMLFile(string xmlFileName);

    void XMLReader_GUISettings(pugi::xml_node &xml_root);
    void XMLReader_PartGeoData(pugi::xml_node &xml_root, string &xmlFileName_path);
    bool readXMLFile(string xmlFileName);
    void updateBoundaryPart(pugi::xml_node &xml_root);

public:
    pugi::xml_node xml_groupdata;
    pugi::xml_node xml_mitsuba;
    pugi::xml_node xml_crossdata;
    pugi::xml_node xml_general;

    pugi::xml_document xml_mitsuba_old;
    pugi::xml_node mitsuba_sensor;
};

class gluiMitsuba{
public:

    void change_attribute(pugi::xml_node &root, pugi::xml_node &node, string str_name, string str_attr, string str_value);
    void change_attribute(pugi::xml_node &node, string str_attr, string str_value);
    void scene_settings(pugi::xml_node &xml_root);

    pugi::xml_node mitsuba_sensor;
};

#endif //TOPOLOCKCREATOR_GLUIXML_H
