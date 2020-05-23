//
// Created by ziqwang on 29.10.18.
//

#ifndef TOPOLOCKCREATOR_GLUIXML_H
#define TOPOLOCKCREATOR_GLUIXML_H

#include "TopoLite/Utility/HelpDefine.h"
#include "TopoLite/IO/InputVar.h"
#include "TopoLite/Mesh/CrossMesh.h"

#include <iostream>
#include <unordered_map>
#include <string>
#include "TopoLite/Utility/TopoObject.h"
#include "pugixml.hpp"

// TI Assembly creator

using IgnoreList = std::unordered_map<int, bool> ;

struct InteractData{
    double angle;
    double x, y;
    double scale;
};

struct InputMeshNormalizeData{
    Vector3f trans;
    float scale;
};

struct XMLData{
    shared_ptr<InputVarList> varList;
    shared_ptr<PolyMesh<double>> reference_surface;
    shared_ptr<PolyMesh<double>> cross_mesh;

    //old data
    vector<int> pickPartIDs;
    double interactMatrix[16];


    //for rhino dll
    InteractData interact_delta;
    InputMeshNormalizeData normalizedData;
};

class XMLIO
{
public:

    pugi::xml_document xmldoc;
    pugi::xml_node xml_groupdata;
    pugi::xml_node xml_mitsuba;
    pugi::xml_node xml_crossdata;
    pugi::xml_node xml_general;

    pugi::xml_document xml_mitsuba_old;
    pugi::xml_node mitsuba_sensor;

public:

//    bool XMLWriter(string xmlFileName, XMLData &data);
//    void XMLWriter_Mitsuba(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path, XMLData &data);
//    void XMLWriter_GUISettings(pugi::xml_node &xml_root, XMLData &data);
//    void XMLWriter_Output(pugi::xml_node &xmlroot, boost::filesystem::path &xmlFileName_path, XMLData &data);
//    void XMLWriter_PartGeoData(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path, XMLData &data);

    bool XMLReader(const string xmlFileName, XMLData &data);
    void XMLReader_GUISettings(pugi::xml_node &xml_root, XMLData &data);
//    void XMLReader_PartGeoData(pugi::xml_node &xml_root, string &xmlFileName_path, XMLData &data);
//    void XMLReader_Boundary(pugi::xml_node &xml_root, XMLData &data);

public:

    vector<std::string> split(const string str_text, const char separator) const;
};

//class MitsubaWriter : public TopoObject
//{
//public:
//
//    shared_ptr<StrucCreator> myStrucCreator;
//    pugi::xml_node mitsuba_sensor;
//
//public:
//
//    MitsubaWriter(shared_ptr<InputVarList> var): TopoObject(var){}
//public:
//
//    void change_attribute(pugi::xml_node &root, pugi::xml_node &node, string str_name, string str_attr, string str_value);
//    void change_attribute(pugi::xml_node &node, string str_attr, string str_value);
//    void scene_settings(pugi::xml_node &xml_root, XMLData &data);
//};

#endif //TOPOLOCKCREATOR_GLUIXML_H
