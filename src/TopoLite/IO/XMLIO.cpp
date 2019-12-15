//
// Created by ziqwang on 22.02.19.
//

#include "XMLIO.h"
#include <sstream>
#include <boost/algorithm/string.hpp>

//**************************************************************************************//
//                                  XML Writer
//**************************************************************************************//

bool XMLIO::XMLWriter(string xmlFileName, XMLData &data)
{
    if(xmlFileName == "") return false;
    boost::filesystem::path xmlFileName_path(xmlFileName);
    if(xmlFileName_path.filename() == "") xmlFileName_path.append("/untitle.xml");
    if(xmlFileName_path.extension() == "") xmlFileName_path.append(".xml");

    std::cout << xmlFileName_path.string() << std::endl;
    bool is_success = boost::filesystem::exists(xmlFileName_path.parent_path());

    if(is_success)
    {
        string file_name = xmlFileName_path.filename().stem().string();
        string data_path = xmlFileName_path.parent_path().string();
        data_path += "/" + file_name + "_data";
        std::cout << data_path << std::endl;

        if(!boost::filesystem::exists(data_path))
        {
            boost::filesystem::create_directories(data_path);
        }
        else {
            if(boost::filesystem::exists(data_path + "/mitsuba.xml"))
            {
                string mistuba_file = data_path + "/mitsuba.xml";
                xml_mitsuba_old.load_file(mistuba_file.c_str());
                mitsuba_sensor =  xml_mitsuba_old.child("scene").child("sensor");
            }
            boost::filesystem::remove_all(data_path);
            boost::filesystem::create_directory(data_path);
        }

        pugi::xml_node documents_node = xmldoc.child("Documents");
        if(!documents_node) documents_node = xmldoc.append_child("Documents");
        xml_general = documents_node;

        XMLWriter_GUISettings(documents_node, data);
        XMLWriter_PartGeoData(documents_node, xmlFileName_path, data);
        if(data.varList->get<bool>("output_mitsuba"))
            XMLWriter_Mitsuba(documents_node, xmlFileName_path, data);
        XMLWriter_Output(documents_node, xmlFileName_path, data);
        string xml_file_path = xmlFileName_path.string();
        xmldoc.save_file(xml_file_path.c_str());
    }
    else
    {
        std::cout << "Output Failed" << std::endl;
        return false;
    }
    return true;
}

void XMLIO::XMLWriter_GUISettings(pugi::xml_node &xml_root, XMLData &data)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    //guisettings
    {
        if(!node_guisettings) node_guisettings = xml_root.append_child("GUISettings");

        InputVarManager varOrganizer;
        //para basic
        {
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Basic");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node_guisettings);
        }

        //Para_ShapeOp
        {
            pugi::xml_node node_shapeop = node_guisettings.child("ShapeOp_Settings");
            if(!node_shapeop) node_shapeop = node_guisettings.append_child("ShapeOp_Settings");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_ShapeOp");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node_shapeop);
        }

        //Para_Mitsuba
        {
            pugi::xml_node node_mitsuba = node_guisettings.child("Para_Mitsuba");
            if(!node_mitsuba) node_mitsuba = node_guisettings.append_child("Para_Mitsuba");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Mitsuba");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node_mitsuba);
        }


        //Para_ContactGraph
        {
            pugi::xml_node node_contactgraph = node_guisettings.child("Para_ContactGraph");
            if(!node_contactgraph ) node_contactgraph  = node_guisettings.append_child("Para_ContactGraph");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_ContactGraph");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node_contactgraph);
        }

        //Para_CrossMesh
        {
            pugi::xml_node node = node_guisettings.child("Para_CrossMesh");
            if(!node ) node  = node_guisettings.append_child("Para_CrossMesh");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_CrossMesh");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Para_Opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(!node ) node  = node_guisettings.append_child("Para_Opt");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Opt");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Para_Assembly
        {
            pugi::xml_node node = node_guisettings.child("Para_Assembly");
            if(!node ) node  = node_guisettings.append_child("Para_Assembly");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Assembly");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Para_Support
        {
            pugi::xml_node node = node_guisettings.child("Para_Support");
            if(!node ) node  = node_guisettings.append_child("Para_Support");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Support");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Optimization_Para
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(!node ) node  = node_guisettings.append_child("Para_Opt");
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Opt");
            for(shared_ptr<InputVar> var: series) varOrganizer.write(var.get(), node);
        }

        // pickPartIDs
        {
            pugi::xml_node node_pickpartIDs = node_guisettings.child("pickPartIDs");
            if (!node_pickpartIDs) node_pickpartIDs = node_guisettings.append_child("pickPartIDs");
            pugi::xml_text text_pickpartIDs = node_pickpartIDs.text();
            string str_pickpartIDs;
            for (int id = 0; id < data.pickPartIDs.size(); id++){
                str_pickpartIDs += to_string(data.pickPartIDs[id]);
                if(id != (int)data.pickPartIDs.size() - 1) str_pickpartIDs += ",";
            }
            text_pickpartIDs.set(str_pickpartIDs.c_str());
        }

        //interactMatrix
        {
            pugi::xml_node node_interactmatrix = node_guisettings.child("Interact_Matrix");
            if(!node_interactmatrix) node_interactmatrix = node_guisettings.append_child("Interact_Matrix");
            pugi::xml_text text_interactmatrix = node_interactmatrix.text();
            char interactmatrix_string[256] = "";
            for(int id = 0; id < 15; id++)
            {
                sprintf(interactmatrix_string, "%s%lf,", interactmatrix_string, data.interactMatrix[id]);
            }
            sprintf(interactmatrix_string, "%s%lf", interactmatrix_string, data.interactMatrix[15]);
            text_interactmatrix.set(interactmatrix_string);
        }

        //BoundaryPartID
        if(data.strucCreator && data.strucCreator->struc)
        {
            pugi::xml_node node_boundary = node_guisettings.child("Boundary_PartIDs");
            if(!node_boundary) node_boundary = node_guisettings.append_child("Boundary_PartIDs");
            pugi::xml_text text_boundary = node_boundary.text();
            char boundary_string[10000] = "";
            for(int id = 0; id < data.strucCreator->struc->partList.size(); id++)
            {
                pPart part = data.strucCreator->struc->partList[id];
                if(part && part->atBoundary){
                    sprintf(boundary_string, "%s%d,", boundary_string, part->partID);
                }
            }
            if(strlen(boundary_string) > 0){
                boundary_string[strlen(boundary_string) - 1] = '\0';
            }
            text_boundary.set(boundary_string);
        }

        //Camera
        {
            pugi::xml_node node_camera = node_guisettings.child("Camera");
            if(!node_camera) node_camera = node_guisettings.append_child("Camera");

            //worldAxesMat
            {
                pugi::xml_node node_worldAxesMat = node_camera.child("worldAxesMat");
                if(!node_worldAxesMat) node_worldAxesMat = node_camera.append_child("worldAxesMat");
                pugi::xml_text text_worldAxesMat = node_worldAxesMat.text();
                char worldAxesMat_string[256] = "";
                for(int id = 0; id < 15; id++)
                {
                    sprintf(worldAxesMat_string, "%s%lf,", worldAxesMat_string, data.strucCreator->worldAxesMat[id]);
                }
                sprintf(worldAxesMat_string, "%s%lf", worldAxesMat_string, data.strucCreator->worldAxesMat[15]);
                text_worldAxesMat.set(worldAxesMat_string);
            }

            //worldMatrix
            {
                pugi::xml_node node_worldMatrix = node_camera.child("worldMatrix");
                if(!node_worldMatrix) node_worldMatrix = node_camera.append_child("worldMatrix");
                pugi::xml_text text_worldMatrix = node_worldMatrix.text();
                char worldMatrix_string[256] = "";
                for(int id = 0; id < 15; id++)
                {
                    sprintf(worldMatrix_string, "%s%lf,", worldMatrix_string, data.strucCreator->worldMatrix[id]);
                }
                sprintf(worldMatrix_string, "%s%lf", worldMatrix_string, data.strucCreator->worldMatrix[15]);
                text_worldMatrix.set(worldMatrix_string);
            }
        }
    }
}

void XMLIO::XMLWriter_Output(pugi::xml_node &xmlroot, boost::filesystem::path &xmlFileName_path, XMLData &data)
{
    pugi::xml_node xml_output = xmlroot.child("Output");
    if(!xml_output) xml_output = xmlroot.append_child("Output");
    //  ./name_data
    {

        string file_name = xmlFileName_path.filename().stem().string();
        string data_path = xmlFileName_path.parent_path().string();
        data_path += "/" + file_name + "_data";
        //Surface
        if(data.strucCreator->crossMeshCreator && data.strucCreator->crossMeshCreator->referenceSurface)
        {
            string surface_path = data_path + "/" + file_name + "_Surface.obj";
            data.strucCreator->crossMeshCreator->referenceSurface->WriteOBJModel(surface_path.c_str());

            shared_ptr<CrossMesh> crossMesh = data.strucCreator->crossMeshCreator->crossMesh;
            if(crossMesh && data.varList->get<bool>("output_mitsuba"))
            {
                string surface_wire_path = data_path + "/" + file_name + "_CrossMeshWire.obj";
                shared_ptr<Cross> cross;
                shared_ptr<Part> part = make_shared<Part>(cross, data.varList);
                shared_ptr<PolyMesh> polyMesh = make_shared<PolyMesh>(data.varList);
                for(int id = 0; id < crossMesh->crossList.size(); id++){
                    shared_ptr<_Polygon> poly = make_shared<_Polygon>(*((_Polygon *)crossMesh->crossList[id].get()));
                    polyMesh->polyList.push_back(poly);
                }
                polyMesh->removeDuplicatedVertices();
                part->polyMesh = polyMesh;
                polyMesh->TranslateMesh(Vector3f(0, -data.varList->get<float>("ground_height"), 0));
                part->WriteOBJWireFrameModel(surface_wire_path.c_str());
            }

            pugi::xml_node struc_node = xml_output.child("Structure");
            if(!struc_node) struc_node = xml_output.append_child("Structure");
            pugi::xml_attribute struc_attr = struc_node.attribute("path");
            if(!struc_attr) struc_attr = struc_node.append_attribute("path");
            string str_attr = file_name + "_" + "data" + "/" + file_name + "_Surface.obj";
            struc_attr.set_value(str_attr.c_str());
        }

        //CrossMesh
        if(data.strucCreator->crossMeshCreator && data.strucCreator->crossMeshCreator->crossMesh)
        {
            string crossMesh_path = data_path + "/" + file_name + "_CrossMesh.obj";
            shared_ptr<CrossMesh> crossMesh = make_shared<CrossMesh>(*data.strucCreator->crossMeshCreator->crossMesh);
            crossMesh ->TranslateMesh(Vector3f(0, -data.varList->get<float>("ground_height"), 0));
            crossMesh->WriteOBJModel(crossMesh_path.c_str());

            pugi::xml_node crossMesh_node = xml_output.child("CrossMesh");
            if(!crossMesh_node) crossMesh_node = xml_output.append_child("CrossMesh");
            pugi::xml_attribute cross_attr = crossMesh_node.attribute("path");
            if(!cross_attr) cross_attr = crossMesh_node.append_attribute("path");
            string str_attr = file_name + "_" + "data" + "/" + file_name + "_CrossMesh.obj";
            cross_attr.set_value(str_attr.c_str());
        }

        //PartGeometry
        if(data.strucCreator->struc)
        {
            string part_path = data_path + "/" + "PartGeometry";
            if(!boost::filesystem::exists(part_path))
            {
                boost::filesystem::create_directories(part_path);
            }

            data.strucCreator->struc->WriteStructure(part_path.c_str());
            if(data.varList->get<bool>("output_mitsuba")) {
                data.strucCreator->struc->WriteStructureWireFrame(part_path.c_str());
            }

            pugi::xml_node partGeo_node = xml_output.child("PartGeometry");
            if(!partGeo_node) partGeo_node = xml_output.append_child("PartGeometry");
            pugi::xml_text partGeo_text = partGeo_node.text();
            string partGeo_names = "";
            string relative_path = file_name + "_" + "data" + "/" + "PartGeometry/";
            for (boost::filesystem::directory_entry &part_name : boost::filesystem::directory_iterator(part_path))
            {
                partGeo_names += relative_path + part_name.path().filename().string() + "\n";
            }
            partGeo_text.set(partGeo_names.c_str());
        }
    }
}

void XMLIO::XMLWriter_Mitsuba(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path, XMLData &data)
{
    string file_name = xmlFileName_path.filename().stem().string();
    string data_path = xmlFileName_path.parent_path().string();
    data_path += "/" + file_name + "_data";

    string mitsuba_path = data_path + "/mitsuba.xml";
    pugi::xml_document mitsuba_doc;

    MitsubaWriter mitsuba_writter(data.varList);

    mitsuba_writter.mitsuba_sensor = mitsuba_sensor;

    mitsuba_writter.scene_settings(mitsuba_doc, data);
    mitsuba_doc.save_file(mitsuba_path.c_str());

    xml_root.remove_child("Mitsuba");
    pugi::xml_node mitsuba_node = xml_root.append_child("Mitsuba");
    pugi::xml_node mitsubadata_node = mitsuba_node.append_child("MitsubaData");
    pugi::xml_attribute mitsubadata_attr = mitsubadata_node.append_attribute("path");
    string relative_Mitsuba_path = file_name + "_data" + "/mitsuba.xml";
    mitsubadata_attr.set_value(relative_Mitsuba_path.c_str());
}

void XMLIO::XMLWriter_PartGeoData(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path, XMLData &data)
{
    string file_name = xmlFileName_path.filename().stem().string();
    string data_path = xmlFileName_path.parent_path().string();
    data_path += "/" + file_name + "_data";

    xml_root.remove_child("PartGeoData");
    pugi::xml_node partGeo_node = xml_root.append_child("PartGeoData");

    //CrossData
    if(data.strucCreator->crossMeshCreator && data.strucCreator->crossMeshCreator->crossMesh)
    {
        string CrossData_path = data_path + "/CrossMesh.xml";
        {
            pugi::xml_document crossDoc;
            pugi::xml_node cross_root = crossDoc.append_child("Documents");
            xml_crossdata = cross_root;

            //CrossData
            pugi::xml_node crossdata_node = cross_root.append_child("CrossData");
            int kd = 0;
            for(auto cross : data.strucCreator->crossMeshCreator->crossMesh->crossList)
            {
                pugi::xml_node cross_node = crossdata_node.append_child("Cross");
                //cross_node.append_attribute("id").set_value(cross->crossID);
                cross_node.append_attribute("id").set_value(kd++);
                Vector3f normal = cross->normal;
                Vector3f center = cross->center;
                char str_normal[256];
                sprintf(str_normal, "(%f,%f,%f)", normal[0], normal[1], normal[2]);
                cross_node.append_attribute("Normal").set_value(str_normal);
                char str_center[256];
                sprintf(str_center, "(%f,%f,%f)", center[0], center[1], center[2]);
                cross_node.append_attribute("Center").set_value(str_center);
                for(int jd = 0; jd < cross->oriPoints.size(); jd++)
                {
                    OrientPoint *pt = cross->oriPoints[jd].get();
                    pugi::xml_node oript_node = cross_node.append_child("OrientPoint");
                    oript_node.append_attribute("id").set_value(jd);

                    pugi::xml_node angle_node = oript_node.append_child("Angle");
                    angle_node.append_attribute("Radian").set_value(pt->rotation_angle);

                    pugi::xml_node normal_node = oript_node.append_child("Normal");
                    pugi::xml_attribute normal_attri = normal_node.append_attribute("XYZ");
                    char XYZ[256];
                    sprintf(XYZ, "(%f, %f, %f)", pt->normal[0], pt->normal[1], pt->normal[2]);
                    normal_attri.set_value(XYZ);

                    pugi::xml_node point_node = oript_node.append_child("Point");
                    pugi::xml_attribute point_attri = point_node.append_attribute("XYZ");
                    sprintf(XYZ, "(%f, %f, %f)", pt->point[0], pt->point[1], pt->point[2]);
                    point_attri.set_value(XYZ);

                    pugi::xml_node vertex_node = oript_node.append_child("Vertex");
                    pugi::xml_attribute vertex_attri = vertex_node.append_attribute("XYZ");
                    sprintf(XYZ, "(%f, %f, %f)", cross->vers[jd].pos[0], cross->vers[jd].pos[1], cross->vers[jd].pos[2]);
                    vertex_attri.set_value(XYZ);

                    point_node = oript_node.append_child("Rotation_Axis");
                    point_attri = point_node.append_attribute("XYZ");
                    sprintf(XYZ, "(%f, %f, %f)", pt->rotation_axis[0], pt->rotation_axis[1], pt->rotation_axis[2]);
                    point_attri.set_value(XYZ);

                    point_node = oript_node.append_child("Rotation_Base");
                    point_attri = point_node.append_attribute("XYZ");
                    sprintf(XYZ, "(%f, %f, %f)", pt->rotation_base[0], pt->rotation_base[1], pt->rotation_base[2]);
                    point_attri.set_value(XYZ);

                    pugi::xml_node neighbor_id = oript_node.append_child("Neighbor");
                    if(cross->neighbors[jd].lock())
                        neighbor_id.append_attribute("id").set_value(cross->neighbors[jd].lock()->crossID);
                    else
                        neighbor_id.append_attribute("id").set_value(-1);
                }
            }

            crossDoc.save_file(CrossData_path.c_str());
            crossDoc.remove_child("Documents");
        }

        pugi::xml_node crossdata_node = partGeo_node.append_child("CrossData");
        pugi::xml_attribute crossdata_attr = crossdata_node.append_attribute("path");
        string relative_CrossData_path = file_name + "_data" + "/CrossMesh.xml";
        crossdata_attr.set_value(relative_CrossData_path.c_str());
    }
}


//**************************************************************************************//
//                                  XML Reader
//**************************************************************************************//

bool XMLIO::XMLReader(string xmlFileName, XMLData &data)
{

    //load xmlfile
    if(!xmldoc.load_file(xmlFileName.c_str()))
        return false;
    pugi::xml_node xml_root = xmldoc.child("Documents");

    if (xml_root)
    {
        //0) allocate memorry
        data.varList = make_shared<InputVarList>();
        data.strucCreator = make_shared<StrucCreator>(data.varList);

        //1) read all gui settings
        InitVar(data.varList.get());
        XMLReader_GUISettings(xml_root, data);

        //2) construct the cross mesh
        boost::filesystem::path xmlPathBoost(xmlFileName);
        data.varList->filename = xmlPathBoost.stem().string();
        string geomDataFolder = boost::filesystem::path(xmlFileName).parent_path().string();
        if (data.varList->get<bool>("texturedModel") == false)
        {
            //2.1) if the model does not have texture
            pugi::xml_node crossMeshNode = xml_root.child("Output").child("CrossMesh");
            if (crossMeshNode)
            {
                string path = geomDataFolder + "/" + crossMeshNode.attribute("path").as_string();
                std::cout << "Read File...\t:" << path << std::endl;
                if (!data.strucCreator->LoadSurface(path.c_str())) return false;
                data.strucCreator->CreateStructure(true, false, data.interactMatrix, true);
            }
        }
        else
        {
            //2.2) if the model has texture
            pugi::xml_node surfaceMesh = xml_root.child("Output").child("Structure");
            if (surfaceMesh)
            {
                string path = geomDataFolder + "/" + surfaceMesh.attribute("path").as_string();
                std::cout << "Read File...\t:" << path << std::endl;
                if (!data.strucCreator->LoadSurface(path.c_str())) return false;
                data.strucCreator->CreateStructure(true, true, data.interactMatrix, true);
            }
        }

        //3) update each cross
        if (data.strucCreator->struc != nullptr) {
            if (xml_root)
            {
                XMLReader_PartGeoData(xml_root, geomDataFolder, data);
                XMLReader_Boundary(xml_root, data);
            }
        }
    }

    return true;
}

void XMLIO::XMLReader_GUISettings(pugi::xml_node &xml_root, XMLData &data)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    //guisettings
    {
        if(!node_guisettings) return;

        //para basic
        InputVarManager varOrganizer;
        {
            vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Basic");
            for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_guisettings);
        }


        //para shapeop
        {
            pugi::xml_node node_shapeop = node_guisettings.child("ShapeOp_Settings");
            if(node_shapeop){
                //Para_ShapeOp
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_ShapeOp");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_shapeop);
            }
        }


        //para mitsuba
        {
            pugi::xml_node node_mitsuba = node_guisettings.child("Para_Mitsuba");
            if(node_mitsuba){
                //Para_Mitsuba
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Mitsuba");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_mitsuba);
            }
        }

        //para contactgraph
        {
            pugi::xml_node node_contactgraph = node_guisettings.child("Para_ContactGraph");
            if(node_contactgraph){
                //Para_ContactGraph
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_ContactGraph");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_contactgraph);
            }
        }

        //para crossmesh
        {
            pugi::xml_node node_crossmesh = node_guisettings.child("Para_CrossMesh");
            if(node_crossmesh){
                //Para_CrossMesh
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_CrossMesh");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_crossmesh);
            }
        }

        //para opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Opt");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        //Para_Assembly
        {
            pugi::xml_node node = node_guisettings.child("Para_Assembly");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Assembly");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        //Para_Support
        {
            pugi::xml_node node = node_guisettings.child("Para_Support");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Support");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        //Para_Opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->findSeries("Para_Opt");
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        };

        //pickPartIDs
        {
            pugi::xml_node node_pickPartIDs = node_guisettings.child("pickPartIDs");
            data.pickPartIDs.clear();
            if(node_pickPartIDs)
            {
                pugi::xml_text text_pickPartIDs = node_pickPartIDs.text();
                string str_text = text_pickPartIDs.as_string();
                vector<string> split_strs;
                split(split_strs, str_text, boost::is_any_of(","));
                for(string split_str : split_strs)
                {
                    if(split_str != "")
                    {
                        int PartID = std::stoi(split_str, nullptr, 10);
                        data.pickPartIDs.push_back(PartID);
                    }
                }
            }
        }

        //interactMatrix
        {
            pugi::xml_node node_interactmatrix = node_guisettings.child("Interact_Matrix");
            if(node_interactmatrix)
            {
                pugi::xml_text text_interactmatrix = node_interactmatrix.text();
                auto interactmatrix_string = text_interactmatrix.get();
                sscanf(interactmatrix_string, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                       &data.interactMatrix[0], &data.interactMatrix[1], &data.interactMatrix[2], &data.interactMatrix[3],
                       &data.interactMatrix[4], &data.interactMatrix[5], &data.interactMatrix[6], &data.interactMatrix[7],
                       &data.interactMatrix[8], &data.interactMatrix[9], &data.interactMatrix[10], &data.interactMatrix[11],
                       &data.interactMatrix[12], &data.interactMatrix[13], &data.interactMatrix[14], &data.interactMatrix[15]);
            }
        }

        //Camera
        {
            pugi::xml_node node_camera = node_guisettings.child("Camera");
            if(node_camera)
            {
                //worldAxesMat
                {
                    pugi::xml_node node_worldAxesMat = node_camera.child("worldAxesMat");
                    if(node_worldAxesMat)
                    {
                        pugi::xml_text text_worldAxesMat = node_worldAxesMat.text();
                        auto worldAxesMat_string = text_worldAxesMat.get();
                        double *worldAxesMat = data.strucCreator->worldAxesMat;
                        sscanf(worldAxesMat_string, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                               worldAxesMat    , worldAxesMat + 1,  worldAxesMat + 2,  worldAxesMat + 3,
                               worldAxesMat + 4, worldAxesMat + 5,  worldAxesMat + 6,  worldAxesMat + 7,
                               worldAxesMat + 8, worldAxesMat + 9,  worldAxesMat + 10, worldAxesMat + 11,
                               worldAxesMat + 12,worldAxesMat + 13, worldAxesMat + 14, worldAxesMat + 15);
                    }
                }
                //worldMatrix
                {
                    pugi::xml_node node_worldMatrix = node_camera.child("worldMatrix");
                    if(node_worldMatrix)
                    {
                        pugi::xml_text text_worldMatrix = node_worldMatrix.text();
                        auto worldMatrix_string = text_worldMatrix.get();
                        double *worldMatrix = data.strucCreator->worldMatrix;
                        sscanf(worldMatrix_string, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                               worldMatrix    , worldMatrix + 1,  worldMatrix + 2,  worldMatrix + 3,
                               worldMatrix + 4, worldMatrix + 5,  worldMatrix + 6,  worldMatrix + 7,
                               worldMatrix + 8, worldMatrix + 9,  worldMatrix + 10, worldMatrix + 11,
                               worldMatrix + 12,worldMatrix + 13, worldMatrix + 14, worldMatrix + 15);
                    }
                }
            }
        }
    }
}

void XMLIO::XMLReader_PartGeoData(pugi::xml_node &xml_root, string &xmlFileName_path, XMLData &data)
{
    pugi::xml_node partGeo_node = xml_root.child("PartGeoData");
    if(partGeo_node)
    {
        //CrossData
        if(data.strucCreator->crossMeshCreator && data.strucCreator->crossMeshCreator->crossMesh)
        {
            pugi::xml_node crossdata_node = partGeo_node.child("CrossData");
            if(crossdata_node)
            {
                string CrossData_path = crossdata_node.attribute("path").as_string();
                CrossData_path = xmlFileName_path + "/" + CrossData_path;
                pugi::xml_document crossDoc;
                crossDoc.load_file(CrossData_path.c_str());
                crossdata_node = crossDoc.child("Documents").child("CrossData");
                if(crossdata_node)
                {
                    for(pugi::xml_node cross_node: crossdata_node.children())
                    {
                        int crossID = cross_node.attribute("id").as_int();
                        Cross *cross = data.strucCreator->crossMeshCreator->crossMesh->crossList[crossID].get();
                        if(cross == NULL) continue;
                        for(pugi::xml_node ori_node : cross_node.children())
                        {
                            int oriPtID = ori_node.attribute("id").as_int();
                            if(oriPtID < 0 || oriPtID >= (int) cross->oriPoints.size())
                                continue;

                            if(cross->oriPoints[oriPtID] == nullptr)
                                continue;

                            OrientPoint *oript = cross->oriPoints[oriPtID].get();
                            pugi::xml_node angle_node = ori_node.child("Angle");

                            if(angle_node) cross->oriPoints[oriPtID]->rotation_angle = angle_node.attribute("Radian").as_double();
                            oript->normal = cross->RotateNormal(oript->rotation_base, oript->rotation_axis, oript->rotation_angle);
                        }
                    }

                    data.strucCreator->CreateStructure(false, false, data.interactMatrix, false);
                }
            }
        }
    }
}

void XMLIO::XMLReader_Boundary(pugi::xml_node &xml_root, XMLData &data)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");

    {
        pugi::xml_node node_boundary = node_guisettings.child("Boundary_PartIDs");
        if(node_boundary && data.strucCreator && data.strucCreator->struc)
        {

            //1) clear all boundary maker
            for(pPart part : data.strucCreator->struc->partList)
            {
                if(part) part->atBoundary = false;
            }

            //2) read the boundary part
            pugi::xml_text text_boundary = node_boundary.text();
            vector<string> str_partIDs;
            string text_str = text_boundary.get();
            boost::split(str_partIDs, text_str, boost::is_any_of(","));

            //3) assign boundary marker
            for(int id = 0; id < str_partIDs.size(); id++)
            {
                if(str_partIDs[id] == "") continue;
                int partID = std::atoi(str_partIDs[id].c_str());
                if(partID < 0 || partID >= data.strucCreator->struc->partList.size())
                {
                    continue;
                }
                else
                {
                    pPart part = data.strucCreator->struc->partList[partID];
                    if(part) part->atBoundary = true;
                }
            }

        }
    }
}



//**************************************************************************************//
//                                  Mitsuba Writer
//**************************************************************************************//


void MitsubaWriter::scene_settings(pugi::xml_node &xml_root, XMLData &data)
{
    //scene
    pugi::xml_node scene_node;
    change_attribute(xml_root, scene_node, "scene", "version", "0.5.0");
    float mitsubaPartYRotation = data.varList->get<float>("mitsuba_PartYRotation");
    float mitsubaCameraRotation = data.varList->get<float>("mitsuba_CameraRotation") / 180 * M_PI;
    float mitsubaCameraDistance = data.varList->get<float>("mitsuba_CameraDistance");
    float mitsubaSunTime = data.varList->get<float>("mitsuba_SunTime");
    float mitsubaSunStrength = data.varList->get<float>("mitsuba_SunStrength");

    int   mitsuba_width =   data.varList->get<int>("mitsuba_width");
    int   mitsuba_height =   data.varList->get<int>("mitsuba_height");
    int   mitsuba_sample =   data.varList->get<int>("mitsuba_sample");

    //integrator
    {
        pugi::xml_node integrator_node;
        change_attribute(scene_node, integrator_node, "integrator", "type", "path");
    }

//    if(mitsuba_sensor)
//    {
//        scene_node.append_copy(mitsuba_sensor);
//    }
//    else
    //camera
    {
        pugi::xml_node camera_node;
        change_attribute(scene_node, camera_node, "sensor", "type", "perspective");

        //camera length
        {
            pugi::xml_node camera_length_node;
            change_attribute(camera_node, camera_length_node, "string", "name", "focalLength");
            change_attribute(camera_length_node, "value", "50mm");
        }

        //transform
        {
            pugi::xml_node camera_transform;
            change_attribute(camera_node, camera_transform, "transform", "name", "toWorld");
            pugi::xml_node camera_transform_lookat;
            Vector3f origin(0, mitsubaCameraDistance * std::sin(mitsubaCameraRotation), mitsubaCameraDistance * std::cos(mitsubaCameraRotation));
            Vector3f up = Vector3f(-1, 0, 0) CROSS origin; up /= len(up);
            std::stringstream scout;
            change_attribute(camera_transform, camera_transform_lookat, "lookat", "target", "0, 0, 0");
            scout << origin.x << ", " << origin.y << ", " << origin.z;
            change_attribute(camera_transform_lookat, "origin", scout.str().c_str());
            scout.str("");
            scout << up.x << ", " << up.y << ", " << up.z;
            change_attribute(camera_transform_lookat, "up", scout.str().c_str());
        }
        //ldsampler
        {
            pugi::xml_node camera_ldsampler;
            change_attribute(camera_node, camera_ldsampler, "sampler", "type", "ldsampler");
            pugi::xml_node camera_transform_sampleCount;
            change_attribute(camera_ldsampler, camera_transform_sampleCount, "integer", "name", "sampleCount");
            change_attribute(camera_transform_sampleCount, "value", std::to_string(mitsuba_sample));
        }
        //hdr
        {
            pugi::xml_node camera_hdrfilm;
            change_attribute(camera_node, camera_hdrfilm, "film", "type", "ldrfilm");

            pugi::xml_node camera_hdrfilm_banner;
            change_attribute(camera_hdrfilm, camera_hdrfilm_banner, "boolean", "name", "banner");
            change_attribute(camera_hdrfilm_banner, "value", "false");

            pugi::xml_node camera_hdrfilm_height;
            change_attribute(camera_hdrfilm, camera_hdrfilm_height, "integer", "name", "height");
            change_attribute(camera_hdrfilm_height, "value", std::to_string(mitsuba_height));

            pugi::xml_node camera_hdrfilm_width;
            change_attribute(camera_hdrfilm, camera_hdrfilm_width, "integer", "name", "width");
            change_attribute(camera_hdrfilm_width, "value", std::to_string(mitsuba_width));

            pugi::xml_node camera_hdrfilm_pixelFormat;
            change_attribute(camera_hdrfilm, camera_hdrfilm_pixelFormat, "string", "name", "pixelFormat");
            change_attribute(camera_hdrfilm_pixelFormat, "value", "rgb");
        }
    }

//    //emitter
////    {
////        pugi::xml_node emitter_node;
////        change_attribute(scene_node, emitter_node, "emitter", "type", "sunsky");
////        pugi::xml_node emitter_scale;
////        change_attribute(emitter_node, emitter_scale, "float", "name", "scale");
////        change_attribute(emitter_scale, "value", std::to_string(mitsubaSunStrength));
////        pugi::xml_node emitter_hour;
////        change_attribute(emitter_node, emitter_hour, "float", "name", "hour");
////        change_attribute(emitter_hour, "value", std::to_string(mitsubaSunTime));
////    }

    {
        string emitter_str = "<shape type=\"rectangle\">\n"
                             "\t\t<emitter type=\"area\">\n"
                             "\t\t\t<spectrum name=\"radiance\" value=\"1.2\" />\n"
                             "\t\t</emitter>\n"
                             "\t\t<transform name=\"toWorld\">\n"
                             "\t\t\t<rotate x=\"1\" angle=\"90\" />\n"
                             "\t\t\t<scale x=\"4\" y=\"4\" z=\"4\" />\n"
                             "\t\t\t<translate y=\"3\" z=\"0\" />\n"
                             "\t\t</transform>\n"
                             "\t</shape>\n"
                             "\t<shape type=\"rectangle\">\n"
                             "\t\t<emitter type=\"area\">\n"
                             "\t\t\t<spectrum name=\"radiance\" value=\"1.2\" />\n"
                             "\t\t</emitter>\n"
                             "\t\t<transform name=\"toWorld\">\n"
                             "\t\t\t<rotate y=\"1\" angle=\"45\" />\n"
                             "\t\t\t<scale x=\"4\" y=\"4\" z=\"4\" />\n"
                             "\t\t\t<translate x=\"-3.5\" z=\"-3.5\" />\n"
                             "\t\t</transform>\n"
                             "\t</shape>\n"
                             "\t<shape type=\"rectangle\">\n"
                             "\t\t<emitter type=\"area\">\n"
                             "\t\t\t<spectrum name=\"radiance\" value=\"2.5\" />\n"
                             "\t\t</emitter>\n"
                             "\t\t<transform name=\"toWorld\">\n"
                             "\t\t\t<rotate y=\"1\" angle=\"150\" />\n"
                             "\t\t\t<scale x=\"3\" y=\"3\" z=\"1\" />\n"
                             "\t\t\t<translate x=\"-1.5\" z=\"3\" />\n"
                             "\t\t</transform>\n"
                             "\t</shape>";
        pugi::xml_document doc;
        doc.load_string(emitter_str.c_str());
        pugi::xml_node root = doc.root();
        for(pugi::xml_node emitter_node = root.child("shape"); emitter_node; emitter_node = emitter_node.next_sibling("shape"))
        {
//            string type = emitter_node.attribute("type").as_string();
//            if(type == "sunsky")
//            {
//                for(pugi::xml_node float_node = emitter_node.child("float"); float_node; float_node = float_node.next_sibling("float"))
//                {
//                    string name = float_node.attribute("name").as_string();
//                    std::cout << name << std::endl;
//                    if(name == "sunScale")
//                        float_node.attribute("value").set_value(mitsubaSunStrength);
//                }
//            }
            scene_node.append_copy(emitter_node);
        }
    }

    //material
    {
        //boundary
        {
            pugi::xml_node mat_boundary_node;
            change_attribute(scene_node, mat_boundary_node, "bsdf", "type", "roughdiffuse");
            change_attribute(mat_boundary_node, "id", "mat_boundary");

            {
                pugi::xml_node rgb_node;
                change_attribute(mat_boundary_node, rgb_node, "rgb", "name", "reflectance");
                change_attribute(rgb_node, "value", "0.3, 0.3, 0.3");
            }

            {
                pugi::xml_node alpha_node;
                change_attribute(mat_boundary_node, alpha_node, "float", "name", "alpha");
                change_attribute(alpha_node, "value", "1");
            }
        }

        //part
        {
            pugi::xml_node mat_part_node;
            change_attribute(scene_node, mat_part_node, "bsdf", "type", "roughdiffuse");
            change_attribute(mat_part_node, "id", "mat_part");
            pugi::xml_node rgb_node;
            change_attribute(mat_part_node, rgb_node, "rgb", "name", "reflectance");
            change_attribute(rgb_node, "value", "1, 1, 1");

            {
                pugi::xml_node alpha_node;
                change_attribute(mat_part_node, alpha_node, "float", "name", "alpha");
                change_attribute(alpha_node, "value", "1");
            }
        }

        //ground
        {
            pugi::xml_node mat_ground_node;
            change_attribute(scene_node, mat_ground_node, "bsdf", "type", "roughdiffuse");
            change_attribute(mat_ground_node, "id", "mat_ground");
            pugi::xml_node rgb_node;
            change_attribute(mat_ground_node, rgb_node, "rgb", "name", "reflectance");
            change_attribute(rgb_node, "value", "1, 1, 1");

            {}
        }

        //wireframe
        {
            pugi::xml_node mat_wireframe_node;
            change_attribute(scene_node, mat_wireframe_node, "bsdf", "type", "diffuse");
            change_attribute(mat_wireframe_node, "id", "mat_wireframe");
            pugi::xml_node rgb_node;
            change_attribute(mat_wireframe_node, rgb_node, "rgb", "name", "reflectance");
            change_attribute(rgb_node, "value", "0, 0, 0");

            {
                pugi::xml_node alpha_node;
                change_attribute(mat_wireframe_node, alpha_node, "float", "name", "alpha");
                change_attribute(alpha_node, "value", "1");
            }
        }
    }

    //shape
    {
        if(myStrucCreator->struc)
        {
            shared_ptr<Struc> struc = myStrucCreator->struc;
            //part
            {
                pugi::xml_node part_node;
                change_attribute(scene_node, part_node, "shape", "type", "obj");
                char objFileName[FILE_NAME_LENGTH];
                sprintf(objFileName, "PartGeometry/Parts.obj");
                pugi::xml_node part_filename_node;
                change_attribute(part_node, part_filename_node, "string", "name", "filename");
                change_attribute(part_filename_node, "value", objFileName);

                pugi::xml_node part_maxSmoothAngle_node;
                change_attribute(part_node, part_maxSmoothAngle_node, "float", "name", "maxSmoothAngle");
                change_attribute(part_maxSmoothAngle_node, "value", "20.0");

                pugi::xml_node transform;
                change_attribute(part_node, transform, "transform", "name", "toWorld");

                pugi::xml_node transform_rotate;
                change_attribute(transform, transform_rotate, "rotate", "y", "1");
                change_attribute(transform_rotate, "angle", std::to_string(mitsubaPartYRotation));

                pugi::xml_node part_ref_node;
                change_attribute(part_node, part_ref_node, "ref", "id", "mat_part");
            }

            //partWire
            {
                pugi::xml_node part_node;
                change_attribute(scene_node, part_node, "shape", "type", "obj");
                char objFileName[FILE_NAME_LENGTH];
                sprintf(objFileName, "PartGeometry/PartsWire.obj");

                pugi::xml_node part_filename_node;
                change_attribute(part_node, part_filename_node, "string", "name", "filename");
                change_attribute(part_filename_node, "value", objFileName);

                pugi::xml_node transform;
                change_attribute(part_node, transform, "transform", "name", "toWorld");

                pugi::xml_node transform_rotate;
                change_attribute(transform, transform_rotate, "rotate", "y", "1");
                change_attribute(transform_rotate, "angle", std::to_string(mitsubaPartYRotation));

                pugi::xml_node part_ref_node;
                change_attribute(part_node, part_ref_node, "ref", "id", "mat_wireframe");
            }

            //boundary
            {
                pugi::xml_node part_node;
                change_attribute(scene_node, part_node, "shape", "type", "obj");
                pugi::xml_node part_filename_node;
                change_attribute(part_node, part_filename_node, "string", "name", "filename");
                change_attribute(part_filename_node, "value", "PartGeometry/Boundary.obj");

                pugi::xml_node part_maxSmoothAngle_node;
                change_attribute(part_node, part_maxSmoothAngle_node, "float", "name", "maxSmoothAngle");
                change_attribute(part_maxSmoothAngle_node, "value", "20.0");

                pugi::xml_node transform;
                change_attribute(part_node, transform, "transform", "name", "toWorld");

                pugi::xml_node transform_rotate;
                change_attribute(transform, transform_rotate, "rotate", "y", "1");
                change_attribute(transform_rotate, "angle", std::to_string(mitsubaPartYRotation));


                pugi::xml_node part_ref_node;
                change_attribute(part_node, part_ref_node, "ref", "id", "mat_boundary");
            }

            //ground
            {
                pugi::xml_node ground_node;
                change_attribute(scene_node, ground_node, "shape", "type", "rectangle");

                //transform
                {
                    pugi::xml_node ground_transform;
                    change_attribute(ground_node, ground_transform, "transform", "name", "toWorld");

                    pugi::xml_node ground_transform_scale;
                    change_attribute(ground_transform, ground_transform_scale, "scale", "value", "1000");

                    pugi::xml_node ground_transform_rotate;
                    change_attribute(ground_transform, ground_transform_rotate, "rotate", "x", "1");
                    change_attribute(ground_transform_rotate, "angle", "-90");

                    struc->ComputeGroundY(false);
                    pugi::xml_node ground_transform_translate;
                    change_attribute(ground_transform, ground_transform_rotate, "translate", "y", std::to_string(0));
                }

                //material
                pugi::xml_node ground_ref_node;
                change_attribute(ground_node, ground_ref_node, "ref", "id", "mat_ground");
            }
        }
    }


}

void MitsubaWriter::change_attribute(pugi::xml_node &root, pugi::xml_node &node, string str_name, string str_attr, string str_value)
{
    node = root.append_child(str_name.c_str());
    pugi::xml_attribute attr = node.attribute(str_attr.c_str());
    if(!attr) attr = node.append_attribute(str_attr.c_str());
    attr.set_value(str_value.c_str());
    return;
}

void MitsubaWriter::change_attribute(pugi::xml_node &node, string str_attr, string str_value)
{
    pugi::xml_attribute attr = node.attribute(str_attr.c_str());
    if(!attr) attr = node.append_attribute(str_attr.c_str());
    attr.set_value(str_value.c_str());
    return;
}