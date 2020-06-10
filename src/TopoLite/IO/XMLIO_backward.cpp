//
// Created by ziqwang on 25.05.20.
//

#include "XMLIO_backward.h"
#include "CrossMesh/CrossMeshCreator.h"

//**************************************************************************************//
//                                  XML Reader
//**************************************************************************************//

bool XMLIO_backward::XMLReader(const string xmlFileName, IOData &data)
{

    pugi::xml_document xmldoc;
    // load xmlfile
    if(!xmldoc.load_file(xmlFileName.c_str()))
        return false;

    path file_path(xmlFileName);
    pugi::xml_node xml_root = xmldoc.child("Documents");

    if (xml_root)
    {
        // 0) allocate memorry
        data.varList = make_shared<InputVarList>();
//        data.strucCreator = make_shared<StrucCreator>(data.varList);

        // 1) read all gui settings
        InitVar_backward(data.varList.get());
        XMLReader_GUISettings(xml_root, data);

        // 2) read reference surface
        XMLReader_ReferenceSurface(xml_root, file_path.parent_path().string(), data);

        // 3) read cross mesh
        XMLReader_CrossMesh(xml_root, file_path.parent_path().string(), data);

        // 4) read cross mesh boundary
        XMLReader_Boundary(xml_root, data);

        // 5) read pattern mesh
        if(data.reference_surface)
        {
            shared_ptr<CrossMeshCreator<double>> crossMeshCreator;
            crossMeshCreator = std::make_shared<CrossMeshCreator<double>>(data.varList);
            crossMeshCreator->updatePatternMesh();
            data.pattern_mesh = crossMeshCreator->pattern2D->getPolyMesh();
        }

        //6) texturedMat and boundary_crossIDs
        if(data.reference_surface){
            shared_ptr<CrossMeshCreator<double>> crossMeshCreator;
            Eigen::Matrix4d interactMat = toEigenMatrix(data.interactMatrix);
            crossMeshCreator = std::make_shared<CrossMeshCreator<double>>(data.varList);
            crossMeshCreator->setReferenceSurface(data.reference_surface);
            data.varList->add(crossMeshCreator->computeTextureMat_backwards_compatible(interactMat), "texturedMat", "");
        }
        else{
            data.varList->add(Eigen::Matrix4d::Identity(), "texturedMat", "");
        }

        data.varList->add(data.boundary_crossIDs, "boundary_crossIDs", "");
    }

    return true;
}

void XMLIO_backward::XMLReader_GUISettings(pugi::xml_node &xml_root, IOData &data)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    // guisettings
    {
        if(!node_guisettings) return;

        // para basic
        InputVarManager varOrganizer;
        {
            vector<shared_ptr<InputVar>> series = data.varList->varLists;
            for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_guisettings);
        }

        // para shapeop
        {
            pugi::xml_node node_shapeop = node_guisettings.child("ShapeOp_Settings");
            if(node_shapeop){
                // Para_ShapeOp
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_shapeop);
            }
        }

        // para mitsuba
        {
            pugi::xml_node node_mitsuba = node_guisettings.child("Para_Mitsuba");
            if(node_mitsuba){
                // Para_Mitsuba
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_mitsuba);
            }
        }

        // para contactgraph
        {
            pugi::xml_node node_contactgraph = node_guisettings.child("Para_ContactGraph");
            if(node_contactgraph){
                // Para_ContactGraph
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_contactgraph);
            }
        }

        // para crossmesh
        {
            pugi::xml_node node_crossmesh = node_guisettings.child("Para_CrossMesh");
            if(node_crossmesh){
                // Para_CrossMesh
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node_crossmesh);
            }
        }

        // para opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        // Para_Assembly
        {
            pugi::xml_node node = node_guisettings.child("Para_Assembly");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        // Para_Support
        {
            pugi::xml_node node = node_guisettings.child("Para_Support");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        // Para_Opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(node){
                vector<shared_ptr<InputVar>> series = data.varList->varLists;
                for(shared_ptr<InputVar> var: series) varOrganizer.read(var.get(), node);
            }
        };

        // pickPartIDs
        {
            pugi::xml_node node_pickPartIDs = node_guisettings.child("pickPartIDs");
            data.pickPartIDs.clear();
            if(node_pickPartIDs)
            {
                pugi::xml_text text_pickPartIDs = node_pickPartIDs.text();
                string str_text = text_pickPartIDs.as_string();
                vector<string> split_strs;
                split_strs = split(str_text, ',');
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

        // interactMatrix
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
    }
}

bool XMLIO_backward::XMLReader_ReferenceSurface(pugi::xml_node &xml_root, const string xmlFileName_path, IOData &data){
    pugi::xml_node refenceSurfaceNode = xml_root.child("Output").child("Structure");
    if (refenceSurfaceNode)
    {
        path reference_surface_path(refenceSurfaceNode.attribute("path").as_string());
        string path_str = (path(xmlFileName_path) / reference_surface_path).string();
        data.reference_surface = make_shared<PolyMesh<double>>(data.varList);
        if (!data.reference_surface->readOBJModel(path_str.c_str(), false))
            return false;

        data.varList->add(data.reference_surface->texturedModel, "texturedModel", "");
    }
    return true;
}


bool XMLIO_backward::XMLReader_CrossMesh(pugi::xml_node &xml_root, const string xmlFileName_path, IOData &data)
{
    pugi::xml_node cross_mesh_node = xml_root.child("PartGeoData");
    if(cross_mesh_node)
    {
        //CrossData
        if(cross_mesh_node.child("CrossData"))
        {
            path cross_data_path = cross_mesh_node.child("CrossData").attribute("path").as_string();
            cross_data_path = path(xmlFileName_path) / cross_data_path;
            pugi::xml_document cross_doc;
            cross_doc.load_file(cross_data_path.string().c_str());
            pugi::xml_node cross_data_node = cross_doc.child("Documents").child("CrossData");
            if (cross_data_node)
            {
                data.cross_mesh = make_shared<CrossMesh<double>>(data.varList);
                vector< shared_ptr<Cross<double>>> crossList;
                for (pugi::xml_node cross_node: cross_data_node.children())
                {
                    int crossID = cross_node.attribute("id").as_int();
                    shared_ptr<Cross<double>> cross = make_shared<Cross<double>>(data.varList);
                    cross->crossID = crossID;
                    for (pugi::xml_node ori_node : cross_node.children())
                    {
                        int edgeID = ori_node.attribute("id").as_int();
                        if (edgeID < 0) continue;

                        Eigen::Vector3d vertex = readXYZ(ori_node.child("Vertex").attribute("XYZ").as_string());
                        cross->push_back(vertex);

                        Eigen::Vector3d point = readXYZ(ori_node.child("Point").attribute("XYZ").as_string());
                        Eigen::Vector3d normal = readXYZ(ori_node.child("Normal").attribute("XYZ").as_string());
                        Eigen::Vector3d axis = readXYZ(ori_node.child("Rotation_Axis").attribute("XYZ").as_string());

                        shared_ptr<OrientPoint<double>> oript = make_shared<OrientPoint<double>>(point, normal, axis);

                        double angle = ori_node.child("Angle").attribute("Radian").as_double();
                        if(angle < 0){
                            oript->tiltSign = -1;
                            oript->updateAngle(angle);
                        }
                        else{
                            oript->tiltSign = 1;
                            oript->updateAngle(angle);
                        }
                        cross->oriPoints.push_back(oript);
                    }
                    crossList.push_back(cross);
                }
                std::sort(crossList.begin(), crossList.end(), [=](shared_ptr<Cross<double>> a, shared_ptr<Cross<double>> b){
                    return a->crossID < b->crossID;
                });
                
                for(auto cross: crossList){
                    data.cross_mesh->push_back(cross);
                }
                
                data.cross_mesh->update();
            }
        }
    }
    return data.cross_mesh != nullptr;
}

void XMLIO_backward::XMLReader_Boundary(pugi::xml_node &xml_root, IOData &data)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    if(node_guisettings)
    {
        pugi::xml_node node_boundary = node_guisettings.child("Boundary_PartIDs");
        if(node_boundary)
        {
            //2) read the boundary part
            pugi::xml_text text_boundary = node_boundary.text();
            vector<string> str_partIDs = split(text_boundary.get(), ',');

            //3) assign boundary marker
            data.boundary_crossIDs.clear();
            for(size_t id = 0; id < str_partIDs.size(); id++)
            {
                data.boundary_crossIDs.push_back(std::stoi(str_partIDs[id]));
            }
        }
    }
}

Eigen::Vector3d XMLIO_backward::readXYZ(std::string xyz_str){
    double x, y, z;
    sscanf(xyz_str.c_str(), "(%lf,%lf,%lf)", &x, &y, &z);
    return Eigen::Vector3d(x, y, z);
}


vector<std::string> XMLIO_backward::split(const string str_text, const char separator) const{
    vector<std::string> split_strings;
    string accumulate = "";
    for(size_t id = 0; id < str_text.size(); id++){
        if(str_text[id] == separator || (id + 1 == str_text.size()))
        {
            if(id + 1 == str_text.size()){
                accumulate += str_text[id];
            }
            
            if(accumulate.find_first_not_of (' ') != accumulate.npos)
            {
                split_strings.push_back(accumulate);
                accumulate = "";
            }
        }
        else{
            accumulate += str_text[id];
        }
    }

    return split_strings;
}
