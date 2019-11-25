//
// Created by ziqwang on 22.02.19.
//

#include "gluiXML.h"
#include <sstream>
#include <boost/algorithm/string.hpp>
extern int mainWinW, mainWinH;

void gluiXML::XMLWriter_GUISettings(pugi::xml_node &xml_root)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    //guisettings
    {
        if(!node_guisettings) node_guisettings = xml_root.append_child("GUISettings");

        gluiVarOrganizer varOrganizer;
        //para basic
        {
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Basic");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node_guisettings);
        }

        //Para_ShapeOp
        {
            pugi::xml_node node_shapeop = node_guisettings.child("ShapeOp_Settings");
            if(!node_shapeop) node_shapeop = node_guisettings.append_child("ShapeOp_Settings");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_ShapeOp");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node_shapeop);
        }

        //Para_Mitsuba
        {
            pugi::xml_node node_mitsuba = node_guisettings.child("Para_Mitsuba");
            if(!node_mitsuba) node_mitsuba = node_guisettings.append_child("Para_Mitsuba");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Mitsuba");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node_mitsuba);
        }


        //Para_ContactGraph
        {
            pugi::xml_node node_contactgraph = node_guisettings.child("Para_ContactGraph");
            if(!node_contactgraph ) node_contactgraph  = node_guisettings.append_child("Para_ContactGraph");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_ContactGraph");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node_contactgraph);
        }

        //Para_CrossMesh
        {
            pugi::xml_node node = node_guisettings.child("Para_CrossMesh");
            if(!node ) node  = node_guisettings.append_child("Para_CrossMesh");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_CrossMesh");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Para_Opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(!node ) node  = node_guisettings.append_child("Para_Opt");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Opt");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Para_Assembly
        {
            pugi::xml_node node = node_guisettings.child("Para_Assembly");
            if(!node ) node  = node_guisettings.append_child("Para_Assembly");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Assembly");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Para_Support
        {
            pugi::xml_node node = node_guisettings.child("Para_Support");
            if(!node ) node  = node_guisettings.append_child("Para_Support");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Support");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node);
        }

        //Optimization_Para
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(!node ) node  = node_guisettings.append_child("Para_Opt");
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Opt");
            for(shared_ptr<gluiVar> var: series) varOrganizer.write(var.get(), node);
        }

        // pickPartIDs
        {
            pugi::xml_node node_pickpartIDs = node_guisettings.child("pickPartIDs");
            if (!node_pickpartIDs) node_pickpartIDs = node_guisettings.append_child("pickPartIDs");
            pugi::xml_text text_pickpartIDs = node_pickpartIDs.text();
            string str_pickpartIDs;
            for (int id = 0; id < pickPartIDs.size(); id++){
                str_pickpartIDs += to_string(pickPartIDs[id]);
                if(id != (int)pickPartIDs.size() - 1) str_pickpartIDs += ",";
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
                sprintf(interactmatrix_string, "%s%lf,", interactmatrix_string, interactMatrix[id]);
            }
            sprintf(interactmatrix_string, "%s%lf", interactmatrix_string, interactMatrix[15]);
            text_interactmatrix.set(interactmatrix_string);
        }

        //BoundaryPartID
        if(myStrucCreator && myStrucCreator->myStruc)
        {
            pugi::xml_node node_boundary = node_guisettings.child("Boundary_PartIDs");
            if(!node_boundary) node_boundary = node_guisettings.append_child("Boundary_PartIDs");
            pugi::xml_text text_boundary = node_boundary.text();
            char boundary_string[10000] = "";
            for(int id = 0; id < myStrucCreator->myStruc->partList.size(); id++)
            {
                pPart part = myStrucCreator->myStruc->partList[id];
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
                    sprintf(worldAxesMat_string, "%s%lf,", worldAxesMat_string, myStrucCreator->worldAxesMat[id]);
                }
                sprintf(worldAxesMat_string, "%s%lf", worldAxesMat_string, myStrucCreator->worldAxesMat[15]);
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
                    sprintf(worldMatrix_string, "%s%lf,", worldMatrix_string, myStrucCreator->worldMatrix[id]);
                }
                sprintf(worldMatrix_string, "%s%lf", worldMatrix_string, myStrucCreator->worldMatrix[15]);
                text_worldMatrix.set(worldMatrix_string);
            }
        }
    }
}

void gluiXML::XMLWriter_Output(pugi::xml_node &xmlroot, boost::filesystem::path &xmlFileName_path)
{
    pugi::xml_node xml_output = xmlroot.child("Output");
    if(!xml_output) xml_output = xmlroot.append_child("Output");
    //  ./name_data
    {

        string file_name = xmlFileName_path.filename().stem().string();
        string data_path = xmlFileName_path.parent_path().string();
        data_path += "/" + file_name + "_data";
        //Surface
        if(myStrucCreator->refModel && myStrucCreator->refModel->polyMesh)
        {
            string surface_path = data_path + "/" + file_name + "_Surface.obj";
            myStrucCreator->refModel->polyMesh->WriteOBJModel(surface_path.c_str());

            shared_ptr<CrossMesh> crossMesh = myStrucCreator->refModel->crossMesh;
            if(crossMesh && varList.get<bool>("output_mitsuba")){
                string surface_wire_path = data_path + "/" + file_name + "_CrossMeshWire.obj";
                shared_ptr<Cross> cross;
                shared_ptr<Part> part = make_shared<Part>(cross);
                shared_ptr<PolyMesh> polyMesh = make_shared<PolyMesh>();
                for(int id = 0; id < crossMesh->crossList.size(); id++){
                    shared_ptr<_Polygon> poly = make_shared<_Polygon>(*((_Polygon *)crossMesh->crossList[id].get()));
                    polyMesh->polyList.push_back(poly);
                }
                polyMesh->UpdateVertices();
                part->polyMesh = polyMesh;
                polyMesh->TranslateMesh(Vector3f(0, -varList.get<float>("ground_height"), 0));
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
        if(myStrucCreator->refModel && myStrucCreator->refModel->crossMesh)
        {
            string crossMesh_path = data_path + "/" + file_name + "_CrossMesh.obj";
            shared_ptr<CrossMesh> crossMesh = make_shared<CrossMesh>(*myStrucCreator->refModel->crossMesh);
            crossMesh ->TranslateMesh(Vector3f(0, -varList.get<float>("ground_height"), 0));
            crossMesh->WriteOBJModel(crossMesh_path.c_str());

            pugi::xml_node crossMesh_node = xml_output.child("CrossMesh");
            if(!crossMesh_node) crossMesh_node = xml_output.append_child("CrossMesh");
            pugi::xml_attribute cross_attr = crossMesh_node.attribute("path");
            if(!cross_attr) cross_attr = crossMesh_node.append_attribute("path");
            string str_attr = file_name + "_" + "data" + "/" + file_name + "_CrossMesh.obj";
            cross_attr.set_value(str_attr.c_str());
        }

        //PartGeometry
        if(myStrucCreator->myStruc)
        {
            string part_path = data_path + "/" + "PartGeometry";
            if(!boost::filesystem::exists(part_path))
            {
                boost::filesystem::create_directories(part_path);
            }

            myStrucCreator->myStruc->WriteStructure(part_path.c_str());
            if(varList.get<bool>("output_mitsuba")) {
                myStrucCreator->myStruc->WriteStructureWireFrame(part_path.c_str());
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

        //Simulation
        if(myStrucCreator->myStruc && !varList.get<bool>("output_mitsuba"))
        {
            string part_path = data_path + "/" + "Simulation";
            if(!boost::filesystem::exists(part_path))
            {
                boost::filesystem::create_directories(part_path);
            }
            myStrucCreator->myStruc->WriteStructureForSimulation(part_path.c_str());
        }
    }
}

void gluiXML::XMLWriter_Mitsuba(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path)
{
    string file_name = xmlFileName_path.filename().stem().string();
    string data_path = xmlFileName_path.parent_path().string();
    data_path += "/" + file_name + "_data";

    string mitsuba_path = data_path + "/mitsuba.xml";
    pugi::xml_document mitsuba_doc;

    gluiMitsuba mitsuba_writter;

    mitsuba_writter.mitsuba_sensor = mitsuba_sensor;

    mitsuba_writter.scene_settings(mitsuba_doc);
    mitsuba_doc.save_file(mitsuba_path.c_str());

    xml_root.remove_child("Mitsuba");
    pugi::xml_node mitsuba_node = xml_root.append_child("Mitsuba");
    pugi::xml_node mitsubadata_node = mitsuba_node.append_child("MitsubaData");
    pugi::xml_attribute mitsubadata_attr = mitsubadata_node.append_attribute("path");
    string relative_Mitsuba_path = file_name + "_data" + "/mitsuba.xml";
    mitsubadata_attr.set_value(relative_Mitsuba_path.c_str());
}

void gluiXML::XMLWriter_PartGeoData(pugi::xml_node &xml_root, boost::filesystem::path &xmlFileName_path)
{
    string file_name = xmlFileName_path.filename().stem().string();
    string data_path = xmlFileName_path.parent_path().string();
    data_path += "/" + file_name + "_data";

    xml_root.remove_child("PartGeoData");
    pugi::xml_node partGeo_node = xml_root.append_child("PartGeoData");

    //CrossData
    if(myStrucCreator->refModel && myStrucCreator->refModel->crossMesh)
    {
        string CrossData_path = data_path + "/CrossMesh.xml";
        {
            pugi::xml_document crossDoc;
            pugi::xml_node cross_root = crossDoc.append_child("Documents");
            xml_crossdata = cross_root;

            //CrossData
            pugi::xml_node crossdata_node = cross_root.append_child("CrossData");
            int kd = 0;
            for(auto cross : myStrucCreator->refModel->crossMesh->crossList)
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

            //TiltNormal
            {
                vector<Vector3f> lines;
                pugi::xml_node node_tiltNormal = cross_root.append_child("TiltNormal");
                myStrucCreator->refModel->crossMesh->getCrossMesh_TiltNormal(lines);
                pugi::xml_text text_tiltNormal = node_tiltNormal.text();
                string strTiltNormalList;

                strTiltNormalList = "[";
                for(int id = 0; id < lines.size()/2; id ++)
                {
                    char strTiltNormal[1000];
                    Vector3f staPt = lines[2 * id];
                    Vector3f endPt = lines[2 * id + 1];
                    sprintf(strTiltNormal, "[[%lf, %lf, %lf], [%lf, %lf, %lf]]", staPt[0], staPt[1], staPt[2], endPt[0], endPt[1], endPt[2]);
                    strTiltNormalList += strTiltNormal;
                    if(id != lines.size()/2 - 1)
                        strTiltNormalList += ", ";
                }
                strTiltNormalList += "]";
                text_tiltNormal.set(strTiltNormalList.c_str());
            }

            //MultMovement
            if(varList.get<bool>("mult_move"))
            {
                vector<Vector3f> lines;
                pugi::xml_node node_multMove = cross_root.append_child("MultMove");
                myStrucCreator->myStruc->saveMultMovement(lines);
                pugi::xml_text text_multMove = node_multMove.text();
                string strmultMove;

                strmultMove = "[";
                for(int id = 0; id < lines.size()/2; id ++)
                {
                    char strMove[1000];
                    Vector3f staPt = lines[2 * id] + Vector3f(0, -varList.get<float>("ground_height"), 0);
                    Vector3f endPt = lines[2 * id + 1] + Vector3f(0, -varList.get<float>("ground_height"), 0);
                    sprintf(strMove, "[[%lf, %lf, %lf], [%lf, %lf, %lf]]", staPt[0], staPt[1], staPt[2], endPt[0], endPt[1], endPt[2]);
                    strmultMove += strMove;
                    if(id != lines.size()/2 - 1)
                        strmultMove += ", ";
                }
                strmultMove += "]";
                text_multMove.set(strmultMove.c_str());
            }


            crossDoc.save_file(CrossData_path.c_str());
            crossDoc.remove_child("Documents");
        }

        pugi::xml_node crossdata_node = partGeo_node.append_child("CrossData");
        pugi::xml_attribute crossdata_attr = crossdata_node.append_attribute("path");
        string relative_CrossData_path = file_name + "_data" + "/CrossMesh.xml";
        crossdata_attr.set_value(relative_CrossData_path.c_str());
    }

    //MergedPartGroup
    if(myStrucCreator->myStruc && myStrucCreator->myAssembly)
    {
        string MPG_path = data_path + "/MergePartGroup.xml";
        {
            pugi::xml_document MPG_Doc;
            pugi::xml_node MPG_root = MPG_Doc.append_child("Documents");
            xml_groupdata = MPG_root;
            pugi::xml_node MPGdata_node = MPG_root.append_child("MergePartGroup");
            int GroupID = 0;

            vector<shared_ptr<AssemblyNode>> &assembly = myStrucCreator->myAssembly->assembly_sequences;
            for(size_t id = 0; id < assembly.size(); id++)
            {
                shared_ptr<AssemblyNode> assembly_node = assembly[id];
                switch(assembly_node->type)
                {
                    case AssemblyNode::NodeKey:case AssemblyNode::NodePart: {
                        int partID = assembly_node->part.lock()->partID;
                        pugi::xml_node group_node = MPGdata_node.append_child("Group");

                        //groupID
                        group_node.append_attribute("id").set_value(GroupID++);

                        //groupType
                        group_node.append_attribute("type").set_value("Single");

                        //groupXYZ
                        char XYZ[1024];
                        sprintf(XYZ, "(%f, %f, %f)", assembly_node->dis_drt[0], assembly_node->dis_drt[1], assembly_node->dis_drt[2]);
                        group_node.append_attribute("XYZ").set_value(XYZ);

                        string groupMember = to_string(partID);
                        group_node.text().set(groupMember.c_str());
                    }
                        break;
                    case AssemblyNode::NodeBoundary:
                    {
                        pugi::xml_node group_node = MPGdata_node.append_child("Group");
                        group_node.append_attribute("id").set_value(GroupID++);
                        string groupMember = "";
                        int NBoundary = assembly_node->partGroup->groupParts.size();
                        for(size_t jd = 0; jd < NBoundary; jd++)
                        {
                            int partID = assembly_node->partGroup->groupParts[jd].lock()->partID;
                            groupMember +=  to_string(partID);
                            if(jd != NBoundary - 1) groupMember += ",";
                        }
                        group_node.text().set(groupMember.c_str());
                        group_node.append_attribute("type").set_value("Boundary");

                        //groupXYZ
                        char XYZ[1024];
                        sprintf(XYZ, "(%f, %f, %f)", assembly_node->dis_drt[0], assembly_node->dis_drt[1], assembly_node->dis_drt[2]);
                        group_node.append_attribute("XYZ").set_value(XYZ);
                    }
                    default:
                        break;
                }

            }
            MPG_Doc.save_file(MPG_path.c_str());
        }

        string relative_MPG_path = file_name + "_data" + "/MergePartGroup.xml";
        pugi::xml_node MPG_node = partGeo_node.append_child("MergePartGroup");
        MPG_node.append_attribute("path").set_value(relative_MPG_path.c_str());
    }


    //Support
    //MergedPartGroup
    double Scale = varList.get<float>("clipper_scale");
    if(myStrucCreator->myStruc && myStrucCreator->myAssembly)
    {
        string Support_path = data_path + "/Support.xml";
        {
            pugi::xml_document Support_Doc;
            pugi::xml_node Support_root = Support_Doc.append_child("Documents");
            xml_groupdata = Support_root;
            pugi::xml_node Supportdata_node = Support_root.append_child("Curves");
            int GroupID = 0;

            vector<shared_ptr<SupportSlice>> &slices = myStrucCreator->myAssembly->support_slices;
            for(size_t id = 0; id < slices.size(); id++)
            {
                shared_ptr<SupportSlice> slice = slices[id];
                if(slice == nullptr) continue;
                pugi::xml_node slice_node = Supportdata_node.append_child("Slice");
                slice_node.append_attribute("id").set_value(id);
                slice_node.append_attribute("layer_height").set_value(slice->height);
                for(int jd = 0; jd < slice->countour.size(); jd++)
                {
                    pugi::xml_node curve_node = slice_node.append_child("Curve");
                    curve_node.append_attribute("id").set_value(jd);
                    string pts_str;
                    for(int kd = 0; kd < slice->countour[jd].size(); kd++)
                    {
                        char PtXYZ[256];
                        ClipperLib::IntPoint intpt = slice->countour[jd][kd];
                        sprintf(PtXYZ, "%.4f, %.4f", (double)intpt.X/Scale, (double)intpt.Y/Scale);
                        pts_str = pts_str + PtXYZ + (kd == slice->countour[jd].size() - 1 ? "" : ",");
                    }
                    curve_node.text().set(pts_str.c_str());
                }
            }

            Support_Doc.save_file(Support_path.c_str());
        }

        string relative_Support_path = file_name + "_data" + "/Support.xml";
        pugi::xml_node Support_node = partGeo_node.append_child("Support");
        Support_node.append_attribute("path").set_value(Support_path.c_str());
    }
}

void gluiXML::XMLWriter_Animation(string data_folder)
{
    if(myStrucCreator && myStrucCreator->myStruc && myStrucCreator->myAssembly)
    {
        string animation_folder = data_folder + "/Animation";
        boost::filesystem::create_directories(animation_folder);
        for(int id = 0; id < myStrucCreator->myAssembly->assembly_sequences.size(); id++){
            shared_ptr<AssemblyNode> node = myStrucCreator->myAssembly->assembly_sequences[id];
            node->filename = "part_" + std::to_string(id) + ".obj";
            string outputFile = animation_folder + "/" + node->filename;
            myStrucCreator->myAssembly->writeAssemblyNode(node, outputFile.c_str());
        }
        string animationFile = animation_folder + "/animation.motion.txt";
        myStrucCreator->myAssembly->writeAnimationMotion(animationFile.c_str());
    }
}

bool gluiXML::SaveXMLFile(string xmlFileName)
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

        XMLWriter_GUISettings(documents_node);
        XMLWriter_PartGeoData(documents_node, xmlFileName_path);
        if(varList.get<bool>("output_mitsuba"))
            XMLWriter_Mitsuba(documents_node, xmlFileName_path);
        XMLWriter_Output(documents_node, xmlFileName_path);
        XMLWriter_Animation(data_path);
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



void gluiXML::XMLReader_GUISettings(pugi::xml_node &xml_root)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    //guisettings
    {
        if(!node_guisettings) return;

        //para basic
        gluiVarOrganizer varOrganizer;
        {
            vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Basic");
            for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node_guisettings);
        }


        //para shapeop
        {
            pugi::xml_node node_shapeop = node_guisettings.child("ShapeOp_Settings");
            if(node_shapeop){
                //Para_ShapeOp
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_ShapeOp");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node_shapeop);
            }
        }


        //para mitsuba
        {
            pugi::xml_node node_mitsuba = node_guisettings.child("Para_Mitsuba");
            if(node_mitsuba){
                //Para_Mitsuba
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Mitsuba");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node_mitsuba);
            }
        }

        //para contactgraph
        {
            pugi::xml_node node_contactgraph = node_guisettings.child("Para_ContactGraph");
            if(node_contactgraph){
                //Para_ContactGraph
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_ContactGraph");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node_contactgraph);
            }
        }

        //para crossmesh
        {
            pugi::xml_node node_crossmesh = node_guisettings.child("Para_CrossMesh");
            if(node_crossmesh){
                //Para_CrossMesh
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_CrossMesh");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node_crossmesh);
            }
        }

        //para opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(node){
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Opt");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        //Para_Assembly
        {
            pugi::xml_node node = node_guisettings.child("Para_Assembly");
            if(node){
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Assembly");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        //Para_Support
        {
            pugi::xml_node node = node_guisettings.child("Para_Support");
            if(node){
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Support");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node);
            }
        }

        //Para_Opt
        {
            pugi::xml_node node = node_guisettings.child("Para_Opt");
            if(node){
                vector<shared_ptr<gluiVar>> series = varList.findSeries("Para_Opt");
                for(shared_ptr<gluiVar> var: series) varOrganizer.read(var.get(), node);
            }
        };

        //pickPartIDs
        {
            pugi::xml_node node_pickPartIDs = node_guisettings.child("pickPartIDs");
            pickPartIDs.clear();
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
                        pickPartIDs.push_back(PartID);
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
                       &interactMatrix[0], &interactMatrix[1], &interactMatrix[2], &interactMatrix[3],
                       &interactMatrix[4], &interactMatrix[5], &interactMatrix[6], &interactMatrix[7],
                       &interactMatrix[8], &interactMatrix[9], &interactMatrix[10], &interactMatrix[11],
                       &interactMatrix[12], &interactMatrix[13], &interactMatrix[14], &interactMatrix[15]);
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
                        double *worldAxesMat = myStrucCreator->worldAxesMat;
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
                        double *worldMatrix = myStrucCreator->worldMatrix;
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

void gluiXML::XMLReader_PartGeoData(pugi::xml_node &xml_root, string &xmlFileName_path)
{
    pugi::xml_node partGeo_node = xml_root.child("PartGeoData");
    if(partGeo_node)
    {
        //CrossData
        if(myStrucCreator->refModel && myStrucCreator->refModel->crossMesh)
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
                        Cross *cross = myStrucCreator->refModel->crossMesh->crossList[crossID].get();
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
//                            std::cout << oript->normal << std::endl;
//                            pugi::xml_node normal_node = ori_node.child("Normal");
//                            string normal_str = normal_node.attribute("XYZ").as_string();
//                            sscanf(normal_str.c_str(), "(%f, %f, %f)", &cross->oriPoints[oriPtID]->normal[0],
//                                    &cross->oriPoints[oriPtID]->normal[1],
//                                    &cross->oriPoints[oriPtID]->normal[2]);
//                            std::cout << cross->oriPoints[oriPtID]->normal << std::endl;
                        }
                    }

                    myStrucCreator->CreateStructure(false, false, interactMatrix, false);
                }
            }
        }

//        //MergePartGroup
//        if(myStrucCreator->myStruc)
//        {
//            myStrucCreator->myStruc->mergePartGroups.clear();
//            pugi::xml_node MPGdata_node = partGeo_node.child("MergePartGroup");
//            if(MPGdata_node)
//            {
//                string MPGData_path = MPGdata_node.attribute("path").as_string();
//                MPGData_path = xmlFileName_path + "/" + MPGData_path;
//                pugi::xml_document MPGDoc;
//                MPGDoc.load_file(MPGData_path.c_str());
//                MPGdata_node = MPGDoc.child("Documents").child("MergePartGroup");
//
//                vector<vector<int>> GroupPartIDs;
//                for(pugi::xml_node group_node : MPGdata_node.children())
//                {
//                    //read GroupID
//                    int groupID = group_node.attribute("id").as_int();
//
//                    //read GroupParts
//                    string str_partIDs = group_node.text().as_string();
//                    vector<string> split_strs;
//                    vector<int>partIDs;
//                    split(split_strs, str_partIDs, boost::is_any_of(","));
//                    for(string split_str : split_strs)
//                    {
//                        int PartID = std::stoi(split_str, nullptr, 10);
//                        partIDs.push_back(PartID);
//                    }
//                    GroupPartIDs.push_back(partIDs);
//
//                    //read type, only add type = "Multiple" into struc
//                    pugi::xml_attribute type_attrib = group_node.attribute("type");
//                    if(type_attrib != NULL)
//                    {
//                        string str_type = type_attrib.as_string();
//                        if(str_type[0] == 'M') myStrucCreator->myStruc->mergePartGroups.push_back(partIDs);
//                    }
//                }
//
//                /*
//                 * 2. Add all key part into struc
//                 */
//                pugi::xml_node node_Disassembly = MPGDoc.child("Documents").child("Disassembly");
//                if(node_Disassembly)
//                {
//                    for(pugi::xml_node group_node : node_Disassembly.children())
//                    {
//                        string type_str = group_node.attribute("type").as_string();
//                        if(type_str[0] == 'k')
//                        {
//                            int groupID = group_node.attribute("id").as_int();
//                            for(int partID : GroupPartIDs[groupID])
//                            {
//                                myStrucCreator->myStruc->keyPartsID.push_back(partID);
//                            }
//                        }
//                    }
//                }
//
//                std::cout << "keyPartsID:\t [";
//                for(int partID : myStrucCreator->myStruc->keyPartsID)
//                    std::cout << partID << " ";
//                std::cout << "]" << std::endl;
//            }
//        }
    }
}

bool gluiXML::readXMLFile(string xmlFileName) {
    xmldoc.load_file(xmlFileName.c_str());
    pugi::xml_node xml_root = xmldoc.child("Documents");
    if (xml_root) {
        gluiXML xml;
        xml.XMLReader_GUISettings(xml_root);
        boost::filesystem::path boostpath(xmlFileName);
        varList.filename = boostpath.stem().string();
        string xmlfile_path = boost::filesystem::path(xmlFileName).parent_path().string();

        if (varList.get<bool>("texturedModel") == false) {
            pugi::xml_node crossMesh = xml_root.child("Output").child("CrossMesh");
            if (crossMesh) {
                string path = xmlfile_path + "/" + crossMesh.attribute("path").as_string();
                std::cout << "Read File...\t:" << path << std::endl;
                if (!myStrucCreator->LoadSurface(path.c_str()))
                    return false;
                myStrucCreator->CreateStructure(true, false, interactMatrix, true);
            }
        } else {
            pugi::xml_node surfaceMesh = xml_root.child("Output").child("Structure");
            if (surfaceMesh) {
                string path = xmlfile_path + "/" + surfaceMesh.attribute("path").as_string();
                std::cout << "Read File...\t:" << path << std::endl;
                if (!myStrucCreator->LoadSurface(path.c_str()))
                    return false;
                myStrucCreator->CreateStructure(true, true, interactMatrix, true);
            }
        }

        if (myStrucCreator->myStruc != nullptr) {
            if (xml_root) {
                gluiXML xml;
                xml.XMLReader_PartGeoData(xml_root, xmlfile_path);
                updateBoundaryPart(xml_root);
            }
        }
    }

    return true;
}

void gluiXML::updateBoundaryPart(pugi::xml_node &xml_root)
{
    pugi::xml_node node_guisettings = xml_root.child("GUISettings");
    //guisettings
    {
        pugi::xml_node node_boundary = node_guisettings.child("Boundary_PartIDs");
        if(node_boundary && myStrucCreator && myStrucCreator->myStruc)
        {
            for(pPart part : myStrucCreator->myStruc->partList){
                if(part) part->atBoundary = false;
            }
            pugi::xml_text text_boundary = node_boundary.text();
            vector<string> str_partIDs;
            string text_str = text_boundary.get();
            boost::split(str_partIDs, text_str, boost::is_any_of(","));
            for(int id = 0; id < str_partIDs.size(); id++){
                if(str_partIDs[id] == "") continue;
                int partID = std::atoi(str_partIDs[id].c_str());
                if(partID < 0 || partID >= myStrucCreator->myStruc->partList.size())
                    continue;
                else{
                    pPart part = myStrucCreator->myStruc->partList[partID];
                    if(part) part->atBoundary = true;
                }
            }
        }
    }
}


void gluiMitsuba::scene_settings(pugi::xml_node &xml_root) {
    //scene
    pugi::xml_node scene_node;
    change_attribute(xml_root, scene_node, "scene", "version", "0.5.0");
    float mitsubaPartYRotation = varList.get<float>("mitsuba_PartYRotation");
    float mitsubaCameraRotation = varList.get<float>("mitsuba_CameraRotation") / 180 * M_PI;
    float mitsubaCameraDistance = varList.get<float>("mitsuba_CameraDistance");
    float mitsubaSunTime = varList.get<float>("mitsuba_SunTime");
    float mitsubaSunStrength = varList.get<float>("mitsuba_SunStrength");

    int   mitsuba_width =   varList.get<int>("mitsuba_width");
    int   mitsuba_height =   varList.get<int>("mitsuba_height");
    int   mitsuba_sample =   varList.get<int>("mitsuba_sample");

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
        if(myStrucCreator->myStruc)
        {
            shared_ptr<Struc> struc = myStrucCreator->myStruc;
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

void gluiMitsuba::change_attribute(pugi::xml_node &root, pugi::xml_node &node, string str_name, string str_attr, string str_value)
{
    node = root.append_child(str_name.c_str());
    pugi::xml_attribute attr = node.attribute(str_attr.c_str());
    if(!attr) attr = node.append_attribute(str_attr.c_str());
    attr.set_value(str_value.c_str());
    return;
}

void gluiMitsuba::change_attribute(pugi::xml_node &node, string str_attr, string str_value)
{
    pugi::xml_attribute attr = node.attribute(str_attr.c_str());
    if(!attr) attr = node.append_attribute(str_attr.c_str());
    attr.set_value(str_value.c_str());
    return;
}
