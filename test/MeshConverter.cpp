////
//// Created by ziqwang on 2019-12-15.
////
//
//#include "Mesh/MeshConverter.h"
//#include <catch2/catch.hpp>
//#include "IO/XMLIO.h"
//
//TEST_CASE("Class MeshConverter")
//{
//    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
//    InitVarLite(varList.get());
//
//    SECTION("merge all face")
//    {
//        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
//        InitVarLite(varList.get());
//        vector<shared_ptr<PolyMesh>> meshes;
//        pPolyMesh mesh = make_shared<PolyMesh>(varList);
//
//        boost::filesystem::path current_path(boost::filesystem::current_path());
//        boost::filesystem::path objfilepath;
//        if(current_path.filename() == "TopoLite"){
//            objfilepath = current_path / "data/TopoInterlock/XML/origin_data/PartGeometry/Part_01.obj";
//        }
//        else{
//            objfilepath = current_path / "../data/TopoInterlock/XML/origin_data/PartGeometry/Part_01.obj";
//        }
//
//        bool texture;
//        REQUIRE(mesh->ReadOBJModel(objfilepath.string().c_str(), texture, false));
//        meshes.push_back(mesh);
//
//        MeshConverter converter(varList);
//
//        converter.Convert2PolyMesh(mesh);
//
//        REQUIRE(meshes[0]->polyList.size() == 8);
//    }
//}