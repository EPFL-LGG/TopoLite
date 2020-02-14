//
// Created by ziqwang on 2020-02-03.
//

#include <catch2/catch.hpp>
#include "Interlocking/InterlockingSolver_AffineScaling.h"
#include "IO/XMLIO.h"

TEST_CASE("Solve Interlocking")
{
    XMLIO Reader;
    XMLData data;

    boost::filesystem::path current_path(boost::filesystem::current_path());
    boost::filesystem::path debugxml_filepath;
    if (current_path.filename() == "TopoLite")
    {
        debugxml_filepath = current_path / "data/TopoInterlock/XML/origin.xml";
    }
    else
    {
        debugxml_filepath = current_path / "../data/TopoInterlock/XML/origin.xml";
    }

    REQUIRE(Reader.XMLReader(debugxml_filepath.string(), data) == 1);

    shared_ptr<Struc> struc = data.strucCreator->struc;
    shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(data.varList);

    vector<shared_ptr<PolyMesh>> meshes;
    vector<bool> atBoundary;

    for(int partID = 0; partID < struc->partList.size(); partID++){
        pPart part = struc->partList[partID];
        meshes.push_back(part->polyMesh);
        atBoundary.push_back(part->atBoundary);
    }

    graph->constructFromPolyMeshes(meshes, atBoundary);
    graph->finalize();

    InterlockingSolver_AffineScaling solver(graph, data.varList);
    shared_ptr<InterlockingData> interlockData;
    std::cout << solver.isRotationalInterlocking(interlockData) << std::endl;
}
