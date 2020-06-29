//
// Created by ziqwang on 06.06.20.
//

#include "guiManager_TopoCreator.h"
#include "guiShader_MeshesWireFrame.h"
/*
 * Input and Output
 */

bool guiManager_TopoCreator::load_from_xmlfile(string xmlFileName)
{
    XMLIO_backward IO;
    clear();
    iodata = make_shared<IOData>();
    if(IO.XMLReader(xmlFileName, *iodata) && iodata->varList) {
        iodata->varList->add((int)1, "layerOfBoundary",  "");
        iodata->varList->add((float)0.002, "wireframe_linewidth",  "");
        iodata->varList->add((float)0.003, "augmentedvectors_linewidth",  "");
        iodata->varList->add((float)0.005, "pattern_linewidth",  "");
        init_from_IOData();
        return true;
    }
    return false;
}

bool guiManager_TopoCreator::load_from_jsonfile(string xmlFileName){
    iodata = make_shared<IOData>();
    JsonIOReader reader(xmlFileName, iodata);
    clear();
    if(reader.read()){
        iodata->varList->add((float)0.002, "wireframe_linewidth",  "");
        iodata->varList->add((float)0.003, "augmentedvectors_linewidth",  "");
        iodata->varList->add((float)0.005, "pattern_linewidth",  "");
        init_from_IOData();
        return true;
    }
    return false;
}

void guiManager_TopoCreator::write_to_json(string jsonFileName)
{
    if(crossMeshCreator){
        if(crossMeshCreator->referenceSurface){
            iodata->reference_surface = crossMeshCreator->referenceSurface;
        }

        if(crossMeshCreator->pattern2D){
            iodata->pattern_mesh = crossMeshCreator->pattern2D->getPolyMesh();
        }

        if(crossMeshCreator->crossMesh){
            iodata->cross_mesh = crossMeshCreator->crossMesh;
            iodata->varList->add(crossMeshCreator->crossMesh->getBoundaryCrossIDs(), "boundary_crossIDs", "");
        }

        iodata->varList->add(last_textureMat, "texturedMat", "");
    }
    JsonIOWriter writer(jsonFileName, iodata);
    writer.write();
}

/*
 * Initialization
 */

void guiManager_TopoCreator::init_from_IOData()
{
    //0)
    crossMeshCreator = make_shared<CrossMeshCreator<double>>(iodata->varList);
    last_textureMat = init_textureMat = iodata->varList->getMatrix4d("texturedMat");

    //1) set crossMeshCreator
    if(iodata->reference_surface){
        crossMeshCreator->setReferenceSurface(iodata->reference_surface);
    }
    if(iodata->pattern_mesh){
        crossMeshCreator->setPatternMesh(iodata->pattern_mesh);
    }

    //2) create the cross mesh
    if(iodata->cross_mesh != nullptr){
        crossMeshCreator->setCrossMesh(iodata->cross_mesh);
        crossMeshCreator->updateCrossMeshBoundary(iodata->varList->getIntList("boundary_crossIDs"));
    }
    else if(iodata->reference_surface != nullptr && iodata->pattern_mesh != nullptr){
        crossMeshCreator->createCrossMeshFromRSnPattern(false, init_textureMat);
        crossMeshCreator->createAugmentedVectors();
    }

    //3) create the structure
    if(crossMeshCreator->crossMesh){
        strucCreator = make_shared<StrucCreator<double>>(iodata->varList);
        strucCreator->compute(crossMeshCreator->crossMesh);
    }

    init_varlist_recompute_func();
}

void guiManager_TopoCreator::init_main_canvas(){
    if(crossMeshCreator){
        vector<shared_ptr<PolyMesh<double>>> meshLists;
        vector<nanogui::Color> colors;
        shared_ptr<CrossMesh<double>> crossMesh = crossMeshCreator->crossMesh;

        arcball_canvas->init_render_pass();
        //CrossMesh
        if(crossMesh)
        {
            colors.push_back(nanogui::Color(200, 200 ,200, 255));
            meshLists.push_back(crossMesh->getPolyMesh());
            shared_ptr<guiShader_MeshesWireFrame<double>> polyMeshLists
            = make_shared<guiShader_MeshesWireFrame<double>>(meshLists, colors, iodata->varList->getFloat("wireframe_linewidth"), arcball_canvas->scene->render_pass);
            polyMeshLists->visible = false;
            arcball_canvas->scene->objects.push_back(polyMeshLists);
        }

        //Augmented Vectors
        if(crossMesh)
        {
            vector<gui_LinesGroup<double>> linegroups;
            gui_LinesGroup<double> lg;
            lg.color = nanogui::Color(200, 100, 0, 255);
            for(int id = 0; id < crossMesh->size(); id++){
                shared_ptr<Cross<double>> cross = crossMesh->cross(id);
                for(int jd = 0; jd < cross->size(); jd++){
                    shared_ptr<OrientPoint<double>> ori = cross->ori(jd);
                    lg.lines.push_back(Line<double>(ori->point, ori->point + ori->normal * 0.03));
                }
            }
            linegroups.push_back(lg);
            shared_ptr<guiShader_Lines<double>> LinesObject = make_shared<guiShader_Lines<double>>(linegroups, iodata->varList->getFloat("augmentedvectors_linewidth"), arcball_canvas->scene->render_pass);
            LinesObject->visible = false;
            arcball_canvas->scene->objects.push_back(LinesObject);
        }

        //struc
        if(strucCreator)
        {
            meshLists.clear();
            colors.clear();
            for(auto block: strucCreator->blocks){
                if(block && block->polyMesh){
                    if(!block->at_boundary()){
                        colors.push_back(nanogui::Color(250, 250 ,250, 255));
                    }
                    else{
                        colors.push_back(nanogui::Color(50, 50 ,50, 255));
                    }
                    meshLists.push_back(block->polyMesh);
                }
            }
            shared_ptr<guiShader_MeshesWireFrame<double>> meshWireFrame
            = make_shared<guiShader_MeshesWireFrame<double>>(meshLists, colors, iodata->varList->getFloat("wireframe_linewidth"), arcball_canvas->scene->render_pass);
            arcball_canvas->scene->objects.push_back(meshWireFrame);
        }

        arcball_canvas->refresh_trackball_center();
    }
}

void guiManager_TopoCreator::init_pattern_canvas()
{
    vector<shared_ptr<PolyMesh<double>>> meshLists;
    vector<nanogui::Color> colors;
    shared_ptr<guiShader_PolyMeshes<double>> polyMeshShader;
    shared_ptr<guiShader_Lines<double>> linesShader;
    pattern_canvas->init_render_pass();

    //baseMesh2D
    {
        vector<gui_LinesGroup<double>> linegroups;

        if(crossMeshCreator->crossMesh->baseMesh2D)
        {
            shared_ptr<PolyMesh<double>> baseMesh = crossMeshCreator->crossMesh->baseMesh2D;
            gui_LinesGroup<double> lg;
            lg.color = nanogui::Color(242, 133, 0, 255);
            lg.lines = baseMesh->getWireFrame();
            linegroups.push_back(lg);
        }

        linesShader = make_shared<guiShader_Lines<double>>(linegroups, iodata->varList->getFloat("pattern_linewidth"), pattern_canvas->scene->render_pass);
        pattern_canvas->scene->objects.push_back(linesShader);
        linesShader->model_mat_fixed = true;
    }

    //Pattern2D
    {
        vector<gui_LinesGroup<double>> linegroups;
        if(crossMeshCreator->pattern2D)
        {
            shared_ptr<PolyMesh<double>> pattern_mesh = crossMeshCreator->pattern2D->getPolyMesh();
            gui_LinesGroup<double> lg;
            lg.color = nanogui::Color(150, 150, 150, 255);
            lg.lines = pattern_mesh->getWireFrame();
            linegroups.push_back(lg);
        }

        linesShader = make_shared<guiShader_Lines<double>>(linegroups, iodata->varList->getFloat("pattern_linewidth"), pattern_canvas->scene->render_pass);
        pattern_canvas->scene->objects.push_back(linesShader);
        linesShader->model_init_mat = init_textureMat.cast<float>();
    }

    //SurfaceTexture
    {
        colors.push_back(nanogui::Color(255, 255 ,255, 255));
        meshLists.clear();
        if(crossMeshCreator->referenceSurface)
        {
            shared_ptr<PolyMesh<double>> textureMesh =crossMeshCreator->referenceSurface->getTextureMesh();
            meshLists.push_back(textureMesh);
        }
        polyMeshShader = make_shared<guiShader_PolyMeshes<double>>(meshLists, colors, pattern_canvas->scene->render_pass);
        polyMeshShader->model_mat_fixed = true;
        pattern_canvas->scene->objects.push_back(polyMeshShader);
    }
    pattern_canvas->scene->focus_item = 2;
    pattern_canvas->refresh_trackball_center();
}


/*
 * Update Rendering
 */

void guiManager_TopoCreator::update_reference_surface_texture()
{
    vector<shared_ptr<PolyMesh<double>>> meshLists;
    vector<nanogui::Color> colors;
    shared_ptr<guiShader_PolyMeshes<double>> polyMeshLists;

    if(crossMeshCreator && crossMeshCreator->referenceSurface)
    {
        colors.push_back(nanogui::Color(255, 255 ,255, 255));
        shared_ptr<PolyMesh<double>> textureMesh = crossMeshCreator->referenceSurface->getTextureMesh();
        meshLists.push_back(textureMesh);
        ((guiShader_PolyMeshes<double> *)(pattern_canvas->scene->objects[2].get()))->update_mesh(meshLists, colors);
    }
}

void guiManager_TopoCreator::update_base_mesh_2D()
{
    vector<gui_LinesGroup<double>> linegroups;

    if(crossMeshCreator && crossMeshCreator->crossMesh && crossMeshCreator->crossMesh->baseMesh2D){
        shared_ptr<PolyMesh<double>> baseMesh = crossMeshCreator->crossMesh->baseMesh2D;
        gui_LinesGroup<double> lg;
        lg.color = nanogui::Color(242, 133, 0, 255);
        lg.lines = baseMesh->getWireFrame();
        linegroups.push_back(lg);
        ((guiShader_Lines<double> *)(pattern_canvas->scene->objects[0].get()))->update_line(linegroups);
    }

}

void guiManager_TopoCreator::update_pattern_2D()
{
    vector<gui_LinesGroup<double>> linegroups;
    if(crossMeshCreator && crossMeshCreator->pattern2D)
    {
        shared_ptr<PolyMesh<double>> pattern_mesh = crossMeshCreator->pattern2D->getPolyMesh();
        gui_LinesGroup<double> lg;
        lg.color = nanogui::Color(150, 150, 150, 255);
        lg.lines = pattern_mesh->getWireFrame();
        linegroups.push_back(lg);
        ((guiShader_Lines<double> *)(pattern_canvas->scene->objects[1].get()))->update_line(linegroups);
    }
}

void guiManager_TopoCreator::update_cross_mesh(){
    //CrossMesh
    vector<shared_ptr<PolyMesh<double>>> meshLists;
    vector<nanogui::Color> colors;

    if(crossMeshCreator && crossMeshCreator->crossMesh)
    {
        shared_ptr<CrossMesh<double>> crossMesh = crossMeshCreator->crossMesh;
        colors.push_back(nanogui::Color(200, 200 ,200, 255));
        meshLists.push_back(crossMesh->getPolyMesh());
        ((guiShader_MeshesWireFrame<double> *)(arcball_canvas->scene->objects[0].get()))->update_mesh(meshLists, colors);
    }
}

void guiManager_TopoCreator::update_augmented_vectors()
{
    vector<gui_LinesGroup<double>> linegroups;
    gui_LinesGroup<double> lg;

    if(crossMeshCreator && crossMeshCreator->crossMesh) {
        shared_ptr<CrossMesh<double>> crossMesh = crossMeshCreator->crossMesh;
        lg.color = nanogui::Color(200, 100, 0, 255);
        for (int id = 0; id < crossMesh->size(); id++) {
            shared_ptr<Cross<double>> cross = crossMesh->cross(id);
            for (int jd = 0; jd < cross->size(); jd++) {
                shared_ptr<OrientPoint<double>> ori = cross->ori(jd);
                lg.lines.push_back(Line<double>(ori->point, ori->point + ori->normal * 0.03));
            }
        }
        linegroups.push_back(lg);
        ((guiShader_Lines<double> *) (arcball_canvas->scene->objects[1].get()))->update_line(linegroups);
    }
}

void guiManager_TopoCreator::update_struc(){
    vector<shared_ptr<PolyMesh<double>>> meshLists;
    vector<nanogui::Color> colors;
    meshLists.clear();
    colors.clear();

    if(strucCreator){
        for(auto block: strucCreator->blocks){
            if(block && block->polyMesh){
                if(!block->at_boundary()){
                    colors.push_back(nanogui::Color(250, 250 ,250, 255));
                }
                else{
                    colors.push_back(nanogui::Color(50, 50 ,50, 255));
                }
                meshLists.push_back(block->polyMesh);
            }
        }
        ((guiShader_MeshesWireFrame<double> *)(arcball_canvas->scene->objects[2].get()))->update_mesh(meshLists, colors);
    }
}

void guiManager_TopoCreator::set_update_list_true(vector<std::string> strs)
{
    if(render_update_list){
        for(auto str: strs){
            render_update_list->add(true, str, "");
        }
    }
}

void guiManager_TopoCreator::update()
{
    Eigen::Matrix4d textureMat = pattern_canvas->get_textureMat();
    if((textureMat - last_textureMat).norm() > 1e-5) {
        last_textureMat = textureMat;
        recompute_cross_mesh();
    }

    if(iodata && iodata->varList)
    {
        for(std::string name: visible_var_nameList){
            InputVar *var = iodata->varList->find(name);
            if(var->update){
                if(var->func != nullptr) var->func();
                var->update = false;
            }
        }
    }

    for(shared_ptr<InputVar> var: render_update_list->varLists)
    {
        if(var->var_value_type == VAR_VALUE_BOOL
        && ((InputVarBool *)(var.get())) -> value
        && var->func != nullptr){
            var->func();
        }
    }

}

/*
 * update the geometry
 */

void guiManager_TopoCreator::recompute_cross_mesh()
{
    if(crossMeshCreator)
    {
        crossMeshCreator->createCrossMeshFromRSnPattern(false, last_textureMat);
        crossMeshCreator->createAugmentedVectors();
        set_update_list_true({"update_base_mesh_2D", "update_cross_mesh", "update_augmented_vectors"});
            if(crossMeshCreator->crossMesh){
            strucCreator->compute(crossMeshCreator->crossMesh);
            set_update_list_true({"update_struc"});
        }
    }
}

void guiManager_TopoCreator::recompute_augmented_vector_angle()
{
    if(crossMeshCreator){
        crossMeshCreator->updateAugmentedVectors();
        set_update_list_true({"update_base_mesh_2D"});
        if(crossMeshCreator->crossMesh){
            strucCreator->compute(crossMeshCreator->crossMesh);
            set_update_list_true({"update_struc"});
        }
    }
}

void guiManager_TopoCreator::recompute_reference_surface(){
    if(iodata && iodata->reference_surface && crossMeshCreator)
    {
        //1) reset referemce surface
        crossMeshCreator->setReferenceSurface(iodata->reference_surface);
        set_update_list_true({"update_reference_surface_texture"});

        //2) create the cross mesh
        if(crossMeshCreator->pattern2D != nullptr)
        {
            crossMeshCreator->createCrossMeshFromRSnPattern(false, last_textureMat);
            crossMeshCreator->createAugmentedVectors();
            set_update_list_true({"update_base_mesh_2D", "update_cross_mesh", "update_augmented_vectors"});
        }

        //3) create the structure
        if(crossMeshCreator->crossMesh)
        {
            strucCreator = make_shared<StrucCreator<double>>(iodata->varList);
            strucCreator->compute(crossMeshCreator->crossMesh);
            set_update_list_true({"update_struc"});
        }
    }
}

void guiManager_TopoCreator::recompute_pattern_mesh(){
    if(iodata && iodata->pattern_mesh && crossMeshCreator)
    {
        //1) reset referemce surface
        crossMeshCreator->updatePatternMesh();
        set_update_list_true({"update_pattern_2D"});

        //2) create the cross mesh
        if(crossMeshCreator->referenceSurface != nullptr)
        {
            crossMeshCreator->createCrossMeshFromRSnPattern(false, last_textureMat);
            crossMeshCreator->createAugmentedVectors();
            set_update_list_true({"update_base_mesh_2D", "update_cross_mesh", "update_augmented_vectors"});
        }

        //3) create the structure
        if(crossMeshCreator->crossMesh)
        {
            strucCreator = make_shared<StrucCreator<double>>(iodata->varList);
            strucCreator->compute(crossMeshCreator->crossMesh);
            set_update_list_true({"update_struc"});
        }
    }
}

void guiManager_TopoCreator::recompute_struc()
{
    if(crossMeshCreator->crossMesh)
    {
        strucCreator = make_shared<StrucCreator<double>>(iodata->varList);
        strucCreator->compute(crossMeshCreator->crossMesh);
        set_update_list_true({"update_struc"});
    }
}