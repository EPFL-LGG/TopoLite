//
// Created by ziqwang on 19.05.20.
//

#ifndef TOPOLITE_GUIMANAGER_TOPOCREATOR_H
#define TOPOLITE_GUIMANAGER_TOPOCREATOR_H

#include "IO/JsonIOReader.h"
#include "IO/JsonIOWriter.h"
#include "IO/XMLIO_backward.h"
#include "guiCanvas_3DArcball.h"
#include "guiCanvas_2DArcball.h"
#include "guiShader_PolyMeshes.h"
#include "guiShader_Lines.h"

#include "CrossMesh/CrossMeshCreator.h"
#include "Structure/StrucCreator.h"

#include "nanogui/object.h"
#include <memory>
#include <Eigen/Dense>

class guiManager_TopoCreator{
public:

public:
    shared_ptr<IOData> iodata;
    shared_ptr<CrossMeshCreator<double>> crossMeshCreator;
    shared_ptr<StrucCreator<double>> strucCreator;

public:
    Eigen::Matrix4d init_textureMat;
    Eigen::Matrix4d last_textureMat;
    shared_ptr<InputVarList> render_update_list;
    vector<std::string> visible_var_nameList;

public:
    nanogui::ref<guiCanvas_3DArcball> arcball_canvas;
    nanogui::ref<guiCanvas_2DArcball> pattern_canvas;

public:
    guiManager_TopoCreator(nanogui::ref<guiCanvas_3DArcball> arcball, nanogui::ref<guiCanvas_2DArcball> pattern)
    {
        arcball_canvas = arcball;
        pattern_canvas = pattern;
        visible_var_nameList = {"tiltAngle",
                                "patternID", "patternRadius",
                                "layerOfBoundary", "minCrossArea", "smooth_bdry", "ground_touch_bdry",
                                "cutUpper", "cutLower"};

        iodata = make_shared<IOData>();
        clear();
    }

public:

public: /* Input and Output */

    bool load_from_xmlfile(string xmlFileName);

    bool load_from_jsonfile(string xmlFileName);

    void write_to_json(string jsonFileName);

public: /* initialization */

    void init_from_IOData();

    void init_main_canvas();

    void init_pattern_canvas();
    
public: /* update the geometry */
    
    void recompute_cross_mesh();

    void recompute_augmented_vector_angle();

    void recompute_reference_surface();

    void recompute_pattern_mesh();

    void recompute_struc();

public: /* update the rendering */

    void update_reference_surface_texture();

    void update_base_mesh_2D();

    void update_pattern_2D();

    void update_cross_mesh();

    void update_augmented_vectors();

    void update_struc();

    void set_update_list_true(vector<std::string> strs);
    
    void update();

private:

    void clear(){
        init_textureMat = Eigen::Matrix4d::Identity();
        last_textureMat = Eigen::Matrix4d::Identity();
        init_render_update_list();
    }

    void init_render_update_list()
    {
        render_update_list = make_shared<InputVarList>();
        render_update_list->add((bool)false, "update_cross_mesh",  "").func = [&](){
            update_cross_mesh();
        };
        render_update_list->add((bool)false, "update_augmented_vectors",  "").func = [&](){
            update_augmented_vectors();
        };
        render_update_list->add((bool)false, "update_base_mesh_2D",  "").func = [&](){
            update_base_mesh_2D();
        };
        render_update_list->add((bool)false, "update_struc",  "").func = [&](){
            update_struc();
        };
        render_update_list->add((bool)false, "update_pattern_2D",  "").func = [&](){
            update_pattern_2D();
        };
        render_update_list->add((bool)false, "update_reference_surface_texture",  "").func = [&](){
            update_reference_surface_texture();
        };
    }

    void init_varlist_recompute_func()
    {
        if(iodata && iodata->varList){
            auto reference_surface = [&](){this->recompute_reference_surface();};
            auto pattern_2D = [&](){this->recompute_pattern_mesh();};
            auto cross_mesh = [&](){this->recompute_cross_mesh();};
            auto augment_vectors = [&](){this->recompute_augmented_vector_angle();};
            auto struc = [&](){this->recompute_struc();};

            vector<std::function<void()>> funcList = {augment_vectors,
                                                      pattern_2D, pattern_2D,
                                                      cross_mesh, cross_mesh, cross_mesh, cross_mesh,
                                                      struc, struc
            };

            for(size_t id = 0; id < visible_var_nameList.size(); id++)
            {
                std::string name = visible_var_nameList[id];
                InputVar *var = iodata->varList->find(name);
                if(var){
                    var ->func = funcList[id];
                }
            }
        }
    }

    Eigen::Matrix4d toEigenMatrix(double *interactMatrix){
        Eigen::Matrix4d interactMat;
        interactMat << interactMatrix[0], interactMatrix[4], interactMatrix[8], interactMatrix[12],
            interactMatrix[1], interactMatrix[5], interactMatrix[9], interactMatrix[13],
            interactMatrix[2], interactMatrix[6], interactMatrix[10], interactMatrix[14],
            interactMatrix[3], interactMatrix[7], interactMatrix[11], interactMatrix[15];
        return interactMat;
    }
};


#endif //TOPOLITE_GUIMANAGER_TOPOCREATOR_H
