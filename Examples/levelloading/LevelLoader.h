//
// Created by private on 27.07.23.
//

#ifndef CFORGESANDBOX_LEVELLOADER_H
#define CFORGESANDBOX_LEVELLOADER_H

#include <string>
#include <flecs/addons/cpp/world.hpp>
#include <json/reader.h>
#include <json/value.h>
#include <iostream>
#include <regex>
#include "crossforge/AssetIO/T3DMesh.hpp"
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "crossforge/AssetIO/SAssetIO.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"
#include "crossforge/Graphics/Actors/StaticActor.h"

namespace CForge {
    class LevelLoader {
    public:
        void loadLevel(std::string filePath, SGNTransformation *rootNode, flecs::world *world) {
            std::string content = SAssetIO::readTextFile(filePath);
            Json::Reader reader;
            Json::Value level;
            reader.parse(content, level);
            const Json::Value &entities = level["entities"];
            for (int i = 0; i < entities.size(); i++) {
                Eigen::Vector3f position(entities[i]["position"]["x"].asFloat(),
                                         entities[i]["position"]["y"].asFloat(),
                                         entities[i]["position"]["z"].asFloat());
                Eigen::Quaternionf rotation;
                rotation = Eigen::AngleAxisf(entities[i]["rotation"]["x"].asFloat(), Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(entities[i]["rotation"]["y"].asFloat(), Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(entities[i]["rotation"]["z"].asFloat(), Eigen::Vector3f::UnitZ());
                Eigen::Vector3f scale(entities[i]["scale"]["x"].asFloat(),
                                      entities[i]["scale"]["y"].asFloat(),
                                      entities[i]["scale"]["z"].asFloat());


                SGNTransformation *obstacle_position = new SGNTransformation();
                obstacle_position->init(rootNode);
                obstacle_position->rotation(rotation);
                obstacle_position->scale(scale);
                obstacle_position->translation(position);
                SGNGeometry *obstacle_geom = new SGNGeometry();
                obstacle_geom->init(obstacle_position, getStaticActor("Assets/Models/" + entities[i]["path"].asString()));
            }
            //load static geometry
            SGNTransformation *static_geom_position = new SGNTransformation();
            static_geom_position->init(rootNode);
            SGNGeometry *static_geom = new SGNGeometry();
            std::string static_mesh_path = std::regex_replace(filePath, std::regex("json"), "gltf");
            static_geom->init(static_geom_position, getStaticActor(static_mesh_path));
        }


        StaticActor *getStaticActor(std::string filePath) {
            if (models.find(filePath) != models.end()) {
                return models.find(filePath)->second;
            }
            T3DMesh<float> M;
            StaticActor *actor = new StaticActor();
            SAssetIO::load(filePath, &M);
            setMeshShader(&M);
            M.computePerVertexNormals();
            M.computeAxisAlignedBoundingBox();
            actor->init(&M);
            M.clear();
            models.insert({filePath, actor});
            return actor;
        }


    protected:
        std::map<std::string, StaticActor *> models;

        static void setMeshShader(T3DMesh<float> *pM) {
            for (uint32_t i = 0; i < pM->materialCount(); ++i) {
                T3DMesh<float>::Material *pMat = pM->getMaterial(i);

                pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
                pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");

                pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
                pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");

                pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
                pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
            }
        }

    };
}
#endif //CFORGESANDBOX_LEVELLOADER_H