/*****************************************************************************\
*                                                                           *
* File(s): EDT.hpp                                            *
*                                                                           *
* Content: Example scene that shows how to use the scene graph to create    *
*          dynamic scene descriptions.                                      *
*                                                                           *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_EDT_HPP__
#define __CFORGE_EDT_HPP__

#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>
#include "ExampleSceneBase.hpp"
#include "Examples/edt/AIComponent.h"
#include "Examples/edt/AiSystem.h"
#include <flecs.h>
#include <iostream>

namespace CForge {
    class EDT : public ExampleSceneBase {
    public:
        EDT(void) {

        }//Constructor

        ~EDT(void) {
            clear();
        }//Destructor

        void init(void) override {
            initWindowAndRenderDevice();
            initCameraAndLights();

            initSkybox();
            initFPSLabel();
            m_FPSLabel.color(1.0f, 1.0f, 1.0f, 1.0f);

            m_RootSGN.init(nullptr);
            m_SG.rootNode(&m_RootSGN);

            // initialize ground plane
            T3DMesh<float> M;

            // load the ground model
            //SAssetIO::load("Assets/ExampleScenes/TexturedGround.gltf", &M);
            PrimitiveShapeFactory::plane(&M, Vector2f(1250.0f, 1250.0f), Vector2i(10, 10));
            setMeshShader(&M, 0.6f, 0.2f);
            M.changeUVTiling(Vector3f(250.0f, 250.0f, 1.0f));
            M.computePerVertexNormals();
            M.computePerVertexTangents();
            M.getMaterial(0)->TexAlbedo = "Assets/ExampleScenes/Textures/Ground003/Ground003_2K_Color.webp";
            M.getMaterial(0)->TexNormal = "Assets/ExampleScenes/Textures/Ground003/Ground003_2K_NormalGL.webp";
            m_Ground.init(&M);
            BoundingVolume BV;
            m_Ground.boundingVolume(BV);
            M.clear();

            // initialize ground transformation and geometry scene graph node
            m_GroundTransformSGN.init(&m_RootSGN);
            m_GroundSGN.init(&m_GroundTransformSGN, &m_Ground);

            // load the tree models
            SAssetIO::load("Assets/placeholder/giesroboter.glb", &M);
            setMeshShader(&M, 0.8f, 0.04f);
            M.computePerVertexNormals();
            scaleAndOffsetModel(&M, 0.5f);
            M.computeAxisAlignedBoundingBox();
            m_Trees[0].init(&M);
            M.clear();

            SAssetIO::load("Assets/ExampleScenes/Trees/LowPolyTree_02.gltf", &M);
            setMeshShader(&M, 0.8f, 0.04f);
            M.computePerVertexNormals();
            M.computeAxisAlignedBoundingBox();
            m_Trees[1].init(&M);
            M.clear();

            SAssetIO::load("Assets/ExampleScenes/Trees/LowPolyTree_03.gltf", &M);
            setMeshShader(&M, 0.8f, 0.04f);
            M.computePerVertexNormals();
            scaleAndOffsetModel(&M, 5.0f, Vector3f(0.0f, 0.25f, 0.0f));
            M.computeAxisAlignedBoundingBox();
            m_Trees[2].init(&M);
            M.clear();

            // sceen graph node that holds our forest
            m_TreeGroupSGN.init(&m_RootSGN);

            float Area = 500.0f;    // square area [-Area, Area] on the xz-plane, where trees are planted
            float TreeCount = 1;    // number of trees to create

            for (uint32_t i = 0; i < TreeCount; ++i) {
                // create the scene graph nodes
                SGNGeometry *pGeomSGN = nullptr;
                SGNTransformation *pTransformSGN = nullptr;

                // initialize position and scaling of the tree
                pTransformSGN = new SGNTransformation();
                pTransformSGN->init(&m_TreeGroupSGN);

                float TreeScale = CForgeMath::randRange(0.1f, 3.0f);

                Vector3f TreePos = Vector3f::Zero();
                TreePos.x() = CForgeMath::randRange(-Area, Area);
                TreePos.z() = CForgeMath::randRange(-Area, Area);

                pTransformSGN->translation(TreePos);
                pTransformSGN->scale(Vector3f(TreeScale, TreeScale, TreeScale));

                // initialize geometry
                // choose one of the trees randomly
                pGeomSGN = new SGNGeometry();
                uint8_t TreeType = CForgeMath::rand() % 3;
                pGeomSGN->init(pTransformSGN, &m_Trees[TreeType]);

                m_TreeTransformSGNs.push_back(pTransformSGN);
                m_TreeSGNs.push_back(pGeomSGN);

            }//for[TreeCount]

            // change sun settings to cover this large area
            m_Sun.position(Vector3f(100.0f, 1000.0f, 500.0f));
            m_Sun.initShadowCasting(2048 * 2, 2048 * 2, Vector2i(1000, 1000), 1.0f, 5000.0f);

            m_Fly = false;

            // create help text
            LineOfText *pKeybindings = new LineOfText();
            pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18),
                               "Movement:(Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text");
            m_HelpTexts.push_back(pKeybindings);
            pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
            m_DrawHelpTexts = true;

            roboter = world.entity();
            roboter.add<AIComponent>();
            roboter.add<SGNTransformation>();
            auto transformation = roboter.get_mut<SGNTransformation>();
            transformation->init(&m_RootSGN);
            SGNGeometry *entityGeom = new SGNGeometry();
            entityGeom->init(transformation, &m_Trees[0]);
            AiSystem::addAiSystem(world);

        }//initialize

        void clear(void) override {
            for (auto &i: m_TreeSGNs) if (nullptr != i) delete i;
            for (auto &i: m_TreeTransformSGNs) if (nullptr != i) delete i;

            ExampleSceneBase::clear();
        }//clear

        void mainLoop(void) override {
            m_RenderWin.update();

            defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse(), 0.1f * 60.0f / m_FPS, 0.5f, 2.0f);
            // make sure to always walk on the ground if not flying
            if (!m_Fly) {
                Vector3f CamPos = m_Cam.position();
                CamPos.y() = 1.0f;
                m_Cam.position(CamPos);
            }

            if (!roboter.get_mut<AIComponent>()->path.empty())
                m_TreeTransformSGNs.front()->translation(roboter.get_mut<AIComponent>()->path.front());

            m_SkyboxSG.update(60.0f / m_FPS);
            m_SG.update(60.0f / m_FPS);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
            m_RenderDev.activeCamera(const_cast<VirtualCamera *>(m_Sun.camera()));
            m_SG.render(&m_RenderDev);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
            m_RenderDev.activeCamera(&m_Cam);
            m_SG.render(&m_RenderDev);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
            m_SkyboxSG.render(&m_RenderDev);
            if (m_FPSLabelActive) m_FPSLabel.render(&m_RenderDev);
            if (m_DrawHelpTexts) drawHelpTexts();

            m_RenderWin.swapBuffers();

            updateFPS();
            world.progress();
            // change between flying and walking mode
            if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_F, true)) m_Fly = !m_Fly;

            defaultKeyboardUpdate(m_RenderWin.keyboard());

        }//run

    protected:

        void scaleAndOffsetModel(T3DMesh<float> *pModel, float Factor, Vector3f Offset = Vector3f::Zero()) {
            Matrix3f Sc = Matrix3f::Identity();
            Sc(0, 0) = Factor;
            Sc(1, 1) = Factor;
            Sc(2, 2) = Factor;
            for (uint32_t i = 0; i < pModel->vertexCount(); ++i) pModel->vertex(i) = Sc * pModel->vertex(i) - Offset;
        }//scaleModel
        flecs::world world;
        flecs::entity roboter;
        flecs::system move_sys;
        SGNTransformation m_RootSGN;

        StaticActor m_Ground;
        SGNGeometry m_GroundSGN;
        SGNTransformation m_GroundTransformSGN;

        StaticActor m_Trees[3];
        std::vector<SGNTransformation *> m_TreeTransformSGNs;
        std::vector<SGNGeometry *> m_TreeSGNs;

        SGNTransformation m_TreeGroupSGN;

        bool m_Fly;
    };//EDT

}//name space

#endif 