/*****************************************************************************\
*                                                                           *
* File(s): exampleMinimumGraphicsSetup.hpp                                            *
*                                                                           *
* Content: Example scene that shows minimum setup with an OpenGL capable   *
*          window, lighting setup, and a single moving object.              *
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
#ifndef __CFORGE_SCENE_HPP__
#define __CFORGE_SCENE_HPP__


#include "ExampleSceneBase.hpp"
#include <crossforge/Graphics/Actors/SkyboxActor.h>

using namespace Eigen;
using namespace std;

namespace CForge {

	class Scene : public ExampleSceneBase {
	public:
		Scene(void) {
			m_WindowTitle = "CrossForge Example - Minimum Graphics Setup";
			m_WinWidth = 1280;
			m_WinHeight = 720;
		}//Constructor

		~Scene(void) {
			clear();
		}//Destructor

		void init() override {

			initWindowAndRenderDevice();
			initCameraAndLights();
			initFPSLabel();

			// build scene graph
			m_RootSGN.init(nullptr);
			m_SG.init(&m_RootSGN);

			initGroundPlane(&m_RootSGN, 100.0f, 20.0f);

			T3DMesh<float> M;
			SAssetIO::load("Assets/Scene/scene.gltf", &M);
			for (uint32_t i = 0; i < M.materialCount(); ++i) CForgeUtility::defaultMaterial(M.getMaterial(i), CForgeUtility::PLASTIC_WHITE);
			M.computePerVertexNormals();
			m_Scene.init(&M);
			M.clear();

			// add scene
			m_SceneTransformSGN.init(&m_RootSGN, Vector3f(0.0f, 0.0f, 0.0f));
			m_SceneSGN.init(&m_SceneTransformSGN, &m_Scene);

			// add skybox
			initSkybox();

			// create help text
			LineOfText* pKeybindings = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement: (Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text");
			m_HelpTexts.push_back(pKeybindings);
			m_DrawHelpTexts = true;

			std::string ErrorMsg;
			if (0 != CForgeUtility::checkGLError(&ErrorMsg)) {
				SLogger::log("OpenGL Error" + ErrorMsg, "PrimitiveFactoryTestScene", SLogger::LOGTYPE_ERROR);
			}

		}//initialize

		void clear(void) override {
			m_RenderWin.stopListening(this);
			if (nullptr != m_pShaderMan) m_pShaderMan->release();
			m_pShaderMan = nullptr;
		}//clear


		void mainLoop(void)override {
			m_RenderWin.update();
			m_SG.update(60.0f / m_FPS);
			m_SkyboxSG.update(60.0f / m_FPS);

			defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);
			m_FPSLabel.render(&m_RenderDev);
			if (m_DrawHelpTexts) drawHelpTexts();

			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
			m_SkyboxSG.render(&m_RenderDev);

			m_RenderWin.swapBuffers();

			updateFPS();

			defaultKeyboardUpdate(m_RenderWin.keyboard());

		}//mainLoop

	protected:

		// Scene Graph
		SGNTransformation m_RootSGN;

		StaticActor m_Scene;
		SGNGeometry m_SceneSGN;
		SGNTransformation m_SceneTransformSGN;

		vector<string> m_ClearSky;
		SkyboxActor m_Skybox;

	};//ExampleMinimumGraphicsSetup

}//name space

#endif