/*****************************************************************************\
*                                                                           *
* File(s): LODTestScene.hpp                                            *
*                                                                           *
* Content: Class to interact with an MF52 NTC Thermistor by using a basic   *
*          voltage divider circuit.                                         *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_LODTESTSCENE_HPP__
#define __CFORGE_LODTESTSCENE_HPP__

#include "../../CForge/AssetIO/SAssetIO.h"
#include "../../CForge/Graphics/Shader/SShaderManager.h"
#include "../../CForge/Graphics/STextureManager.h"

#include "../../CForge/Graphics/GLWindow.h"
#include "../../CForge/Graphics/GraphicsUtility.h"
#include "../../CForge/Graphics/RenderDevice.h"

#include "../../CForge/Graphics/Lights/DirectionalLight.h"

#include "../../CForge/Graphics/SceneGraph/SceneGraph.h"
#include "../../CForge/Graphics/SceneGraph/SGNGeometry.h"
#include "../../CForge/Graphics/SceneGraph/SGNTransformation.h"

#include "../../CForge/Graphics/Actors/StaticActor.h"

#include "../../Examples/SceneUtilities.hpp"

#include "../Actor/LODActor.h"

#include "../LODHandler.h"
#include "../SLOD.h"
#include <chrono>

using namespace Eigen;
using namespace std;

namespace CForge {

	//void setMeshShader(T3DMesh<float>* pM, float Roughness, float Metallic) {
	//	for (uint32_t i = 0; i < pM->materialCount(); ++i) {
	//		T3DMesh<float>::Material* pMat = pM->getMaterial(i);
	//		pMat->VertexShaderSources.push_back("Shader/BasicGeometryPass.vert");
	//		pMat->FragmentShaderSources.push_back("Shader/BasicGeometryPass.frag");
	//		pMat->Metallic = Metallic;
	//		pMat->Roughness = Roughness;
	//	}//for[materials]
	//}//setMeshShader

	void LODTestScene(void) {
		SAssetIO* pAssIO = SAssetIO::instance();
		STextureManager* pTexMan = STextureManager::instance();
		SShaderManager* pSMan = SShaderManager::instance();
		SLOD* pLOD = SLOD::instance();
		
		bool const LowRes = true;

		uint32_t WinWidth = 1280;
		uint32_t WinHeight = 720;

		if (LowRes) {
			WinWidth = 1440;//720;
			WinHeight = 1080-80;//576;
		}

		GLWindow RenderWin;
		RenderWin.init(Vector2i(100, 100), Vector2i(WinWidth, WinHeight), "Absolute Minimum Setup");
		gladLoadGL();
		glfwSwapInterval(0);
		
		std::string GLError;
		GraphicsUtility::checkGLError(&GLError);
		if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());

		RenderDevice RDev;
		RenderDevice::RenderDeviceConfig Config;
		Config.DirectionalLightsCount = 1;
		Config.PointLightsCount = 0;
		Config.SpotLightsCount = 0;
		Config.ExecuteLightingPass = true;
		Config.GBufferHeight = WinHeight;
		Config.GBufferWidth = WinWidth;
		Config.pAttachedWindow = &RenderWin;
		Config.PhysicallyBasedShading = true;
		Config.UseGBuffer = true;
		RDev.init(&Config);

		ShaderCode::LightConfig LC;
		LC.DirLightCount = 1;
		LC.PCFSize = 1;
		LC.PointLightCount = 0;
		LC.ShadowBias = 0.0004f;
		LC.ShadowMapCount = 1;
		LC.SpotLightCount = 0;
		pSMan->configShader(LC);


		VirtualCamera Cam;
		Cam.init(Vector3f(0.0f, 0.0f, 5.0f), Vector3f::UnitY());
		Cam.projectionMatrix(WinWidth, WinHeight, GraphicsUtility::degToRad(90.0f), 0.1f, 1000.0f);

		Vector3f SunPos = Vector3f(5.0f, 25.0f, 25.0f);
		DirectionalLight Sun;
		Sun.init(SunPos, -SunPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 5.0f);

		RDev.activeCamera(&Cam);
		RDev.addLight(&Sun);

		SceneGraph SGTest;
		SGNGeometry CubeSGN;
		SGNTransformation CubeTransformSGN;
		LODActor Cube;

		T3DMesh<float> M;
		SAssetIO::load("Assets/tree0.obj", &M);
		
		LODHandler lodHandler;
		lodHandler.generateLODmodels("Assets/tree0.obj");
		
		SceneUtilities::setMeshShader(&M, 0.1f, 0.04f);
		M.computePerVertexNormals();
		Cube.init(&M);

		CubeTransformSGN.init(nullptr);
		CubeSGN.init(&CubeTransformSGN, &Cube);
		SGTest.init(&CubeTransformSGN);

		// rotate about the y-axis at 45 degree every second
		Quaternionf R;
		R = AngleAxisf(GraphicsUtility::degToRad(45.0f / 60.0f), Vector3f::UnitY());
		CubeTransformSGN.rotationDelta(R);
		CubeTransformSGN.translation(Vector3f(0.0, -5.0, 0.0));

		int64_t LastFPSPrint = GetTickCount();
		int32_t FPSCount = 0;

		bool Wireframe = true;
		
		glLineWidth(GLfloat(1.0f));
		
		while (!RenderWin.shutdown()) {
			RenderWin.update();
			pLOD->update();

			R = AngleAxisf(GraphicsUtility::degToRad(45.0f*100.0f / 60.0f), Vector3f::UnitY());
			CubeTransformSGN.rotationDelta(R);
			
			SGTest.update(1.0f*pLOD->deltaTime);
			
			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_1, true)) {
				Wireframe = !Wireframe;
			}
			
			if (Wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			RDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			SGTest.render(&RDev);
			
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			
			RDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

			RenderWin.swapBuffers();

			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_ESCAPE)) {
				RenderWin.closeWindow();
			}
			printf("deltaTime: %f\tFPS:%f \n", pLOD->deltaTime, 1.0/pLOD->deltaTime);
		}//while[main loop]


		pAssIO->release();
		pTexMan->release();
		pSMan->release();
		pLOD->release();
	}//PITestScene

}

#endif