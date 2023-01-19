#include "StickFigureActor.h"
#include "../../AssetIO/SAssetIO.h"
#include "../GraphicsUtility.h"

using namespace Eigen;

namespace CForge {

	StickFigureActor::StickFigureActor(void) {
		m_ClassName = "StickFigureActor";

		m_JointSize = 0.1f;
		m_BoneSize = 0.2f;
		m_JointColor = Vector4f::Ones();
		m_BoneColor = Vector4f(0.0f, 0.0f, 1.0f, 1.0f);

	}//Constructor

	StickFigureActor::~StickFigureActor(void) {
		clear();
	}//Destructor

	void StickFigureActor::init(T3DMesh<float>* pMesh, SkeletalAnimationController* pController) {
		if (nullptr == pMesh) throw NullpointerExcept("pMesh");
		if (nullptr == pController) throw NullpointerExcept("pController");
		if (pMesh->boneCount() == 0) throw CForgeExcept("Mesh contains no skeleton. Creation of StickFigureActor not possible!");

		m_pAnimationController = pController;

		// initialize the actors
		T3DMesh<float> M;
		AssetIO::load("MyAssets/UnitSphere.glb", &M);
		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			buildMaterial(pMat);
		}
		M.computePerVertexNormals();
		m_Joint.init(&M);
		M.clear();

		AssetIO::load("MyAssets/UnitCylinder.glb", &M);
		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			buildMaterial(pMat);
		}
		M.computePerVertexNormals();
		M.computeAxisAlignedBoundingBox();
		float Offset = -M.aabb().Min.y();
		Matrix4f T = GraphicsUtility::translationMatrix(Vector3f(0.0f, Offset, 0.0f));
		M.applyTransformation(T);

		m_Bone.init(&M);
		M.clear();

		// initialize scene graph
		m_RootSGN.init(nullptr);
		m_SG.init(&m_RootSGN);

		m_JointValues = m_pAnimationController->retrieveSkeleton();
		for (uint32_t i = 0; i < m_JointValues.size(); ++i) {
			SGNTransformation* pTransform = new SGNTransformation();
			SGNGeometry* pGeomJoint = new SGNGeometry();
			SGNGeometry* pGeomBone = new SGNGeometry();

			m_JointTransformSGNs.push_back(pTransform);
			m_JointSGNs.push_back(pGeomJoint);
			m_BoneSGNs.push_back(pGeomBone);
		}//for[all bones]

		// estimate a good value for the joint and bone sizes
		Vector3f Diag = (pMesh->aabb().diagonal().norm() < 0.0001f) ? T3DMesh<float>::computeAxisAlignedBoundingBox(pMesh).diagonal() : pMesh->aabb().diagonal();

		m_JointSize = Diag.norm() / 75.0f;
		m_BoneSize = m_JointSize / 3.0f;

		// create Scene graph nodes for all joints
		createBone(pMesh->rootBone(), &m_RootSGN);
		jointColor(m_JointColor);
		boneColor(m_BoneColor);
		jointSize(m_JointSize);
		boneSize(m_BoneSize);
	}//initialize

	void StickFigureActor::clear(void) {
		SkeletalActor::clear();

		m_JointSize = 0.0f;
		m_BoneSize = 0.0f;
		m_JointColor = Vector4f::Ones();
		m_BoneColor = Vector4f(0.0f, 0.0f, 1.0f, 1.0f);

		for (auto& i : m_JointValues) delete i;
		m_JointValues.clear();
	}//clear

	void StickFigureActor::release(void) {
		delete this;
	}//release

	void StickFigureActor::jointColor(const Vector4f Col) {
		for (uint32_t i = 0; i < m_Joint.materialCount(); ++i) {
			auto* pMat = m_Joint.material(i);
			pMat->color(Col);
		}
		m_JointColor = Col;
	}//jointColor

	void StickFigureActor::boneColor(const Vector4f Col) {
		for (uint32_t i = 0; i < m_Bone.materialCount(); ++i) {
			auto* pMat = m_Bone.material(i);
			pMat->color(Col);
		}
		m_BoneColor = Col;
	}//boneColor

	void StickFigureActor::jointSize(float Size) {
		for (auto i : m_JointSGNs) i->scale(Vector3f(Size, Size, Size));
		m_JointSize = Size;
	}//jointSize

	void StickFigureActor::boneSize(float Size) {
		for (auto i : m_BoneSGNs) i->scale(Vector3f(Size, 1.0f, Size));
		m_BoneSize = Size;
	}//boneSize

	Eigen::Vector4f StickFigureActor::jointColor(void)const {
		return m_JointColor;
	}//jointColor

	Eigen::Vector4f StickFigureActor::boneColor(void)const {
		return m_BoneColor;
	}//boneColor

	float StickFigureActor::jointSize(void)const {
		return m_JointSize;
	}//joinSize

	float StickFigureActor::boneSize(void)const {
		return m_BoneSize;
	}//boneSize


	void StickFigureActor::render(class RenderDevice* pRDev, Eigen::Quaternionf Rotation, Eigen::Vector3f Translation, Eigen::Vector3f Scale) {

		if (nullptr != m_pActiveAnimation && m_pActiveAnimation->Finished) {
			m_pAnimationController->destroyAnimation(m_pActiveAnimation);
			m_pActiveAnimation = nullptr;
		}

		// set current animation data 
		// if active animation is nullptr bind pose will be set
		m_pAnimationController->applyAnimation(m_pActiveAnimation, true);

		m_pAnimationController->updateSkeletonValues(&m_JointValues);

		for (auto i : m_JointValues) {
			m_JointTransformSGNs[i->ID]->translation(Scale.cwiseProduct(i->LocalPosition));
			m_JointTransformSGNs[i->ID]->rotation(i->LocalRotation);
			m_JointTransformSGNs[i->ID]->scale(i->LocalScale);
		}
		m_RootSGN.translation(Translation);
		m_RootSGN.rotation(Rotation);
		m_RootSGN.scale(Scale);

		// compute bone transformations
		for (auto i : m_JointValues) {
			if (i->Parent == -1) continue; // we don't care about the root node

			Vector3f BoneVec = m_JointValues[i->ID]->LocalPosition;
			Matrix3f BoneOrientation = GraphicsUtility::alignVectors(Vector3f::UnitY(), BoneVec.normalized());
			Quaternionf R = m_JointValues[i->Parent]->LocalRotation;
			R.inverse();
			R = BoneOrientation;
			m_BoneSGNs[i->ID]->rotation(R);

			float Length = BoneVec.norm();
			Vector3f s = m_BoneSGNs[i->ID]->scale();
			s.y() = Length;
			m_BoneSGNs[i->ID]->scale(s);

		}

		m_SG.update(1.0f);
		m_SG.render(pRDev);
	}//render

	void StickFigureActor::buildMaterial(T3DMesh<float>::Material* pMat) {
		pMat->Color = Vector4f(0.0f, 0.0f, 1.0f, 1.0f);
		pMat->Metallic = 0.3f;
		pMat->Roughness = 0.2f;
		pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
		pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
		pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
		pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");
		pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
		pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");
	}//buildMaterial

	void StickFigureActor::createBone(T3DMesh<float>::Bone* pBone, SGNTransformation* pParent) {

		SGNTransformation* pTransSGN = m_JointTransformSGNs[pBone->ID];
		SGNGeometry* pGeomSGN = m_JointSGNs[pBone->ID];

		pTransSGN->init(pParent);
		pGeomSGN->init(pTransSGN, &m_Joint);
		for (auto i : pBone->Children) {

			pGeomSGN = m_BoneSGNs[i->ID];
			pGeomSGN->init(pTransSGN, &m_Bone);
			createBone(i, pTransSGN);
		}

	}//createBone

}//name space