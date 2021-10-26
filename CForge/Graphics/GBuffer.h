/*****************************************************************************\
*                                                                           *
* File(s): GBuffer.h and GBuffer.cpp                                     *
*                                                                           *
* Content:    *
*          .                                         *
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
#ifndef __CFORGE_GBUFFER_H__
#define __CFORGE_GBUFFER_H__

#include "../Core/CForgeObject.h"
#include "../AssetIO/T2DImage.hpp"

namespace CForge{ 
	/**
	* \brief Global buffer object. Stores rendering data (position, normal, albedo) and provides it by textures.
	*
	* \todo Do full documentation.
	*/
	class CFORGE_IXPORT GBuffer: public CForgeObject {
	public:
		enum Component: int8_t {
			COMP_POSITION = 0,
			COMP_NORMAL,
			COMP_ALBEDO
		};

		GBuffer(void);
		~GBuffer(void);

		void init(uint32_t Width, uint32_t Height);
		void clear(void);

		void bind(void)const;
		void unbind(void)const;

		void bindTexture(Component Comp, uint32_t Level);
		void blitDepthBuffer(uint32_t Width, uint32_t Height);

		uint32_t width(void)const;
		uint32_t height(void)const;

		void retrievePositionBuffer(T2DImage<uint8_t> *pImg);
		void retrieveNormalBuffer(T2DImage<uint8_t>* pImg);
		void retrieveAlbedoBuffer(T2DImage<uint8_t>* pImg);

	protected:
		uint32_t m_Framebuffer;
		uint32_t m_Renderbuffer;

		uint32_t m_TexPosition;
		uint32_t m_TexNormal;
		uint32_t m_TexAlbedo;

		uint32_t m_Width;
		uint32_t m_Height;
	};//GBuffer

}//name space


#endif 