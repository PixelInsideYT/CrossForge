#include <glad/glad.h>
#include <GL/gl.h>
#include "GLBuffer.h"

namespace CForge {

	GLBuffer::GLBuffer(void): CForgeObject("GLBuffer") {
		m_GLID = GL_INVALID_INDEX;
		m_GLUsage = GL_STATIC_DRAW;
		m_GLTarget = GL_INVALID_INDEX;
		m_BufferType = BTYPE_UNKNOWN;
		m_BufferSize = 0;
	}//Constructor

	GLBuffer::~GLBuffer(void) {
		clear();
	}//Destructor

	void GLBuffer::init(BufferType Type, BufferUsage Usage, const void* pBufferData, uint32_t BufferSize ) {
		clear();
		glGenBuffers(1, &m_GLID);
		switch (Type) {
		case BTYPE_VERTEX: m_GLTarget = GL_ARRAY_BUFFER; break;
		case BTYPE_INDEX: m_GLTarget = GL_ELEMENT_ARRAY_BUFFER; break;
		case BTYPE_SHADER_STORAGE: m_GLTarget = GL_SHADER_STORAGE_BUFFER; break;
		case BTYPE_UNIFORM: m_GLTarget = GL_UNIFORM_BUFFER; break;
		default: {
			throw CForgeExcept("Invalid buffer type specified!");
		}break;
		}//switch[type]

		switch (Usage) {
		case BUSAGE_STATIC_DRAW: m_GLUsage = GL_STATIC_DRAW; break;
		case BUSAGE_DYNAMIC_DRAW: m_GLUsage = GL_DYNAMIC_DRAW; break;
		default: {
			throw CForgeExcept("Invalid buffer usage specified!");
		}break;
		}//switch[usage]

		m_BufferType = Type;

		if (BufferSize != 0) bufferData(pBufferData, BufferSize);

	}//initialize

	void GLBuffer::clear(void) {
		if (glIsBuffer(m_GLID)) {
			glDeleteBuffers(1, &m_GLID);
		}
		m_GLID = GL_INVALID_INDEX;
		m_BufferType = BTYPE_UNKNOWN;
	}//clear

	void GLBuffer::bind(void) {
		if (GL_INVALID_INDEX == m_GLID) throw CForgeExcept("Buffer is invalid. Has the class been initialized?");
		glBindBuffer(m_GLTarget, m_GLID);
	}//bind

	void GLBuffer::unbind(void) {
		glBindBuffer(m_GLTarget, 0);
	}//unbind

	GLBuffer::BufferType GLBuffer::type(void)const {
		return m_BufferType;
	}//type

	void GLBuffer::bufferData(const void* pBufferData, uint32_t BufferSize) {
		bind();
		glBufferData(m_GLTarget, BufferSize, pBufferData, m_GLUsage);
		m_BufferSize = BufferSize;
		unbind();
	}//bufferData

	void GLBuffer::bufferSubData(uint32_t Offset, uint32_t Payload, const void* pData) {
		bind();
		glBufferSubData(m_GLTarget, Offset, Payload, pData);
		unbind();

	}//bufferSubData

	uint32_t GLBuffer::size(void)const {
		return m_BufferSize;
	}//size

	void GLBuffer::bindBufferBase(uint32_t BindingPoint) {
		bind();

		glBindBufferBase(m_GLTarget, BindingPoint, m_GLID);
	}//bindBufferBase

}//name space