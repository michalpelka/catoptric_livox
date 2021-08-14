#pragma once


class VertexBuffer
{
public:

	VertexBuffer(const void * data, unsigned int size);
	void update(const void * data, unsigned int size);

	void update();
	~VertexBuffer();
	void Bind() const;
	void Unbind() const;
private:
	unsigned int m_RenderID;
    const void * data ;
    const unsigned int size;

};