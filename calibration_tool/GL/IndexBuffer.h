#pragma once


class IndexBuffer
{
public:
	IndexBuffer(const unsigned int * data, unsigned int count);
	~IndexBuffer();
    void update(const unsigned int * data, unsigned int count);
	void Bind() const;
	void Unbind() const;
	inline unsigned int GetCount() const { return m_Count; }
private:
	unsigned int m_RenderID;
	unsigned int m_Count;

};