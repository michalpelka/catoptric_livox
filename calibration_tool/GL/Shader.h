#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "glm/glm.hpp"
struct ShaderProgramSource
{
	std::string VertexSource;
	std::string FragmentSource;
};
class Shader
{
private: 
	unsigned int m_RenderID;
	std::string m_FilePath;
	mutable std::unordered_map<std::string, int> m_UniformLocationCache;
public:
	Shader(const std::string& filepath);
    Shader(const std::string& vertex, const std::string& fragment);

	~Shader();
	void Bind() const;
	void Unbind() const;
	void setUniform1i(const std::string& name, int v);
	void setUniform4f(const std::string& name, float v0, float v1, float v2, float v3);
	void setUniformMat4f(const std::string& name, const glm::mat4& matrix);
    void setUniformMat4f(const std::string& name, const float * matrix);
	void setUniform1iv(const std::string& name, const std::vector<int> & v);

private:
	unsigned int GetUniformLocation(const std::string& name) const;
	unsigned int CompileShader(unsigned int type, const std::string& source);
	unsigned int CreateShader(const std::string &vertexShader, const std::string& fragmentShader);
	ShaderProgramSource ParseShader(const std::string& filepath);
	std::vector<std::string> ParseShaderForUniforms(const std::string& filepath);
};
