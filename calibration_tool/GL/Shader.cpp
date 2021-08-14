#include "Shader.h"
#include "Renderer.h"
#include <fstream>
#include <sstream>
#include <iostream>

Shader::Shader(const std::string& filepath):
	m_FilePath(filepath), m_RenderID(0)
{
	ShaderProgramSource source = ParseShader(filepath);
	const auto uniform_names = ParseShaderForUniforms(filepath);
	m_RenderID = CreateShader(source.VertexSource, source.FragmentSource);
	for (const auto &s : uniform_names)
	{
		GetUniformLocation(s);
	}
}

Shader::Shader(const std::string& vertex, const std::string& fragment)
{
    m_RenderID = CreateShader(vertex, fragment);
//    for (const auto &s : uniform_names)
//    {
//        GetUniformLocation(s);
//    }
}

Shader::~Shader()
{
	GLCall(glDeleteProgram(m_RenderID));
}
void Shader::Bind() const
{
	GLCall(glUseProgram(m_RenderID));
}
void Shader::Unbind() const
{
	GLCall(glUseProgram(0));
}
void Shader::setUniform1i(const std::string& name, int v)
{
	int location = GetUniformLocation(name);
	GLCall(glUniform1i(location, v));
}
void Shader::setUniform1iv(const std::string& name, const std::vector<int> & v)
{
	int location = GetUniformLocation(name);
	GLCall(glUniform1iv(location, v.size(), v.data()));
}
void Shader::setUniform4f(const std::string& name, float v0, float v1, float v2, float v3)
{
	int location = GetUniformLocation(name);
	GLCall(glUniform4f(location, v0,v1,v2,v3));
}

void Shader::setUniformMat4f(const std::string& name, const glm::mat4& matrix)
{
	int location = GetUniformLocation(name);
	GLCall(glUniformMatrix4fv(location, 1, GL_FALSE, &matrix[0][0]));
}
void Shader::setUniformMat4f(const std::string& name, const float * matrix)
{
    int location = GetUniformLocation(name);
    GLCall(glUniformMatrix4fv(location, 1, GL_FALSE, matrix));
}

unsigned int Shader::GetUniformLocation(const std::string& name) const
{
	const auto it = m_UniformLocationCache.find(name);
	if (it != m_UniformLocationCache.end())
		return it->second;
	else
	{
		std::cout << "uniform " << name << " is added to cache" << std::endl;
		GLCall(int location = glGetUniformLocation(m_RenderID, name.c_str()));
		//ASSERT(location != -1);
		m_UniformLocationCache[name] = location;
		return location;
	}
}

unsigned int Shader::CompileShader(unsigned int type, const std::string& source)
{
	unsigned int id = glCreateShader(type);
	const char *src = source.c_str();
	glShaderSource(id, 1, &src, nullptr);
	glCompileShader(id);

	int result;
	glGetShaderiv(id, GL_COMPILE_STATUS, &result);
	if (result == GL_FALSE)
	{
		int length;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		char* message = (char*)alloca(length * sizeof(char));
		glGetShaderInfoLog(id, length, &length, message);
		std::cout << "Failed to compile " << std::endl;
		std::cout << message << std::endl;
		glDeleteShader(id);
		return 0;
	}
	//TODO : Error handling
	return id;
}
unsigned int Shader::CreateShader(const std::string &vertexShader, const std::string& fragmentShader)
{
	GLCall(unsigned int program = glCreateProgram());
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);

	glDeleteShader(vs);
	glDeleteShader(fs);

	return program;
}
ShaderProgramSource Shader::ParseShader(const std::string& filepath)
{
	std::ifstream stream(filepath);
	enum class ShaderType
	{
		NONE = -1, VERTEX =0, FRAGMENT =1
	};
	std::string line;
	std::stringstream ss[2];
	ShaderType type = ShaderType::NONE;
	while (std::getline(stream, line))
	{
		if (line.find("#shader") != std::string::npos)
		{
			if (line.find("vertex") != std::string::npos)
				type = ShaderType::VERTEX;
			else if (line.find("fragment") != std::string::npos)
				type = ShaderType::FRAGMENT;
		}
		else
		{
			ss[int(type)] << line << '\n';
		}

	}
	return {ss[0].str(), ss[1].str()};
}


std::vector<std::string> Shader::ParseShaderForUniforms(const std::string& filepath)
{
	std::ifstream stream(filepath);
	std::vector<std::string> uniforms;
	std::string line;
	while (std::getline(stream, line))
	{
		if (line.find("uniform") != std::string::npos)
		{
			std::stringstream oss(line);
			std::string uniform;
			std::string type;
			std::string name;
			oss >> uniform;
			oss >> type;
			oss >> name;
			std::string name_no_semi = name.substr(0, name.size() - 1);
			uniforms.push_back(name_no_semi);
		}
	}
	return uniforms;
}