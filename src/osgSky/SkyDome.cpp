/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <osgSky/SkyDome>

using namespace osgSky;

class UniformFogDensityCallback : public osg::Uniform::Callback
{
	virtual void operator()(osg::Uniform* uniform,osg::NodeVisitor* nv)
	{

	}
};

SkyDome::SkyDome( void )
	:_isDefaultShader(true)
{
    
}

SkyDome::SkyDome( const SkyDome& copy, const osg::CopyOp& copyop ):
    SphereSegment( copy, copyop )
	,_isDefaultShader(true)
{

}

SkyDome::SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap )
	:_isDefaultShader(true)
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    setupStateSet(cubemap);
}
SkyDome::SkyDome( float radius,unsigned int longitudeSteps, unsigned int lattitudeSteps,float longStart,float longEnd,float latStart,float latEnd,osg::TextureCubeMap* cubemap)
	:_isDefaultShader(true)
{
	compute( radius, longitudeSteps, lattitudeSteps, longStart, longEnd, latStart, latEnd);
	setupStateSet(cubemap);
}

SkyDome::~SkyDome(void)
{
}

void SkyDome::create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
	setupStateSet(cubemap);
}

void SkyDome::setupStateSet( osg::TextureCubeMap* cubemap )
{
    osg::ref_ptr<osg::StateSet> ss = getOrCreateStateSet();

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	if(cubemap)
		ss->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
	ss->setAttributeAndModes(createShader().get(), osg::StateAttribute::ON);

	ss->addUniform(new osg::Uniform("uEnvironmentMap",0));
	ss->addUniform(new osg::Uniform("aboveWaterFogColor",osg::Vec4(1.0,1.0,1.0,1.0)));
	float fogDensity=0.0;
	ss->addUniform(new osg::Uniform("aboveWaterFogDensity",fogDensity));
}

osg::ref_ptr<osg::Program> SkyDome::createShader()
{
    osg::ref_ptr<osg::Program> program = new osg::Program;
    
	// Do not use shaders if they were globally disabled.
	if (osgOcean::ShaderManager::instance().areShadersEnabled())
	{
		char vertexSource[] =
			"varying vec3 vTexCoord;\n"
			"\n"
			"void main(void)\n"
			"{\n"
			"    gl_Position = ftransform();\n"
			"    vTexCoord = gl_Vertex.xyz;\n"
			"    gl_FogFragCoord=gl_Position.z;\n"
			"}\n";

		char fragmentSource[] =
			"uniform samplerCube uEnvironmentMap;\n"
			"varying vec3 vTexCoord;\n"
			"\n"
			"void main(void)\n"
			"{\n"
			"    vec3 tex = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);\n"
			"    gl_FragColor = textureCube( uEnvironmentMap, tex.xzy );\n"
			"}\n";

		std::string vertexFile = "skydome.vert";
		std::string fragmentFile = "skydome.frag";

		program->setName("sky_dome_shader");
		if (_isDefaultShader)
		{
			program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexSource));
			program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
		}
		else
		{
			program = osgOcean::ShaderManager::instance().createProgram("sky_dome_shader", vertexFile, fragmentFile, true);
		}
	}

    return program;
}
void SkyDome::setDefaultShader(bool on)
{
	//if (_isDefaultShader == on)
	//	return;
	_isDefaultShader = on;
	getOrCreateStateSet()->setAttributeAndModes(createShader().get(), osg::StateAttribute::ON);
}