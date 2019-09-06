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
	:_isStateSetDirty(true)
{
    
}

SkyDome::SkyDome( const SkyDome& copy, const osg::CopyOp& copyop ):
    SphereSegment( copy, copyop )
{

}

SkyDome::SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
	_cubmap=cubemap;
    //setupStateSet(cubemap);
}
SkyDome::SkyDome( float radius,unsigned int longitudeSteps, unsigned int lattitudeSteps,float longStart,float longEnd,float latStart,float latEnd,osg::TextureCubeMap* cubemap)
{
	compute( radius, longitudeSteps, lattitudeSteps, longStart, longEnd, latStart, latEnd);
	_cubmap=cubemap;
	//setupStateSet(cubemap);
}

SkyDome::~SkyDome(void)
{
}

void SkyDome::create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    _cubmap=cubemap;
	//setupStateSet(cubemap);
}

void SkyDome::setupStateSet( osg::TextureCubeMap* cubemap )
{
    osg::ref_ptr<osg::StateSet> ss = getOrCreateStateSet();

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
    ss->setAttributeAndModes( createShader().get(), osg::StateAttribute::ON );
	ss->getOrCreateUniform("uEnvironmentMap",osg::Uniform::SAMPLER_CUBE)->set(0);
	ss->addUniform(new osg::Uniform("aboveWaterFogColor",osg::Vec4(1.0,1.0,1.0,1.0)));
	float fogDensity=0.0;
	ss->addUniform(new osg::Uniform("aboveWaterFogDensity",fogDensity));
	_isStateSetDirty=false;
}

osg::ref_ptr<osg::Program> SkyDome::createShader(void)
{
    osg::ref_ptr<osg::Program> program = new osg::Program;

    // Do not use shaders if they were globally disabled.
    if (osgOcean::ShaderManager::instance().areShadersEnabled())
    {
        char vertexSource[]=
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "    gl_Position = ftransform();\n"
            "    vTexCoord = gl_Vertex.xyz;\n"
			"    gl_FogFragCoord=gl_Position.z;\n"
            "}\n";

		//肉眼观察确定雾的效果
        char fragmentSource[]=
            "uniform samplerCube uEnvironmentMap;\n"
            "varying vec3 vTexCoord;\n"
			"uniform float aboveWaterFogDensity;\n"
		    "uniform vec4 aboveWaterFogColor;\n"
			"vec4 lighting(vec4 colormap)\n"
			"{\n"
			"   vec4 color=gl_LightSource[0].ambient*colormap;\n"
			"   return color;\n"
			"}\n"
			"float computeFogFactor(float density,float fogCoord)\n"
		    "{\n"
			"return exp2(density*fogCoord*fogCoord);\n"
		    "}\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "    vec3 tex = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);\n"
			"    vec4 textureColor= textureCube( uEnvironmentMap, tex.xzy );\n"
            "    vec4 final_color=textureColor;"
            "/*  float fogFactor=computeFogFactor(aboveWaterFogDensity,gl_FogFragCoord);\n"
			"    vec4 final_color=mix(vec4(1.0,1.0,1.0,1.0),textureColor,fogFactor);\n"
            "*/"
			"   if(aboveWaterFogDensity>50/100000.0)"
			"        final_color=vec4(1.0,1.0,1.0,1.0);"
            "    final_color=lighting(final_color);\n"
            "    gl_FragColor = final_color;\n"
            "  gl_FragColor.a = 1.0;\n"
            "}\n";

        program->setName( "sky_dome_shader" );
        program->addShader(new osg::Shader(osg::Shader::VERTEX,   vertexSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
		std::string vertexFile="skydome.vert";
		std::string fragmentFile="skydome.frag";
		program=osgOcean::ShaderManager::instance().createProgram("sky_dome_shader",vertexFile,fragmentFile,true);
    }

    return program;
}
void SkyDome::traverse(osg::NodeVisitor& nv)
{
	if(_isStateSetDirty && _cubmap.valid())
		setupStateSet(_cubmap.get());
}