#include <osgOcean/FlatRingOcean>

#include <osgUtil/CullVisitor>
#include <osg/Material>
#include <fstream>
#include <osgDB/FileUtils>
#include <osg/Notify>
#include <osgOcean/ShaderManager>
#include <sstream>

using namespace osgOcean;

FlatRingOceanGeode::FlatRingOceanGeode(float w,float out,unsigned int cSteps,unsigned int rSteps)
	:_inR(w)
	,_outR(out)
	,_circleSteps(cSteps)
	,_rSteps(rSteps)
	,_isDirty(true)
	,_centerPoint(0.0,0.0)
	,_height(0.0)
	,_lightColor( 0.411764705f, 0.54117647f, 0.6823529f, 1.f )
	,_waveTopColor(0.192156862f, 0.32549019f, 0.36862745098f)   //直接从FFTOceanSurface中复制过来的颜色
	, _waveBottomColor( 0.11372549019f, 0.219607843f, 0.3568627450f )
	,_enableReflections(false)
	,_skyNodeSet(false)
	,_fogColor(1.0,1.0,1.0,1.0)
	,_fogDensity(0.0)
	,_vertices(new osg::Vec3Array)
	,_normals(new osg::Vec3Array)
	,_texcoords(new osg::Vec2Array)
	,_colors(new osg::Vec4Array)
#if MODE_2018_9_23 
	,_eye(osg::Vec3d(0.0,0.0,0.0))
{
#else
{
	setUserData(new FlatOceanDataType(*this));
#endif
	setUpdateCallback(new FlatRingOceanGeodeCallback);
	setCullCallback(new FlatRingOceanGeodeCallback);
}

FlatRingOceanGeode::~FlatRingOceanGeode()
{
}

void FlatRingOceanGeode::SetInR(float w)
{
	_inR=w;
	_isDirty=true;
}
float FlatRingOceanGeode::GetInR()
{
	return _inR;
}
void FlatRingOceanGeode::SetOutR(float l)
{
	_outR=l;
	_isDirty=true;
}
float FlatRingOceanGeode::GetOutR()
{
	return _outR;
}
float FlatRingOceanGeode::GetOceanWidth()
{
	return GetOutR();
}
void FlatRingOceanGeode::build(float h)
{
	createOceanGeometry();

	computeVertices();

	computePrimitives();

	initStateSet();

	_isDirty=false;
	_isStateDirty=false;
}
void FlatRingOceanGeode::createOceanGeometry()
{
	if(getNumDrawables()>0)
		removeDrawables(0,getNumDrawables());

	_geom = new osg::Geometry(); 

	_geom->setVertexArray( _vertices );
	_geom->setTexCoordArray( 0, _texcoords );
	_geom->setColorArray( _colors );
	_geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	_geom->setNormalArray(_normals);
	_geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	addDrawable( _geom.get() );
}
void FlatRingOceanGeode::computePrimitives()
{
	// First clear old primitive sets
	if(_geom->getNumPrimitiveSets()>0)
		_geom->removePrimitiveSet(0, _geom->getNumPrimitiveSets() );

	for(unsigned int r=0;r<GetRSteps();r++)
	{
		osg::ref_ptr<osg::DrawElementsUInt> indices=new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP,0);
		for(unsigned int c=0;c<=GetCircleSteps();c++)
		{
			indices->push_back(c+r*(GetCircleSteps()+1));
			indices->push_back(c+(r+1)*(GetCircleSteps()+1));
		}
		_geom->addPrimitiveSet(indices.get());
	}

	//2018_9_23 14:00
	dirtyBound();
}
void FlatRingOceanGeode::computeVertices()
{
	_vertices->resize((GetRSteps()+1)*(GetCircleSteps()+1));
	_texcoords->resize((GetRSteps()+1)*(GetCircleSteps()+1));

	unsigned int ptr = 0;

	double x,y,z,R,sin_t,cos_t;
	double thetaDelta=2*osg::PI/GetCircleSteps();
	double RDelta=(GetOutR()-GetInR())/GetRSteps();

	double scale;

	for(unsigned int i=0;i<=GetRSteps();i++)
	{
		for(unsigned int j=0;j<=GetCircleSteps();j++)
		{
			R=GetInR()+i*RDelta;
			sin_t=sin(j*thetaDelta);
			cos_t=cos(j*thetaDelta);
			x=GetCenterPoint().x()+R*cos_t;
			y=GetCenterPoint().y()+R*sin_t;
			z=GetHeight();

			(*_vertices)[ptr]=osg::Vec3(x,y,z);
			scale=R/GetOutR();
		    (*_texcoords)[ptr]=osg::Vec2(0.5*(1+scale*cos_t),0.5*(1+scale*sin_t));
			++ptr;
		}
	}

	_normals->resize(1);
	(*_normals)[0]=osg::Vec3(0.0,0.0,1.0);

	_colors->resize(1);
	(*_colors)[0]=osg::Vec4( 1.f, 1.f, 1.f, 1.f );
}
void FlatRingOceanGeode::initStateSet()
{
	osg::ref_ptr<osg::StateSet> stateSet=getOrCreateStateSet();

	// Environment map    
	stateSet->addUniform( new osg::Uniform("osgOcean_EnvironmentMap", ENV_MAP ) );
	if (osgOcean::ShaderManager::instance().areShadersEnabled())
		stateSet->setTextureAttributeAndModes( ENV_MAP, _environmentMap.get(), osg::StateAttribute::ON );

	stateSet->getOrCreateUniform("mapTexture2D_dark",osg::Uniform::INT_SAMPLER_2D)->set(MAP_DARK);

	// Colouring
	osg::Vec4f waveTop = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveTopColor,1.f) );
	osg::Vec4f waveBot = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveBottomColor,1.f) );

	stateSet->getOrCreateUniform("osgOcean_WaveTop",osg::Uniform::FLOAT_VEC4)->set(waveTop);
	stateSet->getOrCreateUniform("osgOcean_WaveBot",osg::Uniform::FLOAT_VEC4)->set(waveBot);
	stateSet->getOrCreateUniform("osgOcean_EnableReflections",osg::Uniform::BOOL)->set(_enableReflections);
	float _fresnelMul=0.7;
	stateSet->getOrCreateUniform("osgOcean_FresnelMul",osg::Uniform::FLOAT)->set(_fresnelMul);
	_stateset->addUniform( new osg::Uniform("osgOcean_EyePosition", osg::Vec3f() ) );


	stateSet->getOrCreateUniform("osgOcean_AboveWaterFogDensity",osg::Uniform::FLOAT)->set(_fogDensity);
	stateSet->getOrCreateUniform("osgOcean_AboveWaterFogColor",osg::Uniform::FLOAT_VEC4)->set(_fogColor);
	//也可以添加个噪声
	//osg::ref_ptr<osg::Texture2D> noiseMap 
	//	= createNoiseMap( _noiseTileSize, _noiseWindDir, _noiseWindSpeed, _noiseWaveScale, _noiseTileRes ); 

	//if (ShaderManager::instance().areShadersEnabled())
	//	ss->setTextureAttributeAndModes( NORMAL_MAP, noiseMap.get(), osg::StateAttribute::ON );
	osg::ref_ptr<osg::Program> program=createShader();
	if(program.valid())
		stateSet->setAttributeAndModes( program.get(), osg::StateAttribute::ON );

	// If shaders are enabled, the final color will be determined by the 
	// shader so we need a white base color. But on the fixed pipeline the
	// material color will determine the ocean surface's color.
	if (!osgOcean::ShaderManager::instance().areShadersEnabled())
	{
		osg::ref_ptr<osg::Material> mat = new osg::Material;
		mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4f(_waveTopColor, 1.0f));
		stateSet->setAttributeAndModes(mat, osg::StateAttribute::ON);
	}

	_isStateDirty = false;

	osg::notify(osg::INFO) << "FFTOceanSurface::initStateSet() Complete." << std::endl;
}


osg::Texture2D* FlatRingOceanGeode::createTexture(const std::string& name, osg::Texture::WrapMode wrap)
{
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D();

	tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	tex->setWrap  (osg::Texture::WRAP_S,     wrap);
	tex->setWrap  (osg::Texture::WRAP_T,     wrap);
	tex->setImage (osgDB::readImageFile(name.c_str()));

	return tex.release();
}
osg::Vec3f FlatRingOceanGeode::computeNoiseCoords(float noiseSize, const osg::Vec2f& movement, float speed, float time)
{
	float length = noiseSize*movement.length();
	float totalTime = length / speed;   
	float diff=GetOutR()-GetInR();
	if(abs(diff)<0.0001)
		diff=1; //防止为零
	float tileScale = 1.0/(diff) * noiseSize;

	osg::Vec2f velocity = movement * speed / length;
	osg::Vec2f pos = velocity * fmod( time, totalTime );

	return osg::Vec3f( pos, tileScale );
}
osg::Program* FlatRingOceanGeode::createShader()
{
	std::string shaderName="flatRingOcean_surface";
	std::string vertFile="FlatRingOcean.vert";
	std::string fragmentFile="FlatRingOcean.frag";

	return osgOcean::ShaderManager::instance().createProgram(shaderName,vertFile,fragmentFile,true);
}

void FlatRingOceanGeode::UpdateOcean(const osg::Vec3f& eye)
{
	if(_isDirty)
		build(GetHeight());
	else if(_isStateDirty)
		initStateSet();

	getOrCreateStateSet()->getOrCreateUniform("osgOcean_EyePosition",osg::Uniform::FLOAT_VEC3)->set(eye);

	if(upDateCenterPoint(eye))
	{
		SetCenterPoint(osg::Vec2(eye.x(),eye.y()));
		computeVertices();
		computePrimitives();
	}
}

void FlatRingOceanGeode::setEye(osg::Vec3 pos)
{
#if MODE_2018_9_23
	_eye=pos;
#else
	osg::ref_ptr<FlatRingOceanGeode::FlatOceanDataType> data=dynamic_cast<FlatRingOceanGeode::FlatOceanDataType*>(getUserData());
	if(data.valid())
	{
		data->setEye(pos);
	}
#endif
}
osg::Vec3 FlatRingOceanGeode::getEye()
{
#if MODE_2018_9_23
	return _eye;
#else
	osg::ref_ptr<FlatRingOceanGeode::FlatOceanDataType> data=dynamic_cast<FlatRingOceanGeode::FlatOceanDataType*>(getUserData());
	if(data.valid())
	{
		return data->getEye();
	}
#endif
}
#if (MODE_2018_9_23==0)
FlatRingOceanGeode::FlatOceanDataType::FlatOceanDataType(FlatRingOceanGeode& ocean)
	:_oceanSurface(ocean)
{

}
FlatRingOceanGeode::FlatOceanDataType::FlatOceanDataType( const FlatOceanDataType& copy, const osg::CopyOp& copyop/* =osg::CopyOp::SHALLOW_COPY */ )
	:_oceanSurface(copy._oceanSurface)
	,_eye(copy._eye)
{

}
void FlatRingOceanGeode::FlatOceanDataType::updateOcean( void )
{
	_oceanSurface.UpdateOcean(_eye );
}
#endif
void FlatRingOceanGeode::FlatRingOceanGeodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
#if MODE_2018_9_23
	osg::ref_ptr<FlatRingOceanGeode> _flatRingOceanGeode=dynamic_cast<FlatRingOceanGeode*>(node);
	if(_flatRingOceanGeode.valid())
	{
		if(nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)
		{
			_flatRingOceanGeode->UpdateOcean(_flatRingOceanGeode->getEye());
		}
		else if(nv->getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
		{
			osgUtil::CullVisitor* cv=static_cast<osgUtil::CullVisitor*>(nv);
			_flatRingOceanGeode->setEye(cv->getEyePoint());
		}
		traverse(node,nv);
	}
#else

	osg::ref_ptr<FlatOceanDataType> oceanData = dynamic_cast<FlatOceanDataType*> ( node->getUserData() );

	if( oceanData.valid() )
	{
		// If cull visitor update the current eye position
		if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
		{
			osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
			oceanData->setEye( cv->getEyePoint() );
		}
		else if( nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR ){
			oceanData->updateOcean();
		}
	}
	traverse(node, nv); 

#endif
}