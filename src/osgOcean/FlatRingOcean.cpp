#include <osgOcean/FlatRingOcean>
#include <osgUtil/CullVisitor>
#include <osgGA/EventVisitor>
#include <osg/Material>
#include <fstream>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>
#include <osg/Notify>
#include <osgOcean/ShaderManager>
#include <sstream>
#include <iomanip>

#define RESOLUTION 256
#define INITWAVESCALE 1e-8
#define REFLECTIONDAMPING 0.35
using namespace osgOcean;

FlatRingOceanGeode::FlatRingOceanGeode(float w, float out, unsigned int cSteps, unsigned int rSteps)
	:OceanTechnique()
	,_inR(w)
	, _outR(out)
	, _height(0.0)
	, _circleSteps(cSteps)
	, _rSteps(rSteps)
	, _isStateDirty(true)
	,_isWaveDirty(true)
	, _centerPoint(0.0, 0.0)
	, _lightColor(0.411764705f, 0.54117647f, 0.6823529f, 1.f)
	, _waveTopColor(0.192156862f, 0.32549019f, 0.36862745098f)   //直接从FFTOceanSurface中复制过来的颜色
	, _waveBottomColor(0.11372549019f, 0.219607843f, 0.3568627450f)
	, _enableReflections(false)
	,_enableRefractions(false)
	//, _lightID(0)
	,_useCrestFoam(false),
	_foamCapBottom(2.2f),
	_foamCapTop(3.0f),
	_noiseTileResInv(1.f / float(256)),
	_fresnelMul(0.7),
	_noiseTileSize(32),
	_noiseTileRes(256),
	_noiseWindDir(osg::Vec2(1.0,1.0)),
	_noiseWindSpeed(6.0),
	_noiseWaveScale(1e-8),
	_depth(1000.0),
	_reflDampFactor(0.35),
	//_aboveWaterFogDensity(0.0012f),
	//_aboveWaterFogColor(osg::Vec4(199/255.0, 226/255.0, 255/255.0,1.0)),
	_NUMFRAMES(256),
	_cycleTime(10),
	_choppyFactor(-2.5),
	_isChoppy(true),
	 _environmentMap(0)
	, _geom(0)
	, _vertices(new osg::Vec3Array)
	, _normals(new osg::Vec3Array)
	, _texcoords(new osg::Vec2Array)
	, _colors(new osg::Vec4Array)
	,_isEndless(false)
	,_isCardNoWarning(true)
#if MODE_2018_9_23 
	, _eye(osg::Vec3d(0.0, 0.0, 0.0))
{
#else
{
		setUserData(new FlatOceanDataType(*this, _NUMFRAMES, 25));
#endif
		osg::ref_ptr<FlatRingOceanGeodeCallback> _callback = new FlatRingOceanGeodeCallback;
		setEventCallback(new OceanAnimationEventHandler);
		setUpdateCallback(_callback.get());
		setCullCallback(_callback.get());
		//osgOcean::ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
}

FlatRingOceanGeode::~FlatRingOceanGeode()
{
}

void FlatRingOceanGeode::setInR(float w)
{
	_inR = w;
	_isDirty = true;
}
float FlatRingOceanGeode::getInR()
{
	return _inR;
}
void FlatRingOceanGeode::setOutR(float l)
{
	_outR = l;
	_isDirty = true;
}
float FlatRingOceanGeode::getOutR()
{
	return _outR;
}
float FlatRingOceanGeode::getOceanWidth()
{
	return getOutR();
}
void FlatRingOceanGeode::build()
{
	createOceanGeometry();

	computeVertices();

	computePrimitives();

	initStateSet();

	buildWaveTextures();

	_isDirty = false;
	_isStateDirty = false;
	_isWaveDirty = false;
}
void FlatRingOceanGeode::createOceanGeometry()
{
	if (getNumDrawables() > 0)
		removeDrawables(0, getNumDrawables());
	if (!_geom.valid())
	{
		_geom = new osg::Geometry();
		_geom->setName("flatRingOceanGeometry");
		_geom->setUseDisplayList(false);
		_geom->setDataVariance(osg::Object::DYNAMIC);
	}
	_geom->setVertexArray(_vertices);
	_geom->setTexCoordArray(0, _texcoords);
	_geom->setColorArray(_colors);
	_geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	_geom->setNormalArray(_normals);
	_geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	addDrawable(_geom.get());
}
void FlatRingOceanGeode::computePrimitives()
{
	//return;
	// First clear old primitive sets
	if (_geom->getNumPrimitiveSets() > 0)
		_geom->removePrimitiveSet(0, _geom->getNumPrimitiveSets());
	unsigned int stripSize = ((getCircleSteps()+1) * 2)*getRSteps();
	osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, stripSize);
	indices->setName("flatRingOceanDrawElements");
	unsigned int i = 0;
	for (unsigned int r = 0; r < getRSteps(); r++)
	{
		for (unsigned int c = 0; c <getCircleSteps()+1; c++)
		{
			unsigned int cFirst = c + (r) * (getCircleSteps()+1), cSecond = c + (r+1)*(getCircleSteps()+1);
			(*indices)[i] = cFirst;
			(*indices)[i+1]=cSecond;
			i += 2;
			//OSG_NOTICE << "indices: " << cFirst << " and " << cSecond << std::endl;
		}
	}
	_geom->addPrimitiveSet(indices);

	//2018_9_23 14:00
	dirtyBound();
}
void FlatRingOceanGeode::computeVertices()
{
	//return;
	_vertices->resize((getRSteps() + 1)*(getCircleSteps() + 1));
	_texcoords->resize((getRSteps() + 1)*(getCircleSteps() + 1));

	unsigned int ptr = 0;

	float x, y, z, R, sin_t, cos_t;
	float thetaDelta = 2 * osg::PI / getCircleSteps();
	float RDelta = (getOutR() - getInR()) / getRSteps();

	float scale;

	for (unsigned int i = 0; i <= getRSteps(); i++)
	{
		for (unsigned int j = 0; j <= getCircleSteps(); j++)
		{
			R = getInR() + i * RDelta;
			sin_t = sin(j*thetaDelta);
			cos_t = cos(j*thetaDelta);
			x = getCenterPoint().x() + R * cos_t;
			y = getCenterPoint().y() + R * sin_t;
			z = getSurfaceHeight();

			(*_vertices)[ptr] = osg::Vec3(x, y, z);
			scale = R / getOutR();
			(*_texcoords)[ptr] = osg::Vec2(0.5*(1 + scale * cos_t), 0.5*(1 + scale * sin_t));
			++ptr;
		}
	}
	//OSG_NOTICE << "FlatRingOcean vertices size[" << ptr <<"] and ["<<_vertices->size()<<"] "<<_texcoords->size()<< std::endl;
	_normals->resize(1);
	(*_normals)[0] = osg::Vec3(0.0, 0.0, 1.0);

	_colors->resize(1);
	(*_colors)[0] = osg::Vec4(1.f, 1.f, 1.f, 1.f);
}
void FlatRingOceanGeode::initStateSet()
{
	osg::notify(osg::INFO) << "FlatRingOcean::initStateSet()" << std::endl;
	_stateset = new osg::StateSet;

	/*_stateset->getOrCreateUniform("osgOcean_LightID", osg::Uniform::UNSIGNED_INT)->set(_lightID);
	const float LOG2E = 1.442695;

	_stateset->getOrCreateUniform("osgOcean_AboveWaterFogDensity", osg::Uniform::Type::FLOAT)->set(-_aboveWaterFogDensity * _aboveWaterFogDensity*LOG2E);
	_stateset->getOrCreateUniform("osgOcean_AboveWaterFogColor", osg::Uniform::Type::FLOAT_VEC4)->set(_aboveWaterFogColor);*/
	
	// Note that we will only set the textures in the state set if shaders are
	// enabled, otherwise the fixed pipeline will try to put the env map onto
	// the water surface, which has no texture coordinates, so the surface
	// will take the general color of the env map...

	// Environment map    
	_stateset->addUniform(new osg::Uniform("osgOcean_EnvironmentMap", ENV_MAP));
	if (osgOcean::ShaderManager::instance().areShadersEnabled())
		_stateset->setTextureAttributeAndModes(ENV_MAP, _environmentMap.get(), osg::StateAttribute::ON);

	// Foam
	_stateset->addUniform(new osg::Uniform("osgOcean_EnableCrestFoam", _useCrestFoam));
	_stateset->addUniform(new osg::Uniform("osgOcean_FoamCapBottom", _foamCapBottom));
	_stateset->addUniform(new osg::Uniform("osgOcean_FoamCapTop", _foamCapTop));
	_stateset->addUniform(new osg::Uniform("osgOcean_FoamMap", FOAM_MAP));
	_stateset->addUniform(new osg::Uniform("osgOcean_FoamScale", _noiseTileResInv*30.f));

	//反射与折射
	_stateset->addUniform(new osg::Uniform("osgOcean_EnableReflections", _enableReflections));
	_stateset->addUniform(new osg::Uniform("osgOcean_EnableRefractions", _enableRefractions));

	if (_useCrestFoam)
	{
		osg::Texture2D* foam_tex = createTexture("sea_foam.png", osg::Texture::REPEAT);
		if (osgOcean::ShaderManager::instance().areShadersEnabled())
			_stateset->setTextureAttributeAndModes(FOAM_MAP, foam_tex, osg::StateAttribute::ON);
	}

	// Noise
	_stateset->addUniform(new osg::Uniform("osgOcean_NoiseMap", NORMAL_MAP));
	_stateset->addUniform(new osg::Uniform("osgOcean_NoiseCoords0", computeNoiseCoords(32.f, osg::Vec2f(2.f, 4.f), 2.f, 0.f)));
	_stateset->addUniform(new osg::Uniform("osgOcean_NoiseCoords1", computeNoiseCoords(8.f, osg::Vec2f(-4.f, 2.f), 1.f, 0.f)));

	osg::ref_ptr<osg::Texture2D> noiseMap
		= createNoiseMap(_noiseTileSize, _noiseWindDir, _noiseWindSpeed, _noiseWaveScale, _noiseTileRes);

	if (osgOcean::ShaderManager::instance().areShadersEnabled())
		_stateset->setTextureAttributeAndModes(NORMAL_MAP, noiseMap.get(), osg::StateAttribute::ON);

	// Colouring
	osg::Vec4f waveTop = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveTopColor, 1.f));
	osg::Vec4f waveBot = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveBottomColor, 1.f));

	_stateset->addUniform(new osg::Uniform("osgOcean_WaveTop", waveTop));
	_stateset->addUniform(new osg::Uniform("osgOcean_WaveBot", waveBot));
	_stateset->addUniform(new osg::Uniform("osgOcean_FresnelMul", _fresnelMul));
	_stateset->addUniform(new osg::Uniform("osgOcean_EyePosition", osg::Vec3f()));

	int sizeTemp = int(_noiseTileSize);
	_stateset->addUniform(new osg::Uniform("tileSize",sizeTemp));    //oneTileSize
	//_stateset->addUniform(new osg::Uniform("tileResolution", _noiseTileRes));    //oneTileRes
	int numTiles = 2*(int)_outR / _noiseTileRes;
	if (numTiles < 1)
		numTiles = 17;
	_stateset->addUniform(new osg::Uniform("numTiles",numTiles ));                 //tileNum 
	_stateset->addUniform(new osg::Uniform("currentFrame",0));                 //frame

	osg::ref_ptr<osg::Program> program = createShader();

	if (program.valid())
		_stateset->setAttributeAndModes(program.get(), osg::StateAttribute::ON);

	// If shaders are enabled, the final color will be determined by the 
	// shader so we need a white base color. But on the fixed pipeline the
	// material color will determine the ocean surface's color.
	if (!osgOcean::ShaderManager::instance().areShadersEnabled())
	{
		osg::Material* mat = new osg::Material;
		mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4f(_waveTopColor, 1.0f));
		_stateset->setAttributeAndModes(mat, osg::StateAttribute::ON);
	}

	_isStateDirty = false;

	osg::notify(osg::INFO) << "FlatRingOcean::initStateSet() Complete." << std::endl;
}


osg::Texture2D* FlatRingOceanGeode::createTexture(const std::string& name, osg::Texture::WrapMode wrap)
{
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D();

	tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, wrap);
	tex->setWrap(osg::Texture::WRAP_T, wrap);
	tex->setImage(osgDB::readImageFile(name.c_str()));

	return tex.release();
}
osg::Vec3f FlatRingOceanGeode::computeNoiseCoords(float noiseSize, const osg::Vec2f& movement, float speed, float time)
{
	float length = noiseSize * movement.length();
	float totalTime = length / speed;
	float diff = getOutR() - getInR();
	if (abs(diff) < 0.0001)
		diff = 1; //防止为零
	float tileScale = 1.0 / (diff)* noiseSize;

	osg::Vec2f velocity = movement * speed / length;
	osg::Vec2f pos = velocity * fmod(time, totalTime);

	return osg::Vec3f(pos, tileScale);
}
osg::ref_ptr<osg::Texture2D> FlatRingOceanGeode::createNoiseMap(unsigned int size,
	const osg::Vec2f& windDir,
	float windSpeed,
	float waveScale,
	float tileResolution)
{
	osg::ref_ptr<osg::FloatArray> heights = new osg::FloatArray;

	osgOcean::FFTSimulation noiseFFT(size, windDir, windSpeed, _depth, _reflDampFactor, waveScale, tileResolution, 10.f);
	noiseFFT.setTime(0.f);
	noiseFFT.computeHeights(heights.get());

	osgOcean::OceanTile oceanTile(heights.get(), size, tileResolution / size);

	return oceanTile.createNormalMap();
}
osg::Program* FlatRingOceanGeode::createShader()
{
	std::string shaderName = "flatRingOcean_surface";
	std::string vertFile = "flatRingWaterDebug.vert";
	std::string fragmentFile = "flatRingWater.frag";
	if (getVideoCardState())
	{
		vertFile = "flatRingWater.vert";
		fragmentFile = "flatRingWaterDebug.frag";
	}
	return osgOcean::ShaderManager::instance().createProgram(shaderName, vertFile, fragmentFile, true);
}

void FlatRingOceanGeode::updateOcean(const osg::Vec3f& eye,const double& dt,unsigned int frame)
{
	if (_isDirty)
		build();
	else if (_isStateDirty)
		initStateSet();
	if (_isWaveDirty)
		buildWaveTextures();

	getOrCreateStateSet()->getOrCreateUniform("osgOcean_EyePosition", osg::Uniform::FLOAT_VEC3)->set(eye);
	static double time = 0.0;
	time += (dt*0.0008);

	getStateSet()->getUniform("osgOcean_NoiseCoords0")->set(computeNoiseCoords(32.f, osg::Vec2f(2.f, 4.f), 2.f, time));
	getStateSet()->getUniform("osgOcean_NoiseCoords1")->set(computeNoiseCoords(8.f, osg::Vec2f(-4.f, 2.f), 1.f, time));
	
	int frameTemp = frame;
	getStateSet()->getUniform("currentFrame")->set(frameTemp);

	if (updateCenterPoint(eye))
	{
		setCenterPoint(osg::Vec2(eye.x(), eye.y()));
		computeVertices();
		computePrimitives();
	}
}
float FlatRingOceanGeode::getSurfaceHeight()const
{
	return _height;
}
float FlatRingOceanGeode::getSurfaceHeightAt(float x, float y, osg::Vec3f* normal /* = NULL */)
{
	return _height;
}
void FlatRingOceanGeode::setEye(osg::Vec3 pos)
{
#if MODE_2018_9_23
	_eye = pos;
#else
	osg::ref_ptr<FlatRingOceanGeode::FlatOceanDataType> data = dynamic_cast<FlatRingOceanGeode::FlatOceanDataType*>(getUserData());
	if (data.valid())
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
	osg::ref_ptr<FlatRingOceanGeode::FlatOceanDataType> data = dynamic_cast<FlatRingOceanGeode::FlatOceanDataType*>(getUserData());
	if (data.valid())
	{
		return data->getEye();
	}
#endif
	return osg::Vec3(0.0, 0.0, 0.0);
}
#if (MODE_2018_9_23==0)
FlatRingOceanGeode::FlatOceanDataType::FlatOceanDataType(FlatRingOceanGeode& ocean, unsigned int numFrames, unsigned int fps)
	:_oceanSurface(ocean),
	_NUMFRAMES(numFrames),
	_time(0.f),
	_FPS(fps),
	_msPerFrame(1000.f / (float)fps),
	_frame(0),
	_oldTime(0),
	_newTime(0)
{

}
FlatRingOceanGeode::FlatOceanDataType::FlatOceanDataType(const FlatOceanDataType& copy, const osg::CopyOp& copyop/* =osg::CopyOp::SHALLOW_COPY */)
	: _oceanSurface(copy._oceanSurface),
	_NUMFRAMES(copy._NUMFRAMES),
	_eye(copy._eye),
	_time(copy._time),
	_FPS(copy._FPS),
	_msPerFrame(copy._msPerFrame),
	_frame(copy._frame),
	_oldTime(copy._oldTime),
	_newTime(copy._newTime)
{

}
void FlatRingOceanGeode::FlatOceanDataType::updateOcean(void)
{
	_oldTime = _newTime;
	_newTime = osg::Timer::instance()->tick();

	double dt = osg::Timer::instance()->delta_m(_oldTime, _newTime);
	_time += dt;

	if (_time >= _msPerFrame)
	{
		_frame += (_time / _msPerFrame);

		if (_frame >= _NUMFRAMES)
			_frame = _frame % _NUMFRAMES;

		_time = fmod(_time, _msPerFrame);
	}
	_oceanSurface.updateOcean(_eye,dt,_frame);
}
#endif
void FlatRingOceanGeode::FlatRingOceanGeodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
#if MODE_2018_9_23
	osg::ref_ptr<FlatRingOceanGeode> _flatRingOceanGeode = dynamic_cast<FlatRingOceanGeode*>(node);
	if (_flatRingOceanGeode.valid())
	{
		if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
		{
			_flatRingOceanGeode->updateOcean(_flatRingOceanGeode->getEye());
		}
		else if (nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
		{
			osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
			_flatRingOceanGeode->setEye(cv->getEyePoint());
		}
		else if (nv->getVisitorType() == osg::NodeVisitor::EVENT_VISITOR)
		{
			osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
			if (ev)
			{
				osg::View* view = dynamic_cast<osg::View*>(ev->getActionAdapter());
				if (view)
				{
					osg::Vec3f centre, up, eye;
					view->getCamera()->getViewMatrixAsLookAt(eye, centre, up);
					_flatRingOceanGeode->setEye(eye);
				}
			}
		}
		traverse(node, nv);
	}
#else

	osg::ref_ptr<FlatOceanDataType> oceanData = dynamic_cast<FlatOceanDataType*> (node->getUserData());

	if (oceanData.valid())
	{
		// If cull visitor update the current eye position
		if (nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
		{
			osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
			oceanData->setEye(cv->getEyePoint());
		}
		else if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
		{
			oceanData->updateOcean();
		}
		else if (nv->getVisitorType() == osg::NodeVisitor::EVENT_VISITOR)
		{
			osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
			if (ev)
			{
				osg::View* view = dynamic_cast<osg::View*>(ev->getActionAdapter());
				if (view)
				{
					osg::Vec3f centre, up, eye;
					view->getCamera()->getViewMatrixAsLookAt(eye, centre, up);
					oceanData->setEye(eye);
				}
			}
		}
		traverse(node, nv);

#endif
	}
}
void FlatRingOceanGeode::setTileResolution(unsigned int s)
{
	if (s == 0)
		return;

	_noiseTileResInv = 1.f / float(s);

	_noiseTileRes = s;

	_isStateDirty = true;
	_isWaveDirty = true;
}
unsigned int FlatRingOceanGeode::getTileResolution()
{
	return _noiseTileRes;
}
unsigned int FlatRingOceanGeode::getTileSize()
{
	return _noiseTileSize;
}
void FlatRingOceanGeode::setTileSize(unsigned int s)
{
	if (s == 0)
		return;
	_noiseTileSize = s;
	_isStateDirty = true;
	_isWaveDirty = true;
}
bool FlatRingOceanGeode::isCrestFoamEnabled()
{
	return _useCrestFoam;
}
void FlatRingOceanGeode::enableCrestFoam(bool on)
{
	if (_useCrestFoam == on)
		return;
	_useCrestFoam = on;
	_isStateDirty = true;
}

void FlatRingOceanGeode::setFoamTopHeight(float value)
{
	if (_foamCapTop == value)
		return;
	_foamCapTop = value;
	_isStateDirty = true;
}
float FlatRingOceanGeode::getFoamTopHeight()
{
	return _foamCapTop;
}
float FlatRingOceanGeode::getReflDampFactor()
{
	return _reflDampFactor;
}
void FlatRingOceanGeode::setReflDampFactor(float value)
{
	if (_reflDampFactor == value)
		return;
	_reflDampFactor = value;
	_isStateDirty = true;
	_isWaveDirty = true;
}
void FlatRingOceanGeode::setFoamBottomHeight(float value)
{
	if (_foamCapBottom == value)
		return;
	_foamCapBottom = value;
	_isStateDirty = true;
}
float FlatRingOceanGeode::getFoamBottomHeight()
{
	return _foamCapBottom;
}

bool FlatRingOceanGeode::areEnableReflections()
{
	return _enableReflections;
}
void FlatRingOceanGeode::enableReflections(bool on)
{
	if (_enableReflections == on)
		return;
	_enableReflections = on;
	_isStateDirty = true;
}

bool FlatRingOceanGeode::areEnableRefractions()
{
	return _enableRefractions;
}
void FlatRingOceanGeode::enableRefractions(bool on)
{
	if (_enableRefractions == on)
		return;
	_enableRefractions = on;
	_isStateDirty = true;
}

float FlatRingOceanGeode::getWaveScaleFactor()
{
	return _noiseWaveScale;
}
void FlatRingOceanGeode::setWaveScaleFactor(float value,bool dirty)
{
	if (_noiseWaveScale == value)
		return;
	_noiseWaveScale = value;
	if(dirty)
		_isStateDirty = true;
	_isWaveDirty = true;
}

float FlatRingOceanGeode::getWindSpeed()
{
	return _noiseWindSpeed;
}
void FlatRingOceanGeode::setWindSpeed(float value,bool dirty)
{
	if (_noiseWindSpeed == value)
		return;
	_noiseWindSpeed = value;
	if(dirty)
		_isStateDirty = true;
	_isWaveDirty = true;
}
void FlatRingOceanGeode::setWindDir(osg::Vec2 dir,bool dirty)
{
	if (_noiseWindDir == dir)
		return;
	_noiseWindDir = dir;
	if(dirty)
		_isStateDirty = true;
}

void FlatRingOceanGeode::setFresnelMul(float value)
{
	if (_fresnelMul == value)
		return;
	_fresnelMul = value;
	_isStateDirty = true;
}
float FlatRingOceanGeode::getFresnelMul()
{
	return _fresnelMul;
}
void FlatRingOceanGeode::setFrameNum(unsigned int value)
{
	if (_NUMFRAMES == value)
		return;
	_NUMFRAMES = value;
	_isWaveDirty = true;
}
unsigned int FlatRingOceanGeode::getFrameNum()
{
	return _NUMFRAMES;
}
void FlatRingOceanGeode::setCycleTime(float value)
{
	if (_cycleTime == value)
		return;
	_cycleTime = value;
	_isWaveDirty = true;
}
float FlatRingOceanGeode::getCycleTime()
{
	return _cycleTime;
}
void FlatRingOceanGeode::setDepth(float value)
{
	if (_depth == value)
		return;
	_depth = value;
	_isWaveDirty = true;
	_isStateDirty = true;
}
float FlatRingOceanGeode::getDepth()
{
	return _depth;
}
void FlatRingOceanGeode::setChoppyFactor(float value)
{
	if (_choppyFactor == value)
		return;
	_choppyFactor = value;
	_isWaveDirty = true;
}
void FlatRingOceanGeode::setLightColor(const osg::Vec4f& color)
{
	if (_lightColor == color)
		return;
	_lightColor = color;
	_isStateDirty = true;
}
osg::Vec4 FlatRingOceanGeode::getLightColor()
{
	return _lightColor;
}
float FlatRingOceanGeode::getChoppyFactor()
{
	return _choppyFactor;
}
void FlatRingOceanGeode::setIsChoppy(bool value,bool dirty)
{
	if (_isChoppy == value)
		return;
	_isChoppy = value;
	if(dirty)
		_isWaveDirty = true;
}
bool FlatRingOceanGeode::isChoppy()
{
	return _isChoppy;
}
osg::Texture2D* FlatRingOceanGeode::getTexture2DFrame(unsigned int index)
{
	if (index < _texturesFrame.size())
		return _texturesFrame.at(index).get();
	return 0;
}
void FlatRingOceanGeode::buildWaveTextures()
{
	//如果显卡有问题，就不运行如下内容
	//2020_5_19在另一张显卡上测试时会出现textureCube导致的INVALID_OPERATION，不知道什么原因
	if (!getVideoCardState())
		return;

	osgOcean::FFTSimulation FFTSim(getTileSize(),getWindDirection(),getWindSpeed(),_depth ,_reflDampFactor,_noiseWaveScale, _noiseTileRes,_cycleTime);

	_texturesFrame.clear();
	_texturesFrame.resize(getFrameNum());

	float _pointSpacing =getTileResolution()/ getTileSize();

	osg::ref_ptr<osg::Uniform> waveTexUnif = new osg::Uniform(osg::Uniform::SAMPLER_2D, "osgOcean_WaterWaveMap", getFrameNum());
	for (unsigned int frame = 0; frame <getFrameNum(); ++frame)
	{
		osg::ref_ptr<osg::FloatArray> heights = new osg::FloatArray;
		osg::ref_ptr<osg::Vec2Array> displacements = NULL;

		if (_isChoppy)
			displacements = new osg::Vec2Array;

		float time = getCycleTime() * (float(frame) / float(getFrameNum()));

		FFTSim.setTime(time);
		FFTSim.computeHeights(heights.get());

		if (_isChoppy)
			FFTSim.computeDisplacements(_choppyFactor, displacements.get());

		osgOcean::OceanTile tile(heights.get(), _noiseTileSize, _pointSpacing, displacements.get());
		_texturesFrame[frame] = convertToR32F(tile);
		if (_texturesFrame[frame].valid())
		{
			waveTexUnif->set(frame, WAVE_MAP + frame);
			getStateSet()->setTextureAttributeAndModes(WAVE_MAP+frame, _texturesFrame[frame].get(), osg::StateAttribute::ON);
		}
	}
	getStateSet()->addUniform(waveTexUnif.get());
	_isWaveDirty = false;
}
bool FlatRingOceanGeode::getVideoCardState()
{
	return _isCardNoWarning;
}
void FlatRingOceanGeode::setVideoCardState(bool on)
{
	if (_isCardNoWarning == on)
		return;
	_isCardNoWarning = on;
	_isStateDirty = true;
	_isWaveDirty = true;
}
osg::Texture2D* FlatRingOceanGeode::convertToR32F(const osgOcean::OceanTile hf)
{
	static unsigned int index = 0;
	unsigned int currentSize = index++;
	std::stringstream imageName;
	imageName << "oceanImage_" << currentSize << ".jpeg";
//	OSG_NOTICE << "convet [" << imageName.str() << "begin!" << std::endl;
	osg::ref_ptr<osg::FloatArray> waveheights = new osg::FloatArray;
	waveheights->resize(hf.getNumVertices());
	for (unsigned int y = 0; y < hf.getRowLen(); ++y)
	{
		for (unsigned int x = 0; x < hf.getRowLen(); ++x)
		{
			waveheights->at(y*hf.getRowLen() + x) = hf.getVertex(x, y).z();

//			OSG_NOTICE << " " <<std::setw(5)<< hf.getVertex(x, y).z();
		}
//		OSG_NOTICE << std::endl;
	}
	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->allocateImage(hf.getRowLen(), hf.getRowLen(), 1, GL_RED, GL_FLOAT);
	image->setInternalTextureFormat(GL_R32F);
	memcpy(image->data(), &waveheights->front(), sizeof(float) * waveheights->size());

	//OSG_NOTICE << "convet [" << imageName.str() << "end!" << std::endl;
	//osgDB::writeImageFile(*image.get(), imageName.str());

	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(image);
	tex->setInternalFormat(GL_R32F);
	tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	tex->setResizeNonPowerOfTwoHint(false);
	tex->setMaxAnisotropy(1.0f);
	return tex.release();
}
bool FlatRingOceanGeode::OceanAnimationEventHandler::handle(osgGA::Event* event, osg::Object* object, osg::NodeVisitor* nv)
{
	osg::ref_ptr<FlatOceanDataType> oceanData = dynamic_cast<FlatOceanDataType*> (object->getUserData());
	if (oceanData.valid())
	{
		osgGA::GUIEventAdapter* ea = event->asGUIEventAdapter();
		if (ea)
		{
			//更新不至于太频繁
			if (ea->getEventType() == osgGA::GUIEventAdapter::FRAME)
			{
				osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
				if (ev)
				{
					osg::View* view = dynamic_cast<osg::View*>(ev->getActionAdapter());
					if (view)
					{
						osg::Vec3f centre, up, eye;
						view->getCamera()->getViewMatrixAsLookAt(eye, centre, up);
						oceanData->setEye(eye);
					}
				}
			}
		}
	}
	return false;
}
FlatRingOceanGeode::EventHandler::EventHandler(osgOcean::OceanTechnique* oceanSurface) :
	osgOcean::OceanTechnique::EventHandler(oceanSurface),
	_autoDirty(true)
{
}

bool FlatRingOceanGeode::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object* object, osg::NodeVisitor* nv)
{
	// Call parent class's handle().
	osgOcean::OceanTechnique::EventHandler::handle(ea, aa, object, nv);

	if (ea.getHandled()) return false;

	// Now we can handle this class's events.
	switch (ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::KEYUP):
	{
		// Downcast to the concrete class we're interested in.
		FlatRingOceanGeode* fftSurface = dynamic_cast<FlatRingOceanGeode*>(_oceanSurface);
		if (!fftSurface) return false;

		// Crest foam
		if (ea.getKey() == 'f')
		{
			fftSurface->enableCrestFoam(!fftSurface->isCrestFoamEnabled());
			osg::notify(osg::NOTICE) << "Crest foam " << (fftSurface->isCrestFoamEnabled() ? "enabled" : "disabled") << std::endl;
			return true;
		}
		// isChoppy
		if (ea.getKey() == 'p')
		{
			fftSurface->setIsChoppy(!fftSurface->isChoppy(), _autoDirty);
			osg::notify(osg::NOTICE) << "Choppy waves " << (fftSurface->isChoppy() ? "enabled" : "disabled") << std::endl;
			return true;
		}
		// Wind speed + 0.5
		if (ea.getKey() == 'W')
		{
			fftSurface->setWindSpeed(fftSurface->getWindSpeed() + 0.5, _autoDirty);
			osg::notify(osg::NOTICE) << "Wind speed now " << fftSurface->getWindSpeed() << std::endl;
			return true;
		}
		// Wind speed - 0.5
		if (ea.getKey() == 'w')
		{
			fftSurface->setWindSpeed(fftSurface->getWindSpeed() - 0.5, _autoDirty);
			osg::notify(osg::NOTICE) << "Wind speed now " << fftSurface->getWindSpeed() << std::endl;
			return true;
		}
		// Scale factor + 1e-9
		if (ea.getKey() == 'K')
		{
			float waveScale = fftSurface->getWaveScaleFactor();
			fftSurface->setWaveScaleFactor(waveScale + (1e-9), _autoDirty);
			osg::notify(osg::NOTICE) << "Wave scale factor now " << fftSurface->getWaveScaleFactor() << std::endl;
			return true;
		}
		// Scale factor - 1e-9
		if (ea.getKey() == 'k')
		{
			float waveScale = fftSurface->getWaveScaleFactor();
			fftSurface->setWaveScaleFactor(waveScale - (1e-9), _autoDirty);
			osg::notify(osg::NOTICE) << "Wave scale factor now " << fftSurface->getWaveScaleFactor() << std::endl;
			return true;
		}
		// Dirty geometry
		if (ea.getKey() == 'd')
		{
			osg::notify(osg::NOTICE) << "Dirtying ocean geometry" << std::endl;
			fftSurface->dirty();
			return true;
		}
		// Toggle autoDirty, if off then individual changes will be 
		// instantaneous but the user will get no feedback until they 
		// dirty manually, if on each change will dirty automatically.
		if (ea.getKey() == 'D')
		{
			_autoDirty = !_autoDirty;
			osg::notify(osg::NOTICE) << "AutoDirty " << (_autoDirty ? "enabled" : "disabled") << std::endl;
			return true;
		}
		// Print out all current settings to the console.
		if (ea.getKey() == 'P')
		{
			osg::notify(osg::NOTICE) << "Current FlatRingOcean settings are:" << std::endl;
			osg::notify(osg::NOTICE) << "  Endless ocean " << (fftSurface->isEndlessOceanEnabled() ? "enabled" : "disabled") << std::endl;
			osg::notify(osg::NOTICE) << "  Crest foam " << (fftSurface->isCrestFoamEnabled() ? "enabled" : "disabled") << std::endl;
			osg::notify(osg::NOTICE) << "  Choppy waves " << (fftSurface->isChoppy() ? "enabled" : "disabled") << std::endl;
			osg::notify(osg::NOTICE) << "  Choppy factor " << fftSurface->getChoppyFactor() << std::endl;
//			osg::notify(osg::NOTICE) << "  Wind direction " << fftSurface->getWindDirection() << std::endl;
			osg::notify(osg::NOTICE) << "  Wind speed " << fftSurface->getWindSpeed() << std::endl;
			osg::notify(osg::NOTICE) << "  Wave scale factor " << fftSurface->getWaveScaleFactor() << std::endl;
			return true;
		}
		break;
	}
	default:
		break;
	}

	return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void FlatRingOceanGeode::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
	// Add parent class's keys too.
	osgOcean::OceanTechnique::EventHandler::getUsage(usage);

	usage.addKeyboardMouseBinding("f", "Toggle crest foam");
	usage.addKeyboardMouseBinding("p", "Toggle choppy waves (dirties geometry if autoDirty is active)");
	usage.addKeyboardMouseBinding("k", "Decrease wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
	usage.addKeyboardMouseBinding("K", "Increase wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
	usage.addKeyboardMouseBinding("w", "Decrease wind speed by 0.5 (dirties geometry if autoDirty is active)");
	usage.addKeyboardMouseBinding("W", "Increase wind speed by 0.5 (dirties geometry if autoDirty is active)");
	usage.addKeyboardMouseBinding("d", "Dirty geometry manually");
	usage.addKeyboardMouseBinding("D", "Toggle autoDirty (if off, changes will require manual dirty)");
	usage.addKeyboardMouseBinding("P", "Print out current ocean surface settings");
}