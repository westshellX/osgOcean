#include <osgOcean/FlatRingOcean>
#include <osgUtil/CullVisitor>
#include <osgGA/EventVisitor>
#include <osg/Material>
#include <fstream>
#include <osgDB/FileUtils>
#include <osg/Notify>
#include <osgOcean/ShaderManager>
#include <sstream>
#define RESOLUTION 256
#define INITWAVESCALE 1e-8
#define REFLECTIONDAMPING 0.35
using namespace osgOcean;

FlatRingOceanGeode::FlatRingOceanGeode(float w, float out, unsigned int cSteps, unsigned int rSteps)
	:_inR(w)
	, _outR(out)
	, _height(0.0)
	, _circleSteps(cSteps)
	, _rSteps(rSteps)
	, _isDirty(true)
	, _isStateDirty(true)
	, _centerPoint(0.0, 0.0)
	, _lightColor(0.411764705f, 0.54117647f, 0.6823529f, 1.f)
	, _waveTopColor(0.192156862f, 0.32549019f, 0.36862745098f)   //直接从FFTOceanSurface中复制过来的颜色
	, _waveBottomColor(0.11372549019f, 0.219607843f, 0.3568627450f)
	, _enableReflections(false)
	,_enableRefractions(false)
	, _skyNodeSet(false)
	, _lightID(0)
	,_useCrestFoam(false),
	_foamCapBottom(2.2f),
	_foamCapTop(3.0f),
	_tileResolution(RESOLUTION),
	_tileResInv(1.f / float(RESOLUTION)),
	_fresnelMul(0.7),
	_noiseTileSize(32),
	_noiseTileRes(RESOLUTION),
	_noiseWindDir(osg::Vec2(1.0,1.0)),
	_noiseWindSpeed(6.0),
	_noiseWaveScale(INITWAVESCALE),
	_depth(1000.0),
	_reflDampFactor(REFLECTIONDAMPING),
	_aboveWaterFogDensity(0.0012f),
	_aboveWaterFogColor(osg::Vec4(199/255.0, 226/255.0, 255/255.0,1.0)),
	 _environmentMap(0)
	, _geom(new osg::Geometry)
	, _vertices(new osg::Vec3Array)
	, _normals(new osg::Vec3Array)
	, _texcoords(new osg::Vec2Array)
	, _colors(new osg::Vec4Array)
	,_isEndless(false)
#if MODE_2018_9_23 
	, _eye(osg::Vec3d(0.0, 0.0, 0.0))
{
#else
{
		int _NUMFRAMES = 256;
		setUserData(new FlatOceanDataType(*this, _NUMFRAMES, 25));
#endif
		osg::ref_ptr<FlatRingOceanGeodeCallback> _callback = new FlatRingOceanGeodeCallback;
		setEventCallback(_callback.get());
		setUpdateCallback(_callback.get());
		setCullCallback(_callback.get());
		osgOcean::ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
}

FlatRingOceanGeode::~FlatRingOceanGeode()
{
}

void FlatRingOceanGeode::SetInR(float w)
{
	_inR = w;
	_isDirty = true;
}
float FlatRingOceanGeode::GetInR()
{
	return _inR;
}
void FlatRingOceanGeode::SetOutR(float l)
{
	_outR = l;
	_isDirty = true;
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

	_isDirty = false;
	_isStateDirty = false;
}
void FlatRingOceanGeode::createOceanGeometry()
{
	if (getNumDrawables() > 0)
		removeDrawables(0, getNumDrawables());
	if (!_geom.valid())
		_geom = new osg::Geometry();

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
	// First clear old primitive sets
	if (_geom->getNumPrimitiveSets() > 0)
		_geom->removePrimitiveSet(0, _geom->getNumPrimitiveSets());

	for (unsigned int r = 0; r < GetRSteps(); r++)
	{
		osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);
		for (unsigned int c = 0; c <= GetCircleSteps(); c++)
		{
			indices->push_back(c + r * (GetCircleSteps() + 1));
			indices->push_back(c + (r + 1)*(GetCircleSteps() + 1));
		}
		_geom->addPrimitiveSet(indices.get());
	}

	//2018_9_23 14:00
	dirtyBound();
}
void FlatRingOceanGeode::computeVertices()
{
	_vertices->resize((GetRSteps() + 1)*(GetCircleSteps() + 1));
	_texcoords->resize((GetRSteps() + 1)*(GetCircleSteps() + 1));

	unsigned int ptr = 0;

	double x, y, z, R, sin_t, cos_t;
	double thetaDelta = 2 * osg::PI / GetCircleSteps();
	double RDelta = (GetOutR() - GetInR()) / GetRSteps();

	double scale;

	for (unsigned int i = 0; i <= GetRSteps(); i++)
	{
		for (unsigned int j = 0; j <= GetCircleSteps(); j++)
		{
			R = GetInR() + i * RDelta;
			sin_t = sin(j*thetaDelta);
			cos_t = cos(j*thetaDelta);
			x = GetCenterPoint().x() + R * cos_t;
			y = GetCenterPoint().y() + R * sin_t;
			z = GetHeight();

			(*_vertices)[ptr] = osg::Vec3(x, y, z);
			scale = R / GetOutR();
			(*_texcoords)[ptr] = osg::Vec2(0.5*(1 + scale * cos_t), 0.5*(1 + scale * sin_t));
			++ptr;
		}
	}

	_normals->resize(1);
	(*_normals)[0] = osg::Vec3(0.0, 0.0, 1.0);

	_colors->resize(1);
	(*_colors)[0] = osg::Vec4(1.f, 1.f, 1.f, 1.f);
}
void FlatRingOceanGeode::initStateSet()
{
	osg::notify(osg::INFO) << "FFTOceanSurface::initStateSet()" << std::endl;
	_stateset = new osg::StateSet;

	_stateset->getOrCreateUniform("osgOcean_LightID", osg::Uniform::UNSIGNED_INT)->set(_lightID);
	const float LOG2E = 1.442695;

	_stateset->getOrCreateUniform("osgOcean_AboveWaterFogDensity", osg::Uniform::Type::FLOAT)->set(-_aboveWaterFogDensity * _aboveWaterFogDensity*LOG2E);
	_stateset->getOrCreateUniform("osgOcean_AboveWaterFogColor", osg::Uniform::Type::FLOAT_VEC4)->set(_aboveWaterFogColor);
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
	_stateset->addUniform(new osg::Uniform("osgOcean_FoamScale", _tileResInv*30.f));

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

	osg::notify(osg::INFO) << "FFTOceanSurface::initStateSet() Complete." << std::endl;
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
	float diff = GetOutR() - GetInR();
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
	std::string vertFile = "water.vert";
	std::string fragmentFile = "water.frag";

	return osgOcean::ShaderManager::instance().createProgram(shaderName, vertFile, fragmentFile, true);
}

void FlatRingOceanGeode::UpdateOcean(const osg::Vec3f& eye,const double& dt)
{
	if (_isDirty)
		build(GetHeight());
	else if (_isStateDirty)
		initStateSet();

	getOrCreateStateSet()->getOrCreateUniform("osgOcean_EyePosition", osg::Uniform::FLOAT_VEC3)->set(eye);
	static double time = 0.0;
	time += (dt*0.0008);

	getStateSet()->getUniform("osgOcean_NoiseCoords0")->set(computeNoiseCoords(32.f, osg::Vec2f(2.f, 4.f), 2.f, time));
	getStateSet()->getUniform("osgOcean_NoiseCoords1")->set(computeNoiseCoords(8.f, osg::Vec2f(-4.f, 2.f), 1.f, time));
	if (upDateCenterPoint(eye))
	{
		SetCenterPoint(osg::Vec2(eye.x(), eye.y()));
		computeVertices();
		computePrimitives();
	}
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

	//_oceanSurface.update(_frame, dt, _eye);
	_oceanSurface.UpdateOcean(_eye,dt);
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
			_flatRingOceanGeode->UpdateOcean(_flatRingOceanGeode->getEye());
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
	_tileResolution = s;

	_tileResInv = 1.f / float(s);

	_noiseTileRes = s;

	_isStateDirty = true;
}
unsigned int FlatRingOceanGeode::getTileResolution()
{
	return _tileResolution;
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
}
bool FlatRingOceanGeode::isUseCrestFoam()
{
	return _useCrestFoam;
}
void FlatRingOceanGeode::enableUseCrestFoam(bool on)
{
	if (_useCrestFoam == on)
		return;
	_useCrestFoam = on;
	_isStateDirty = true;
}

void FlatRingOceanGeode::setFoamCapTop(float value)
{
	if (_foamCapTop == value)
		return;
	_foamCapTop = value;
	_isStateDirty = true;
}
float FlatRingOceanGeode::getFoamCapTop()
{
	return _foamCapTop;
}

void FlatRingOceanGeode::setFoamCapBottom(float value)
{
	if (_foamCapBottom == value)
		return;
	_foamCapBottom = value;
	_isStateDirty = true;
}
float FlatRingOceanGeode::getFoamCapBottom()
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

float FlatRingOceanGeode::getNoiseWaveScale()
{
	return _noiseWaveScale;
}
void FlatRingOceanGeode::setNoiseWaveScale(float value)
{
	if (_noiseWaveScale == value)
		return;
	_noiseWaveScale = value;
	_isStateDirty = true;
}

float FlatRingOceanGeode::getNoiseWindSpeed()
{
	return _noiseWindSpeed;
}
void FlatRingOceanGeode::setNoiseWindSpeed(float value)
{
	if (_noiseWindSpeed == value)
		return;
	_noiseWindSpeed = value;
	_isStateDirty = true;
}

osg::Vec2 FlatRingOceanGeode::getNoiseWindDir()
{
	return _noiseWindDir;
}
void FlatRingOceanGeode::setNoiseWindDir(osg::Vec2 dir)
{
	if (_noiseWindDir == dir)
		return;
	_noiseWindDir = dir;
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