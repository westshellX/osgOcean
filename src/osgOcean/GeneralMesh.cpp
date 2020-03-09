#include <osgOcean/GeneralMesh.h>

using namespace osgOcean;

GeneralMesh::GeneralMesh()
	:_isDirty(true)
	,_isStateDirty(true)
	, numColumns(38)
	, numRows(39)
	, startPos(0.0, 0.0, 0.0)
	, size(1000.0, 1000.0, 1000.0)
	,windSpeed(0)
	,windDir(45.0)
	,waveSpeed(0.51444)    //初始1节流
	,waveDir(90.0)
	,waveHeight(10.0)       //4级风对应的浪高是1米
{
	setUpdateCallback(new AnimationCallback);
	build();
}
void GeneralMesh::build()
{
	if (getNumDrawables() > 0)
		removeDrawables(0, getNumDrawables());

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;

	osg::Vec3Array& v = *(new osg::Vec3Array(numColumns*numRows));
	osg::Vec4ubArray& color = *(new osg::Vec4ubArray(1));
	osg::Vec2Array* texcoords = new osg::Vec2Array();

	color[0].set(255, 255, 255, 255);

	float rowCoordDelta = size.y() / (float)(numRows - 1);
	float columnCoordDelta = size.x() / (float)(numColumns - 1);

	float rowTexDelta = 1.0f / (float)(numRows - 1);
	float columnTexDelta = 1.0f / (float)(numColumns - 1);

	osg::Vec3 pos = startPos;
	osg::Vec2 tex(0.0f, 0.0f);
	int vi = 0;
	unsigned int r = 0, c = 0;
	for (r = 0; r < numRows; ++r)
	{
		pos.x() = startPos.x();
		tex.x() = 0.0f;
		for (c = 0; c < numColumns; ++c)
		{
			v[vi].set(pos.x(), pos.y(), pos.z());
			texcoords->push_back(tex);

			pos.x() += columnCoordDelta;
			tex.x() += columnTexDelta;
			++vi;
		}
		pos.y() += rowCoordDelta;
		tex.y() += rowTexDelta;
	}

	geometry->setVertexArray(&v);
	geometry->setTexCoordArray(0, texcoords);
	geometry->setColorArray(&color, osg::Array::BIND_OVERALL);

	for (r = 0; r < numRows - 1; ++r)
	{
		osg::DrawElementsUShort& drawElements = *(new osg::DrawElementsUShort(GL_QUAD_STRIP, 2 * numColumns));
		geometry->addPrimitiveSet(&drawElements);
		int ei = 0;
		for (c = 0; c < numColumns; ++c)
		{
			drawElements[ei++] = (r + 1)*numColumns + c;
			drawElements[ei++] = (r)*numColumns + c;
		}
	}

	geometry->setInitialBound(osg::BoundingBox(startPos, startPos + size));

	addDrawable(geometry);

	initStateSet();

	_isDirty = false;
}
void GeneralMesh::initStateSet()
{
	osg::Vec3 scaleDown(1.0f / size.x(), 1.0f / size.y(), 1.0f / size.z());
	// ---------------------------------------
// Set up a StateSet to texture the objects
// ---------------------------------------
	osg::ref_ptr<osg::StateSet> stateset = getOrCreateStateSet();

	osg::ref_ptr<osg::Uniform> originUniform = new osg::Uniform("terrainOrigin", getStartPos());
	stateset->addUniform(originUniform);

	osg::ref_ptr<osg::Uniform> sizeUniform = new osg::Uniform("terrainSize", getSize());
	stateset->addUniform(sizeUniform);

	osg::ref_ptr<osg::Uniform> scaleDownUniform = new osg::Uniform("terrainScaleDown", scaleDown);
	stateset->addUniform(scaleDownUniform);

	osg::ref_ptr<osg::Uniform> terrainTextureSampler = new osg::Uniform("terrainTexture", 0);
	stateset->addUniform(terrainTextureSampler);

	osg::ref_ptr<osg::Uniform> baseTextureSampler = new osg::Uniform("baseTexture", 1);
	stateset->addUniform(baseTextureSampler);

	osg::ref_ptr<osg::Uniform> treeTextureSampler = new osg::Uniform("treeTexture", 1);
	stateset->addUniform(treeTextureSampler);

	//float scale_z = size.z() / (max_z - min_z);

	//osg::Image* terrainImage = new osg::Image;
	//terrainImage->allocateImage(numColumns, numRows, 1, GL_LUMINANCE, GL_FLOAT);
	//terrainImage->setInternalTextureFormat(GL_LUMINANCE_FLOAT32_ATI);
	//for (r = 0; r < numRows; ++r)
	//{
	//	for (c = 0; c < numColumns; ++c)
	//	{
	//		*((float*)(terrainImage->data(c, r))) = (vertex[r + c * numRows][2] - min_z)*scale_z;
	//	}
	//}
	//osg::ref_ptr<osg::Texture2D> terrainTexture = new osg::Texture2D;
	//terrainTexture->setImage(terrainImage);
	//terrainTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
	//terrainTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
	//terrainTexture->setResizeNonPowerOfTwoHint(false);
	//stateset->setTextureAttributeAndModes(0, terrainTexture, osg::StateAttribute::ON);


	osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile("Images/lz.rgb");
	if (image)
	{
		osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;

		texture->setImage(image);
		stateset->setTextureAttributeAndModes(1, texture, osg::StateAttribute::ON);
	}
	stateset->setAttributeAndModes(createProgram(), osg::StateAttribute::ON);

	osg::ref_ptr<osg::FloatArray>_constants = new osg::FloatArray();

	// reset, create and pack trochoids
	SmuGerstnerWave _trochoids(0.05f, 0.25f, 18.f, 1.2f, 1.f, 0.2f);
	_trochoids.setWaveDirDegree(waveDir);
	_trochoids.setWaveSpeed(waveSpeed);
	_trochoids.setWindDirDegree(windDir);
	_trochoids.setWaveHeight(waveHeight);
	_trochoids.createWaves();
	_trochoids.packWaves(_constants.get());

	//_stateSet = new osg::StateSet;

	//osg::BlendFunc *blend = new osg::BlendFunc;
	//blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE);

	osg::ref_ptr<osg::Uniform> waveUniform = new osg::Uniform(osg::Uniform::FLOAT, "osgOcean_Waves", (int)_constants->size());
	waveUniform->setArray(_constants.get());
	stateset->addUniform(waveUniform);

	//时间
	osg::ref_ptr<osg::Uniform> timeUniform = new osg::Uniform("time", (float)0.0);
	stateset->addUniform(timeUniform);

	_isStateDirty = false;
}
osg::Program* GeneralMesh::createProgram()
{

	osg::ref_ptr<osg::Program> program = new osg::Program;

	///////////////////////////////////////////////////////////////////
	// vertex shader using just Vec4 coefficients
	char vertexShaderSource[] =
		"uniform sampler2D terrainTexture;\n"
		"uniform vec3 terrainOrigin;\n"
		"uniform vec3 terrainScaleDown;\n"
		"\n"
		"varying vec2 texcoord;\n"
		"\n"
		"void main(void)\n"
		"{\n"
		"    texcoord = gl_Vertex.xy - terrainOrigin.xy;\n"
		"    texcoord.x *= terrainScaleDown.x;\n"
		"    texcoord.y *= terrainScaleDown.y;\n"
		"\n"
		"    vec4 position;\n"
		"    position.x = gl_Vertex.x;\n"
		"    position.y = gl_Vertex.y;\n"
		"    position.z = texture2D(terrainTexture, texcoord).r;\n"
		"    position.w = 1.0;\n"
		" \n"
		"    gl_Position     = gl_ModelViewProjectionMatrix * position;\n"
		"   gl_FrontColor = vec4(1.0,1.0,1.0,1.0);\n"
		"}\n";

	//////////////////////////////////////////////////////////////////
	// fragment shader
	//
	char fragmentShaderSource[] =
		"uniform sampler2D baseTexture; \n"
		"varying vec2 texcoord;\n"
		"\n"
		"void main(void) \n"
		"{\n"
		"    gl_FragColor = texture2D( baseTexture, texcoord); \n"
		"}\n";


	program->setName("gerneralMeshProgram");

	std::string fileName = "generalMesh.vert";
	osg::ref_ptr<osg::Shader> verShader = osgDB::readRefShaderFile(fileName);
	if (!verShader.valid())
	{
		osg::notify(osg::WARN) << "File \"" << fileName << "\" not found." << std::endl;
		verShader = new osg::Shader(osg::Shader::VERTEX, vertexShaderSource);
	}
	program->addShader(verShader.get());

	fileName = "generalMesh.frag";
	osg::ref_ptr<osg::Shader> fragShader = osgDB::readRefShaderFile(fileName);
	if (!fragShader.valid())
	{
		osg::notify(osg::WARN) << "File \"" << fileName << "\" not found." << std::endl;
		fragShader = new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource);
	}
	program->addShader(fragShader.get());

	return program.release();
}
void GeneralMesh::setColumns(unsigned int value)
{
	if (numColumns == value)
		return;
	numColumns = value;
	_isDirty = true;
}
unsigned int GeneralMesh::getColumns()
{
	return numColumns;
}

void GeneralMesh::setRows(unsigned int value)
{
	if (numRows == value)
		return;
	numRows = value;
	_isDirty = true;
}
unsigned int GeneralMesh::getRows()
{
	return numRows;
}

void GeneralMesh::setStartPos(osg::Vec3 value)
{
	if (startPos == value)
		return;
	startPos = value;
	_isDirty = true;
}
osg::Vec3 GeneralMesh::getStartPos()
{
	return startPos;
}

void GeneralMesh::setSize(osg::Vec3 value)
{
	if (size == value)
		return;
	size = value;
	_isDirty = true;
}
osg::Vec3 GeneralMesh::getSize()
{
	return size;
}
void GeneralMesh::setWindSpeed(float value)
{
	if (windSpeed == value)
		return;
	windSpeed = value;
	_isStateDirty = true;
}
void GeneralMesh::setWindDir(float value)
{
	if (windDir == value)
		return;
	windDir = value;
	_isStateDirty = true;
}
void GeneralMesh::setWaveSpeed(float value)
{
	if (waveSpeed == value)
		return;
	waveSpeed = value;
	_isStateDirty = true;
}
void GeneralMesh::setWaveDir(float value)
{
	if (waveDir == value)
		return;
	waveDir = value;
	_isStateDirty = true;
}
void GeneralMesh::setWaveHeight(float value)
{
	if (waveHeight == value)
		return;
	waveHeight = value;
	_isStateDirty = true;
}
float GeneralMesh::getSurfaceHeight()const
{
	OSG_WARN<< "Warning: the surfaceHeight is the waveHeight in this methods!" << std::endl;
	return waveHeight;
}
float GeneralMesh::getSurfaceHeightAt(float x, float y, osg::Vec3f* normal /* = NULL */)
{
	OSG_WARN<<"Warning: the surfaceHeight is the waveHeight in this methods!" << std::endl;
	return waveHeight;
}
void GeneralMesh::update(float time)
{
	if (_isDirty)
		build();
	if (_isStateDirty)
		initStateSet();

	getStateSet()->getUniform("time")->set(time);
}
void GeneralMesh::AnimationCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	osg::ref_ptr<GeneralMesh> data = dynamic_cast<GeneralMesh*>(node);
	if (data.valid())
	{
		if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
		{
			data->update(nv->getFrameStamp()->getSimulationTime());
		}
	}
}