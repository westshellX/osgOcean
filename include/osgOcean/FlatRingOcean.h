#ifndef FLATRECTOCEAN_H
#define FLATRECTOCEAN_H
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgOcean/Export>

//不太确定是否使用setUserData(new FlatOceanDataType(*this)); 0:使用， 1：不使用
#define MODE_2018_9_23   0

namespace osgOcean
{
	class OSGOCEAN_EXPORT FlatRingOceanGeode : public osg::Geode
	{
	public:
		FlatRingOceanGeode(float in=10.0,float out=100.0,unsigned int cSteps=10,unsigned int rSteps=10);

		~FlatRingOceanGeode();

		void SetInR(float w);
		float GetInR();

		void SetOutR(float l);
		float GetOutR();

		float GetOceanWidth();

		inline void SetHeight(float h)
		{
			_height=h;
			_isDirty=true;
		}

		inline float GetHeight(){return _height;}

		inline unsigned int GetCircleSteps(){return _circleSteps;}
		inline void SetCirlceSteps(unsigned int wS)
		{
			_circleSteps=wS;
			_isDirty=true;
		}

		inline unsigned int GetRSteps(){return _rSteps;}
		inline void SetRSteps(unsigned int lS)
		{
			_rSteps=lS;
			_isDirty=true;
		}

		inline void SetCenterPoint(osg::Vec2 pos)
		{
			_centerPoint=pos;
		}
		inline bool upDateCenterPoint( const osg::Vec3f& eye)
		{
			bool update=false;
			osg::Vec2 moveVector=osg::Vec2(eye.x()-GetCenterPoint().x(),eye.y()-GetCenterPoint().y());

			//2018_9_23 14:00 优化一点是一点
			if(moveVector.length()>0.25*GetInR())
				update=true;
			return update;
		}
		inline osg::Vec2 GetCenterPoint(){return _centerPoint;}

		inline void enableReflections(bool able)
		{
			if(_enableReflections==able)
				return;
			_enableReflections=able;
			getOrCreateStateSet()->getOrCreateUniform("osgOcean_EnableReflections",osg::Uniform::BOOL)->set(_enableReflections);
		}
		inline void setLightID( unsigned int id ){
			if(_lightID == id)
				return;
			_lightID=id;
			getOrCreateStateSet()->getOrCreateUniform("osgOcean_LightID",osg::Uniform::UNSIGNED_INT)->set(_lightID);
		}

		inline void SetSkyNodeSet(bool able)
		{
			if(_skyNodeSet==able)
				return;
			_skyNodeSet=able;
			getOrCreateStateSet()->getOrCreateUniform("skyLoadSet",osg::Uniform::BOOL)->set(_skyNodeSet);
		}

		void build(float h=0.0);

		//主要是设置远处海平面的颜色
		void initStateSet();

		void UpdateOcean(const osg::Vec3f& eye);

		inline void SetWaveTopColor(osg::Vec3 color)
		{
			_waveTopColor=color;
			osg::Vec4f waveTop = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveTopColor,1.f) );
			getOrCreateStateSet()->getOrCreateUniform("osgOcean_WaveTop",osg::Uniform::FLOAT_VEC4)->set(osg::Vec4(waveTop));
		}
		inline void SetWaveBottomColor(osg::Vec3 color)
		{
			_waveBottomColor=color;
			osg::Vec4f waveBot = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveBottomColor,1.f) );
			getOrCreateStateSet()->getOrCreateUniform("osgOcean_WaveBot",osg::Uniform::FLOAT_VEC4)->set(waveBot);
		}
		inline void SetEnvironmentMap(osg::TextureCubeMap* environmentMap)
		{
			_environmentMap=environmentMap;
			_isStateDirty=true;
		}
		inline osg::TextureCubeMap* GetEnvironmetMap()
		{
			return _environmentMap.get();
		}
		osg::Program* createShader();

		/**
		* Linear interpolation of 3 RGBA colors.
		* @return interpolated color (vec4)
		*/
		inline osg::Vec4f colorLerp (const osg::Vec4f& c0, const osg::Vec4f& c1, const osg::Vec4f& c2) const 
		{
			return osg::Vec4f(
				c1[0]*(1-c0[0]) + c2[0]*c0[0],
				c1[1]*(1-c0[1]) + c2[1]*c0[1],
				c1[2]*(1-c0[2]) + c2[2]*c0[2],
				c1[3]*(1-c0[3]) + c2[3]*c0[3]
			);
		}

		/**
		* Compute noise coordinates for the fragment shader.
		* @param noiseSize Size of noise tile (m).
		* @param movement Number of tiles moved x,y.
		* @param speed Speed of movement(m/s).
		* @parem time Simulation Time.
		*/
		osg::Vec3f computeNoiseCoords(float noiseSize, const osg::Vec2f& movement, float speed, float time);

		osg::Texture2D* createTexture( const std::string& path, osg::Texture::WrapMode wrap );

		inline void SetFogColor(osg::Vec4 color)
		{
			if(_fogColor==color)
				return;
			_fogColor=color;
			getOrCreateStateSet()->getOrCreateUniform("osgOcean_AboveWaterFogColor",osg::Uniform::FLOAT_VEC4)->set(_fogColor);
		}
		inline osg::Vec4 GetFogColor(){return _fogColor;}

		inline void SetFogDensity(float f)
		{
			if(_fogDensity==f)
				return;
			_fogDensity=f;

			const float LOG2E = 1.442695;
			{
				osg::ref_ptr<osg::Uniform> uniform=getOrCreateStateSet()->getOrCreateUniform("osgOcean_AboveWaterFogDensity",osg::Uniform::FLOAT);
				//与OceanScene保持一致
				uniform->set(-GetFogDensity()*GetFogDensity()*LOG2E);
			}
		}
		inline float GetFogDensity(){return _fogDensity;}

		void createOceanGeometry();

		void computeVertices();

		void computePrimitives();


		void setEye(osg::Vec3 pos);
		osg::Vec3 getEye();


	private:
		float _inR;    //与FFTOceanSurface的宽度相同
		float _outR;

		float _height;//平面高度

		unsigned int _circleSteps;
		unsigned int _rSteps;

		bool _isDirty;
		bool _isStateDirty;

		osg::Vec2f _centerPoint;

		osg::Vec4f _lightColor;
		osg::Vec3f _waveTopColor;   //远处的海水颜色
		osg::Vec3f _waveBottomColor;

		osg::ref_ptr<osg::TextureCubeMap> _environmentMap;
		enum TEXTURE_UNITS{ MAP2D=0,MAP_DARK=1,ENV_MAP=2 };

		bool _enableReflections;
		bool _skyNodeSet;  //加载的是天空模型
		osg::Vec4 _fogColor;
		float _fogDensity;

		unsigned int _lightID;

		osg::ref_ptr<osg::Geometry> _geom;

		osg::ref_ptr<osg::Vec3Array> _vertices;
		osg::ref_ptr<osg::Vec3Array> _normals;
		osg::ref_ptr<osg::Vec2Array> _texcoords;
		osg::ref_ptr<osg::Vec4Array> _colors;

#if MODE_2018_9_23
		osg::Vec3 _eye;

#else
	public:
		/**
		* Data structure to store data needed for animation callback.
		*/
		class OSGOCEAN_EXPORT FlatOceanDataType: public osg::Referenced
		{
		private:
			osgOcean::FlatRingOceanGeode& _oceanSurface;
			osg::Vec3f _eye;
		public:
			FlatOceanDataType(  FlatRingOceanGeode& ocean);
			FlatOceanDataType( const FlatOceanDataType& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

			inline void setEye( const osg::Vec3f& eye ){ _eye = eye;    }
			inline osg::Vec3 getEye(){return _eye;}
			/**
			* @see FFTOceanSurface::update();
			*/
			void updateOcean(void);
		};
#endif
		public:
			class FlatRingOceanGeodeCallback : public osg::NodeCallback
			{
				virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
			};
	};
}
#endif