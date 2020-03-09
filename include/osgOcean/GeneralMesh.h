#ifndef GERNERALMESH_H
#define GERNERALMESH_H
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/Texture2D>
#include <osg/StateSet>
#include <osg/NodeCallback>
#include <osgDB/ReadFile>
#include <osgOcean/SmuGerstnerWave.h>
#include <osgOcean/OceanTechnique>
#include <osgOcean/Export>
namespace osgOcean
{

	class OSGOCEAN_EXPORT GeneralMesh :public OceanTechnique
	{
	public:
		GeneralMesh();
		virtual void build();
		void initStateSet();

		osg::Program* createProgram();

		void setColumns(unsigned int value);
		unsigned int getColumns();

		void setRows(unsigned int value);
		unsigned int getRows();

		void setStartPos(osg::Vec3 value);
		osg::Vec3 getStartPos();

		void setSize(osg::Vec3 value);
		osg::Vec3 getSize();

		void update(float time);

		void setWindSpeed(float value);
		void setWindDir(float value);     //角度设置
		void setWaveSpeed(float value);
		void setWaveDir(float value);    //角度设置
		void setWaveHeight(float value);

		/// Returns the average height over the whole surface (in local space)
		virtual float getSurfaceHeight(void) const;

		/// Returns the height at the given point (in local space). If the 
		/// address of a Vec3f is passed in the normal argument, the normal at
		/// that position will be calculated and stored in it.
		virtual float getSurfaceHeightAt(float x, float y, osg::Vec3f* normal = NULL);

		/**
	* Update animation callback.

	* Update callback calls GeneralMesh::update().
	*/
		class AnimationCallback : public osg::NodeCallback
		{
		public:
			virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
		};

	private:
		bool _isDirty;
		bool _isStateDirty;
		unsigned int numColumns;
		unsigned int numRows;
		osg::Vec3 startPos;
		osg::Vec3 size;
		float windSpeed;
		float windDir;    //角度
		float waveSpeed;
		float waveDir;  //角度
		float waveHeight;
	};
}
#endif