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

#ifndef SMUGERSTNERWAVE_H
#define SMUGERSTNERWAVE_H

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <osg/Array>
#include <osg/Math>
#include <osgOcean/Export>

namespace osgOcean
{
	class  OSGOCEAN_EXPORT SmuGerstnerWave
	{
	private:
		static const int NUM_WAVES = 16;

		// Wave data structure
		struct Wave {
			float A;        // amplitude
			float w;        // frequency
			float kx, ky;    // wave vector components
			float kmod;     // wave vector module
			float phi0;     // phase at time 0
			float phase;    // current phase
			float Ainvk;    // amplitude/K module
		};
		// Parameters used for the waves initialization
		float _amplitude;    // base amplitude (must be <1.0 or they loop over themselves)
		float _amplitudeMul; // amplitude multiplier
		float _lambda0;      // base wavelength
		float _lambdaMul;    // wavelength multiplier
		float _direction;    // main waves direction        流向 单位：弧度
		float _angleDev;     // deviation from main direction 可以认为是风向 单位：弧度
		float _waveSpeed;    // 米每秒
	public:
		std::vector<Wave> _waves;

	public:
		SmuGerstnerWave(void);

		SmuGerstnerWave(const SmuGerstnerWave& copy);

		SmuGerstnerWave(float amplitude,
			float amplitudeMul,
			float baseWavelen,
			float wavelenMul,
			float direction,
			float angleDev);

		~SmuGerstnerWave(void);

		// create base waves
		void createWaves(void);

		// update waves
		void updateWaves(float time);

		// pack waves for vertex shader
		void packWaves(osg::FloatArray* constant) const;

		inline void setWaveDirDegree(float value)
		{
			_direction = osg::DegreesToRadians(value);
		}
		inline float getWaveDirRadians() { return _direction; }
		inline float getWaveDirDegree() { return osg::RadiansToDegrees(_direction); }

		inline void setWaveSpeed(float value)
		{
			_waveSpeed = value;
			_lambda0 = _waveSpeed;
			if (value == 0)
				_lambda0 = 14.0;
		}
		inline float getWaveSpeed() { return _waveSpeed; }

		inline void setWindDirDegree(float value)
		{
			_angleDev = osg::DegreesToRadians(value);
		}

		inline float getWindDirRadians() { return _angleDev; }
		inline float getWindDirDegree() { return osg::RadiansToDegrees(_angleDev); }

		inline void setWaveHeight(float value)
		{
			_amplitude = value;
		}
		inline float getWaveHeght()
		{
			return _amplitude;
		}
	private:

		inline float nextRandomDouble(float lBound, float uBound)
		{
			static int seed = 0;
			srand(seed);
			seed++;
			return (float)rand() / (float)RAND_MAX * (uBound - lBound) + lBound;
		}
	};
}
#endif
