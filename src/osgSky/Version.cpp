#include <osgSky/Version>
#include <string>
#include <stdio.h>

const char* osgSkyGetVersion()
{
    static char sky_version[256];
    static int sky_version_init = 1;
    
    if( sky_version_init )
    {
        if( OSGSKY_VERSION_REVISION == 0 )
            sprintf( sky_version, "%d.%d.%d", OSGSKY_VERSION_MAJOR, OSGSKY_VERSION_MINOR, OSGSKY_VERSION_RELEASE );
        else
            sprintf( sky_version, "%d.%d.%d-%d", OSGSKY_VERSION_MAJOR, OSGSKY_VERSION_MINOR, OSGSKY_VERSION_RELEASE, OSGSKY_VERSION_REVISION );

        sky_version_init = 0;
    }
    
    return sky_version;
}

const char* osgSkyGetLibraryName()
{
    return "osgOcean Library";
}