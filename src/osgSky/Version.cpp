#include <osgSky/Version>
#include <string>
#include <stdio.h>

const char* osgOceanGetVersion()
{
    static char ocean_version[256];
    static int ocean_version_init = 1;
    
    if( ocean_version_init )
    {
        if( OSGSKY_VERSION_REVISION == 0 )
            sprintf( ocean_version, "%d.%d.%d", OSGSKY_VERSION_MAJOR, OSGSKY_VERSION_MINOR, OSGSKY_VERSION_RELEASE );
        else
            sprintf( ocean_version, "%d.%d.%d-%d", OSGSKY_VERSION_MAJOR, OSGSKY_VERSION_MINOR, OSGSKY_VERSION_RELEASE, OSGSKY_VERSION_REVISION );

        ocean_version_init = 0;
    }
    
    return ocean_version;
}

const char* osgOceanGetLibraryName()
{
    return "osgOcean Library";
}