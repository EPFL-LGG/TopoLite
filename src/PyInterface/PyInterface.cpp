#include "PyInterface.h"

int ReadFile(string filename)
{
    boost::filesystem::path filepath(filename);
    if (boost::filesystem::extension(filepath) == ".xml")
    {
        XMLIO xml;
        xml.readXMLFile(filename);
    }
    else if (boost::filesystem::extension(filepath) == ".obj")
    {
        myStrucCreator->LoadSurface((char *) filename.c_str());
        printf("%s", filename.c_str());
        printf("\n");
    }

    if(myStrucCreator->myStruc){
        return myStrucCreator->myStruc->partList.size();
    }

    return 0;
}

int SaveFile(string filename) {
    if (myStrucCreator && myStrucCreator->myStruc) {
        XMLIO xml;
        return xml.SaveXMLFile(filename);
    }
    return 0;
}

