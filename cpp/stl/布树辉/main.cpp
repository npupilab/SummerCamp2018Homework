#include <string>
#include <iostream>
#include <fstream>
#include <vector.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <assert.h>

int main(int argc,char** argv)
{
    summercamp::vector<std::string> strVec;
    assert(strVec.capacity()==0);
    assert(strVec.size()==0);

    strVec.reserve(100);
    assert(strVec.capacity()==100);
    assert(strVec.size()==0);
    assert(strVec.empty());

    strVec.push_back("zhaoyong");
    assert(strVec.size()==1);

    strVec.resize(10,"string");
    assert(strVec.size()==10);
    assert(strVec[1]=="string"&&strVec[0]=="zhaoyong");
    strVec[2]="zy";
    assert(strVec[2]=="zy");

    strVec.reserve(10);
    assert(strVec.capacity()==100);

    strVec.clear();
    assert(strVec.size()==0&&strVec.capacity()==100);
    strVec=summercamp::vector<std::string>();

    summercamp::vector<std::string> b=strVec;
    assert(b.size()==strVec.size());
    assert(b.capacity()==strVec.size());
    for(size_t i=0;i<b.size();i++)
        assert(b[i]==strVec[i]);

    if(argc>=4){
        std::ofstream ofs(argv[3]);
        if(ofs.is_open())
            ofs<<"[S](cpp/stl/"+std::string(argv[2])+"/vector.h)";
    }

    return 0;
}

