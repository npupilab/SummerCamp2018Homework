#include <GSLAM/core/GSLAM.h>

class Initializer
{
    typedef GSLAM::Point3d Point3d;
    typedef GSLAM::Point2d Point2d;
    typedef GSLAM::Point2f Point2f;
public:
    virtual ~Initializer(){}
    virtual bool initialize(const std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> >& matches,
                            const GSLAM::Camera& camera,
                            GSLAM::SE3& t21,
                            std::vector<std::pair<int,GSLAM::Point3d> >& mpts)
    {return false;}

    static  SPtr<Initializer> create(std::string desireType="default");
};

typedef SPtr<Initializer> InitializerPtr;
typedef InitializerPtr (*funcCreateInitializer)();

inline InitializerPtr Initializer::create(std::string desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateInitializer>::instance();
    funcCreateInitializer createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return InitializerPtr();
    return createFunc();
}

#define REGISTER_INITIALIZER(D,E) \
    extern "C" InitializerPtr create##E(){ return InitializerPtr(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateInitializer>::instance().insert(#E,create##E);\
}}D##E##_instance;
