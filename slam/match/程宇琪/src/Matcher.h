#pragma once
#include <GSLAM/core/GSLAM.h>

class Matcher
{
public:
    virtual ~Matcher(){}
    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const {
        return false;}

    virtual bool match4triangulation(const GSLAM::FramePtr& ref,const GSLAM::FramePtr& cur,
                                     std::vector<std::pair<int,int> >& matches)const {return false;}

    virtual bool findMatchWindow(const GSLAM::GImage& des,const GSLAM::FramePtr& fr,
                                 const float& x,const float& y,const float& r,
                                 int& idx,bool discardMapPoints=true)const{return false;}

    virtual bool findMatchEpipolarLine(const GSLAM::GImage&  des , const GSLAM::FramePtr& fr,
                                       const GSLAM::Point3d& line, const float& r,
                                       int& idx, bool discardMapPoints=true){return false;}


    static SPtr<Matcher> create(std::string desireType="default");
};

typedef SPtr<Matcher> MatcherPtr;
typedef MatcherPtr (*funcCreateMatcher)();

inline MatcherPtr Matcher::create(std::string desireType){
    auto& inst=GSLAM::SvarWithType<funcCreateMatcher>::instance();
    funcCreateMatcher createFunc=inst.get_var(desireType,NULL);
    if(!createFunc) return MatcherPtr();
    return createFunc();
}

#define REGISTER_MATCHER(D,E) \
    extern "C" MatcherPtr create##E(){ return MatcherPtr(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::SvarWithType<funcCreateMatcher>::instance().insert(#E,create##E);\
}}D##E##_instance;
