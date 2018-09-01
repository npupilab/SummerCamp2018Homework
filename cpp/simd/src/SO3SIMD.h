#include "SO3.h"
#include <xmmintrin.h>
#include <GSLAM/core/Camera.h>

/**
return SO3( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
            w * rq.y + y * rq.w + z * rq.x - x * rq.z,
            w * rq.z + z * rq.w + x * rq.y - y * rq.x,
            w * rq.w - x * rq.x - y * rq.y - z * rq.z);
**/
inline pi::SO3f mul(pi::SO3f& l,const pi::SO3f& r)
{
    __m128 r1=_mm_mul_ps(_mm_load1_ps(&l.w),*(__m128*)&r);
    r1=_mm_add_ps(r1,_mm_mul_ps(_mm_setr_ps(l.x,l.y,l.z,-l.x),
                                _mm_setr_ps(r.w,r.w,r.w,r.x)));
    r1=_mm_add_ps(r1,_mm_mul_ps(_mm_setr_ps(l.y,l.z,l.x,-l.y),
                                _mm_setr_ps(r.z,r.x,r.y,r.y)));
    r1=_mm_sub_ps(r1,_mm_mul_ps(_mm_setr_ps(l.z,l.x,l.y,l.z),
                                _mm_setr_ps(r.y,r.z,r.x,r.z)));

    return *(pi::SO3f*)&r1;
}

inline pi::SO3d mul(pi::SO3d& l,const pi::SO3d& r)
{
    return l*r;
}
