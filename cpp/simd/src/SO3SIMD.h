#include "SO3.h"
#include <GSLAM/core/SE3.h>
#include <xmmintrin.h>

/**
return SO3( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
            w * rq.y + y * rq.w + z * rq.x - x * rq.z,
            w * rq.z + z * rq.w + x * rq.y - y * rq.x,
            w * rq.w - x * rq.x - y * rq.y - z * rq.z);
**/
inline pi::SO3f mul(const pi::SO3f& l,const pi::SO3f& r)
{
    __m128 r1=_mm_mul_ps(_mm_load1_ps(&l.w),_mm_load_ps((float*)&r));
    r1=_mm_add_ps(r1,_mm_mul_ps(_mm_setr_ps(l.x,l.y,l.z,-l.x),
                                _mm_setr_ps(r.w,r.w,r.w,r.x)));
    r1=_mm_add_ps(r1,_mm_mul_ps(_mm_setr_ps(l.y,l.z,l.x,-l.y),
                                _mm_setr_ps(r.z,r.x,r.y,r.y)));
    r1=_mm_sub_ps(r1,_mm_mul_ps(_mm_setr_ps(l.z,l.x,l.y,l.z),
                                _mm_setr_ps(r.y,r.z,r.x,r.z)));

    return *(pi::SO3f*)&r1;
}

inline pi::SO3d mul(const pi::SO3d& l,const pi::SO3d& r)
{
    return l*r;
}

template <typename T>
inline T sine(T x) {
    T sin = 0;
    //always wrap input angle to -PI..PI
    while (x < -3.14159265)
        x += 6.28318531;
    while (x > 3.14159265)
        x -= 6.28318531;
    //compute sine
    if (x < 0) {
        sin = 1.27323954 * x + .405284735 * x * x;
        if (sin < 0)
            sin = .225 * (sin * -sin - sin) + sin;
        else
            sin = .225 * (sin * sin - sin) + sin;
    } else {
        sin = 1.27323954 * x - 0.405284735 * x * x;
        if (sin < 0)
            sin = .225 * (sin * -sin - sin) + sin;
        else
            sin = .225 * (sin * sin - sin) + sin;
    }
    return sin;
}

template <typename T>
inline T cosine(T x) {
    //compute cosine: sin(x + PI/2) = cos(x)
    return sine(x+1.57079632);
}

inline GSLAM::Point3f log(const pi::SO3f& l)
{
    typedef float Precision;
    const Precision squared_w = l.w*l.w;
    const Precision n = sqrt(l.x*l.x+l.y*l.y+l.z*l.z);

    Precision A_inv;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    if (n < NEAR_ZERO)
    {
        //If n is too small
        A_inv = 2./l.w - 2.*(1.0-squared_w)/(l.w*squared_w);
    }
    else
    {
        if (fabs(l.w)<NEAR_ZERO)
        {
            //If w is too small
            if (l.w>0)
            {
                A_inv = M_PI/n;
            }
            else
            {
                A_inv = -M_PI/n;
            }
        }
        else
            A_inv = 2*atan(n/l.w)/n;
    }
    return GSLAM::Point3_<Precision>(l.x*A_inv,l.y*A_inv,l.z*A_inv);
}

inline GSLAM::Point3d log(const pi::SO3d& l)
{
    return l.ln();
}

template <typename Scalar>
inline pi::SO3<Scalar> exp(const GSLAM::Point3_<Scalar>& l)
{
    Scalar theta_sq = l.dot(l);
    Scalar theta    = sqrt(theta_sq);
    Scalar half_theta = static_cast<Scalar>(0.5) * theta;

    Scalar imag_factor;
    Scalar real_factor;

    if (theta < static_cast<Scalar>(1e-10)) {
      Scalar theta_po4 = theta_sq * theta_sq;
      imag_factor = static_cast<Scalar>(0.5) -
                    static_cast<Scalar>(1.0 / 48.0) * theta_sq +
                    static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
      real_factor = static_cast<Scalar>(1) -
                    static_cast<Scalar>(0.5) * theta_sq +
                    static_cast<Scalar>(1.0 / 384.0) * theta_po4;
    } else {
      Scalar sin_half_theta = sine(half_theta);
      imag_factor = sin_half_theta / theta;
      real_factor = cosine(half_theta);
    }

    return pi::SO3<Scalar>( imag_factor * l.x, imag_factor * l.y,
        imag_factor * l.z,real_factor);
}

template <typename Scalar>
inline pi::SE3<Scalar> exp(const pi::Array_<Scalar,6>& l)
{
    pi::Point3_<Scalar> p(l.data[0],l.data[1],l.data[2]);
    pi::Point3_<Scalar> r(l.data[3],l.data[4],l.data[5]);
    Scalar theta_sq = r.dot(r);
    Scalar theta    = sqrt(theta_sq);
    Scalar half_theta = static_cast<Scalar>(0.5) * theta;

    Scalar imag_factor;
    Scalar real_factor;

    if (theta < static_cast<Scalar>(1e-10)) {
      Scalar theta_po4 = theta_sq * theta_sq;
      imag_factor = static_cast<Scalar>(0.5) -
                    static_cast<Scalar>(1.0 / 48.0) * theta_sq +
                    static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
      real_factor = static_cast<Scalar>(1) -
                    static_cast<Scalar>(0.5) * theta_sq +
                    static_cast<Scalar>(1.0 / 384.0) * theta_po4;
    } else {
      Scalar sin_half_theta = sin(half_theta);
      imag_factor = sin_half_theta / theta;
      real_factor = cos(half_theta);
    }

    pi::SO3<Scalar> R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
    auto t= p+(1-cos(theta))/theta_sq*r.cross(p)+
            (theta-sin(theta))/(theta_sq*theta)*r.cross(r.cross(p));
    return pi::SE3<Scalar>(R,t);
}

template <typename Scalar>
inline pi::SE3<Scalar> expFast(const pi::Array_<Scalar,6>& l)
{
    pi::Point3_<Scalar> p(l.data[0],l.data[1],l.data[2]);
    pi::Point3_<Scalar> r(l.data[3],l.data[4],l.data[5]);
    Scalar theta_sq = r.dot(r);
    Scalar theta    = sqrt(theta_sq);
    Scalar half_theta = static_cast<Scalar>(0.5) * theta;

    Scalar imag_factor;
    Scalar real_factor;

    if (theta < static_cast<Scalar>(1e-10)) {
      Scalar theta_po4 = theta_sq * theta_sq;
      imag_factor = static_cast<Scalar>(0.5) -
                    static_cast<Scalar>(1.0 / 48.0) * theta_sq +
                    static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
      real_factor = static_cast<Scalar>(1) -
                    static_cast<Scalar>(0.5) * theta_sq +
                    static_cast<Scalar>(1.0 / 384.0) * theta_po4;
    } else {
      Scalar sin_half_theta = sine(half_theta);
      imag_factor = sin_half_theta / theta;
      real_factor = cosine(half_theta);
    }

    pi::SO3<Scalar> R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
    auto t= p+(1-cosine(theta))/theta_sq*r.cross(p)+
            (theta-sine(theta))/(theta_sq*theta)*r.cross(r.cross(p));
    return pi::SE3<Scalar>(R,t);
}

template <typename Scalar>
inline pi::Array_<Scalar,6> log(const pi::SE3<Scalar>& T)
{
    pi::Array_<Scalar,6> result;
    const auto& l=T.get_rotation();
    const auto& t=T.get_translation();
    const Scalar squared_w = l.w*l.w;
    const Scalar n = sqrt(l.x*l.x+l.y*l.y+l.z*l.z);

    Scalar A_inv;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    if (n < NEAR_ZERO||true)
    {
        //If n is too small
        A_inv = 2./l.w - 2.*(1.0-squared_w)/(l.w*squared_w);
        GSLAM::Point3_<Scalar> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);
        GSLAM::Point3_<Scalar> p=t-0.5*r.cross(t)+static_cast<Scalar>(1. / 12.)*r.cross(r.cross(t));
        *(GSLAM::Point3_<Scalar>*)&result.data=p;
        *(GSLAM::Point3_<Scalar>*)&result.data[3]=r;
    }
    else
    {
        if (fabs(l.w)<NEAR_ZERO)
        {
            //If w is too small
            if (l.w>0)
            {
                A_inv = M_PI/n;
            }
            else
            {
                A_inv = -M_PI/n;
            }
        }
        else
            A_inv = 2*atan(n/l.w)/n;

        auto theta=A_inv*n;
        GSLAM::Point3_<Scalar> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);
        GSLAM::Point3_<Scalar> a=r/theta;
        GSLAM::Point3_<Scalar> p=t-0.5*r.cross(t)+(1-theta/(2*tan(0.5*theta)))*a.cross(a.cross(t));
        *(GSLAM::Point3_<Scalar>*)&result.data=p;
        *(GSLAM::Point3_<Scalar>*)&result.data[3]=r;
    }
    return result;

}


