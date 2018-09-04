#include "SO3.h"
#include <xmmintrin.h>

/**
return SO3( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
            w * rq.y + y * rq.w + z * rq.x - x * rq.z,
            w * rq.z + z * rq.w + x * rq.y - y * rq.x,
            w * rq.w - x * rq.x - y * rq.y - z * rq.z);
**/
inline pi::SO3f mul(const pi::SO3f& l,const pi::SO3f& r)
{
    __m128 r1=_mm_mul_ps(_mm_load1_ps(&l.w),_mm_load_ps(&r.x));
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

#define PI 3.1415926535

inline double shift(double x) {
    //always wrap input angle to -PI..PI
    while (x < -3.14159265)
        x += 6.28318531;
    while (x > 3.14159265)
        x -= 6.28318531;
    return x;
}

template <typename T>
inline T sine(T x) {
    T sin = 0;
    x = shift( x );
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
    T cos = 0;
    //compute cosine: sin(x + PI/2) = cos(x)
    x += 1.57079632;
    if (x > 3.14159265)
        x -= 6.28318531;
    if (x < 0) {
        cos    = 1.27323954 * x + 0.405284735 * x * x;
        if (cos < 0)
            cos = .225 * (cos *-cos - cos) + cos;
        else
            cos = .225 * (cos * cos - cos) + cos;
    }
    else
    {
        cos = 1.27323954 * x - 0.405284735 * x * x;
        if (cos < 0)
            cos = .225 * (cos *-cos - cos) + cos;
        else
            cos = .225 * (cos * cos - cos) + cos;
    }
    return cos;
}

template <typename T>
T FastArcTan(T x)
{
    return M_PI_4*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
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

