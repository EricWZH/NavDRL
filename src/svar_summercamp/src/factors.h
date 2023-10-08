#pragma once
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "GSLAM/core/GSLAM.h"
#include "GSLAM/core/Camera.h"

namespace ceres{

template <class T>
struct Point3
{
    Point3(const T* const pt){
        memcpy(&x,pt,sizeof(x)*3);
    }

    Point3():x(0),y(0),z(0){}

    Point3(const T& x_,const T& y_,const T& z_)
        : x(x_),y(y_),z(z_){}

    inline T& operator[](int index)const
    {
        return ((T*)this)[index];
    }

    inline T norm()const
    {
        return sqrt(x*x+y*y+z*z);
    }

    inline T dot(const Point3& a)const
    {
        return x*a.x+y*a.y+z*a.z;
    }

    inline Point3<T> cross(const Point3& a)const
    {
        return Point3<T>(y*a.z-z*a.y,z*a.x-x*a.z,x*a.y-y*a.x);
    }

    inline Point3<T> normalize()const
    {
        return (*this)*(1./norm());
    }

    friend Point3 operator + (const Point3& a,const Point3& b)
    {
        return Point3(a.x+b.x,a.y+b.y,a.z+b.z);
    }

    friend Point3 operator - (const Point3& a,const Point3& b)
    {
        return Point3(a.x-b.x,a.y-b.y,a.z-b.z);
    }

    friend Point3 operator -(const Point3& a)
    {
        return Point3(-a.x,-a.y,-a.z);
    }

    friend T operator * (const Point3& a,const Point3& b)
    {
        return (a.x*b.x+a.y*b.y+a.z*b.z);
    }

    friend Point3<T> operator ^ (const Point3& a,const Point3& b)
    {
        return Point3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);
    }

    friend Point3 operator * (const T& a,const Point3& b)
    {
        return Point3(a*b.x,a*b.y,a*b.z);
    }

    friend Point3 operator * (const Point3& b,const T& a)
    {
        return Point3(a*b.x,a*b.y,a*b.z);
    }

    friend Point3 operator / (const Point3& a,const T& b)
    {
        return (1./b)*a;
    }

    friend inline bool operator < (const Point3& a,const Point3 b)
    {
        return a.x<b.x;
    }

    T x,y,z;
};

template <typename T>
class SO3{
public:
    SO3(const T& rx=T(0),const T& ry=T(0),const T& rz=T(0),const T& rw=T(1))
        : x(rx),y(ry),z(rz),w(rw){}

    SO3(const T s[4]) // x,y,z,w
    {
        memcpy(&x,s,sizeof(T)*4);
    }

    SO3 inverse()const{
        return SO3(-x,-y,-z,w);
    }

    SO3 operator*(const SO3& rq)const{
        return SO3( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
                    w * rq.y + y * rq.w + z * rq.x - x * rq.z,
                    w * rq.z + z * rq.w + x * rq.y - y * rq.x,
                    w * rq.w - x * rq.x - y * rq.y - z * rq.z);
    }

    Point3<T> operator *(const Point3<T>& p)const{
        Point3<T> me(x,y,z);
        Point3<T> uv = me.cross(p);
        uv = uv + uv;
        return p + w * uv + me.cross(uv);
    }

    Point3<T> log()const
    {
        const T squared_w = w*w;
        const T n = sqrt(x*x+y*y+z*z);

        T A_inv;
        // Atan-based log thanks to
        //
        // C. Hertzberg et al.:
        // "Integrating Generic Sensor Fusion Algorithms with Sound State
        // Representation through Encapsulation of Manifolds"
        // Information Fusion, 2011

        if (n < NEAR_ZERO)
        {
            //If n is too small
            A_inv = 2./w - 2.*(1.0-squared_w)/(w*squared_w);
        }
        else
        {
            if (abs(w)<NEAR_ZERO)
            {
                //If w is too small
                if (w>0.)
                {
                    A_inv = M_PI/n;
                }
                else
                {
                    A_inv = -M_PI/n;
                }
            }
            else
                A_inv = 2.*atan(n/w)/n;
        }
        return Point3<T>(x*A_inv,y*A_inv,z*A_inv);
    }

    T x,y,z,w;
};

template <typename T>
class SE3{
public:
    SE3(){}

    SE3(const Point3<T>& t,const SO3<T>& r)
        :t(t),r(r){}

    SE3(const T* st,const T* sr){
        memcpy(&t,st,sizeof(t));
        memcpy(&r,sr,sizeof(r));
    }

    SE3 inverse()const{
        SE3 ret;
        ret.r = r.inverse();
        ret.t = - (ret.r * t);
        return ret;
    }

    SE3 operator*(const SE3& rh)const{
        return SE3(r * rh.t + t, r * rh.r);
    }

    Point3<T> operator *(const Point3<T>& p)const{
        return t + r * p;
    }

    Point3<T>       t;
    SO3<T>          r;
};

struct KB4ProjectionXYRError{
    /// ux,uy,ux1 in z=1, baseline in meters
    KB4ProjectionXYRError(GSLAM::Point3d pt, GSLAM::SE3 body2cam, std::vector<double>&  camera, GSLAM::Point2d uv)
        : pt(pt), body2cam(body2cam), camera(camera), uv(uv){}

    template <typename T>
    bool operator()( const T* const body_xyr, T* residuals) const
    {
        auto& body2cam_T = body2cam.get_translation();
        auto& body2cam_R = body2cam.get_rotation();
        T body2cam_t[3] = {T(body2cam_T.x), T(body2cam_T.y), T(body2cam_T.z)};
        T body2cam_r[4] = {T(body2cam_R.x), T(body2cam_R.y), T(body2cam_R.z), T(body2cam_R.w)};
        SE3<T> T_body2cam(body2cam_t, body2cam_r);

        Point3<T> p_w ((T)pt.x - body_xyr[0], (T)pt.y - body_xyr[1], (T)pt.z);
        Point3<T> p_body;

        T angle_axis[3] = {T(0), T(0), - body_xyr[2]};
        ceres::AngleAxisRotatePoint(angle_axis, &p_w.x, &p_body.x);

        Point3<T> p = T_body2cam * p_body;

        T u, v;

        const T fx = (T)camera[2];
        const T fy = (T)camera[3];
        const T cx = (T)camera[4];
        const T cy = (T)camera[5];
        const T k1 = (T)camera[6];
        const T k2 = (T)camera[7];
        const T k3 = (T)camera[8];
        const T k4 = (T)camera[9];

        T x = p.x;
        T y = p.y;
        T z = p.z;

        T r2 = x * x + y * y;

        const T r = sqrt(r2);

        if (r > 1e-5) {

          T theta = atan2(r, z);
          T theta2 = theta * theta;

          T r_theta = k4 * theta2;
          r_theta += k3;
          r_theta *= theta2;
          r_theta += k2;
          r_theta *= theta2;
          r_theta += k1;
          r_theta *= theta2;
          r_theta += 1;
          r_theta *= theta;

          T mx = x * r_theta / r;
          T my = y * r_theta / r;

          u = fx * mx + cx;
          v = fy * my + cy;

        } else {
          u = fx * x / z + cx;
          v = fy * y / z + cy;
        }

        residuals[0] = u - uv.x;
        residuals[1] = v - uv.y;

        return true;
    }

    static ceres::CostFunction* Create(GSLAM::Point3d pt, GSLAM::SE3 body2cam, std::vector<double>&  camera, GSLAM::Point2d uv)
    {
        return (new ceres::AutoDiffCostFunction<
                KB4ProjectionXYRError, 2, 3>(
                    new KB4ProjectionXYRError(pt, body2cam, camera, uv)));
    }

    GSLAM::Point3d pt;
    GSLAM::SE3 body2cam;
    std::vector<double>&  camera;
    GSLAM::Point2d uv;
};

struct PointToLineDistanceFactor
{
    PointToLineDistanceFactor(double scaling_factor_,
                              GSLAM::Point3d curr_point_,
                              GSLAM::Point3d closed_point_a_,
                              GSLAM::Point3d closed_point_b_)
        : scaling_factor(scaling_factor_),
          curr_point(curr_point_),
          closed_point_a(closed_point_a_),
          closed_point_b(closed_point_b_) {}

    template <typename T>
    bool operator()(const T *pose_q, const T *pose_t, T *residual) const
    {
        SE3<T> T_cw(pose_t, pose_q);
        Point3<T> cur((T)curr_point.x, (T)curr_point.y, (T)curr_point.z);
        Point3<T> lpa((T)closed_point_a.x, (T)closed_point_a.y, (T)closed_point_a.z);
        Point3<T> lpb((T)closed_point_b.x, (T)closed_point_b.y, (T)closed_point_b.z);

        Point3<T> point_w = T_cw * cur;

        Point3<T> nu = (point_w - lpa).cross(point_w - lpb); // 向量OA 叉乘 向量OB
        T de = (lpa - lpb).norm();                // 向量AB
        T nu_norm = nu.norm();

        if(abs(nu_norm) < 1e-9)
            nu_norm = nu.x + nu.y + nu.z;

        residual[0] = nu_norm / de;
        residual[0] = scaling_factor * residual[0];

        return true;
    }

    static ceres::CostFunction *Create(double scaling_factor_,
                                       GSLAM::Point3d curr_point_,
                                       GSLAM::Point3d closed_point_a_,
                                       GSLAM::Point3d closed_point_b_)
    {
        return (new ceres::AutoDiffCostFunction<PointToLineDistanceFactor, 1, 4, 3>(
                    new PointToLineDistanceFactor(scaling_factor_,curr_point_, closed_point_a_,
                                                  closed_point_b_)));
    }

    double  scaling_factor;
    GSLAM::Point3d curr_point, closed_point_a, closed_point_b;
};

struct PointDistanceFactor
{
  PointDistanceFactor(GSLAM::Point3d curr_point_,
                      GSLAM::Point3d closed_point_)
      : curr_point(curr_point_), closed_point(closed_point_) {}

  template <typename T>
  bool operator()(const T *pose_q, const T *pose_t, T *residual) const {
    SE3<T> T_cw(pose_t, pose_q);
    Point3<T> cur((T)curr_point.x, (T)curr_point.y, (T)curr_point.z);
    Point3<T> lpa((T)closed_point.x, (T)closed_point.y, (T)closed_point.z);
    Point3<T> point_w = T_cw * cur;

    residual[0] = (point_w - lpa).norm();

    return true;
  }

  static ceres::CostFunction *Create(const GSLAM::Point3d curr_point_,
                                     const GSLAM::Point3d closed_point_) {
    //
    return (new ceres::AutoDiffCostFunction<PointDistanceFactor, 1, 4, 3>(
        new PointDistanceFactor(curr_point_, closed_point_)));
  }

  GSLAM::Point3d curr_point;
  GSLAM::Point3d closed_point;
};

struct PosePriorFactor{
    PosePriorFactor(GSLAM::SE3 prior, std::vector<double> weights)
        : prior(prior), weights(weights){}

    // z of tag center should be 0, and the direction of surface is up: 0,0,1
    template <typename T>
    bool operator()(const T* const ref_t, const T* const ref_q, // marker parameters
                    T* residuals) const // residuals 4
    {
        residuals[0] = (ref_t[0] - prior.get_translation().x) * weights[0];
        residuals[1] = (ref_t[1] - prior.get_translation().y) * weights[1];
        residuals[2] = (ref_t[2] - prior.get_translation().z) * weights[2];

        auto& r = prior.get_rotation();

        SO3<T> R(ref_q);
        SO3<T> prior_R(T(r.x),T(r.y),T(r.z),T(r.w));
        SO3<T> error_R = R.inverse()*prior_R;

        Point3<T> e = error_R.log();
        residuals[3] = e.x * weights[3];
        residuals[4] = e.y * weights[4];
        residuals[5] = e.z * weights[5];

        return true;
    }

    static ceres::CostFunction* Create(GSLAM::SE3 prior, std::vector<double> weights)
    {
        return (new ceres::AutoDiffCostFunction<
                PosePriorFactor, 6, 3, 4>(
                    new PosePriorFactor(prior, weights)));
    }

    GSLAM::SE3 prior;
    std::vector<double> weights;
};

}
