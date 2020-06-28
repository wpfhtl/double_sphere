

#include <math.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <eigen3/Eigen/Dense>

class DoubleSphereCamera {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        static const int N = 6;  ///< Number of intrinsic parameters.

        typedef Eigen::Matrix<double, 2, 1> Vec2;
        typedef Eigen::Matrix<double, 4, 1> Vec4;
        typedef Eigen::Matrix<double, N, 1> VecN;
        
        VecN param;

        ~DoubleSphereCamera() {
        };

        /// @brief Construct camera model with given vector of intrinsics
        ///
        /// @param[in] p vector of intrinsic parameters [fx, fy, cx, cy, xi, alpha]
        DoubleSphereCamera(const VecN& p) {
            param = p;
        }

        inline bool project(const Vec4& p3d, Vec2& proj) const {
            const double& fx = param[0];
            const double& fy = param[1];
            const double& cx = param[2];
            const double& cy = param[3];

            const double& xi = param[4];
            const double& alpha = param[5];

            const double& x = p3d[0];
            const double& y = p3d[1];
            const double& z = p3d[2];

            const double xx = x * x;
            const double yy = y * y;
            const double zz = z * z;

            const double r2 = xx + yy;

            const double d1_2 = r2 + zz;
            const double d1 = sqrt(d1_2);

            const double w1 = alpha > double(0.5) ? (double(1) - alpha) / alpha
                                                : alpha / (double(1) - alpha);
            const double w2 =
                (w1 + xi) / sqrt(double(2) * w1 * xi + xi * xi + double(1));
            if (z <= -w2 * d1) return false;

            const double k = xi * d1 + z;
            const double kk = k * k;

            const double d2_2 = r2 + kk;
            const double d2 = sqrt(d2_2);

            const double norm = alpha * d2 + (double(1) - alpha) * k;

            const double mx = x / norm;
            const double my = y / norm;

            proj[0] = fx * mx + cx;
            proj[1] = fy * my + cy;

            return true;
        }

        /// @param[in] proj point to unproject
        /// @param[out] p3d result of unprojection
        /// @param[out] d_p3d_d_proj if not nullptr computed Jacobian of unprojection
        /// with respect to proj
        /// @param[out] d_p3d_d_param point if not nullptr computed Jacobian of
        /// unprojection with respect to intrinsic parameters
        /// @return if unprojection is valid
        inline bool unproject(const Vec2& proj, Vec4& p3d) const {
            const double& fx = param[0];
            const double& fy = param[1];
            const double& cx = param[2];
            const double& cy = param[3];

            const double& xi = param[4];
            const double& alpha = param[5];

            const double mx = (proj[0] - cx) / fx;
            const double my = (proj[1] - cy) / fy;

            const double r2 = mx * mx + my * my;

            if (alpha > double(0.5)) {
                if (r2 >= double(1) / (double(2) * alpha - double(1))) 
                    return false;
            }

            const double xi2_2 = alpha * alpha;
            const double xi1_2 = xi * xi;

            const double sqrt2 = sqrt(double(1) - (double(2) * alpha - double(1)) * r2);

            const double norm2 = alpha * sqrt2 + double(1) - alpha;

            const double mz = (double(1) - xi2_2 * r2) / norm2;
            const double mz2 = mz * mz;

            const double norm1 = mz2 + r2;
            const double sqrt1 = sqrt(mz2 + (double(1) - xi1_2) * r2);
            const double k = (mz * xi + sqrt1) / norm1;

            p3d[0] = k * mx;
            p3d[1] = k * my;
            p3d[2] = k * mz - xi;
            p3d[3] = double(0);
            return true;
        }

        const VecN& getParam() const { 
            return param;
        }

};