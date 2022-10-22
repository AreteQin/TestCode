#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <mutex>
#include <sophus/se3.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>

// 极小量的定义
const float eps = 1e-4;

namespace IMU {
    // 将不是标准正交的旋转矩阵单位正交化
    Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R) {
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        return svd.matrixU() * svd.matrixV().transpose();
    }

    Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z) {
        Eigen::Matrix3f I;
        I.setIdentity();
        const float d2 = x * x + y * y + z * z;
        const float d = sqrt(d2);
        Eigen::Vector3f v;
        v << x, y, z;
        Eigen::Matrix3f W = Sophus::SO3f::hat(v);
        if (d < eps) {
            return I;
        } else {
            // equation (1.6)
            return I - W * (1.0f - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
        }
    }

    Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v) {
        return RightJacobianSO3(v(0), v(1), v(2));
    }

    Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z) {
        Eigen::Matrix3f I;
        I.setIdentity();
        const float d2 = x * x + y * y + z * z;
        const float d = sqrt(d2);
        Eigen::Vector3f v;
        v << x, y, z;
        Eigen::Matrix3f W = Sophus::SO3f::hat(v);

        if (d < eps) {
            return I;
        } else {
            return I + W / 2 + W * W * (1.0f / d2 - (1.0f + cos(d)) / (2.0f * d * sin(d)));
        }
    }

    Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v) {
        return InverseRightJacobianSO3(v(0), v(1), v(2));
    }

    class point {
    };

    class Bias {
//        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & bax;
            ar & bay;
            ar & baz;

            ar & bwx;
            ar & bwy;
            ar & bwz;
        }

    public:
        Bias() : bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0) {}

        Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
             const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z) :
                bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z) {}

        void CopyFrom(Bias &b);

        friend std::ostream &operator<<(std::ostream &out, const Bias &b);

    public:
        float bax, bay, baz;
        float bwx, bwy, bwz;
    };

    class Calib {
        // ？
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            serializeSophusSE3(ar, mTcb, version);
            serializeSophusSE3(ar, mTbc, version);

            ar & boost::serialization::make_array(Cov.diagonal().data(), Cov.diagonal().size());
            ar & boost::serialization::make_array(CovWalk.diagonal().data(), CovWalk.diagonal().size());

            ar & mbIsSet;
        }

    public:

        Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na, const float &ngw, const float &naw) {
            Set(Tbc, ng, na, ngw, naw);
        }

        Calib(const Calib &calib);

        Calib() { mbIsSet = false; }

        //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
        void
        Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw);

    public:
        // Sophus/Eigen implementation
        // Tcb: Transformation from camera to IMU
        Sophus::SE3<float> mTcb;
        Sophus::SE3<float> mTbc;
        // IMU intrinsic parameters: noise and 随机游走
        Eigen::DiagonalMatrix<float, 6> Cov, CovWalk;
        bool mbIsSet;
    };

    //Integration of 1 gyro measurement
    class IntegratedRotation {
    public:
        IntegratedRotation() {}

        // calculate the rotation matrix from one gyro measurement and corresponding right Jacobian
        // equation (5.1) and (5.6.1)
        IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
            const float x = (angVel(0) - imuBias.bwx) * time;
            const float y = (angVel(1) - imuBias.bwy) * time;
            const float z = (angVel(2) - imuBias.bwz) * time;

            const float d2 = x * x + y * y + z * z;
            const float d = sqrt(d2);

            Eigen::Vector3f v;
            v << x, y, z;
            Eigen::Matrix3f W = Sophus::SO3f::hat(v);
            if (d < eps) {
                // equation (1.3)
                deltaR = Eigen::Matrix3f::Identity() + W;
                // equation (1.8)
                rightJ = Eigen::Matrix3f::Identity();
            } else {
                // using Rodrigues' formula
                deltaR = Eigen::Matrix3f::Identity() + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2;
                // equation (1.6)
                rightJ = Eigen::Matrix3f::Identity() - W * (1.0f - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
            }
        }

    public:
        float deltaT; //integration time
        Eigen::Matrix3f deltaR;
        Eigen::Matrix3f rightJ; // right jacobian
    };

    //Preintegration of Imu Measurements
    class Preintegrated {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & dT;
            ar & boost::serialization::make_array(C.data(), C.size());
            ar & boost::serialization::make_array(Info.data(), Info.size());
            ar & boost::serialization::make_array(Nga.diagonal().data(), Nga.diagonal().size());
            ar & boost::serialization::make_array(NgaWalk.diagonal().data(), NgaWalk.diagonal().size());
            ar & b;
            ar & boost::serialization::make_array(dR.data(), dR.size());
            ar & boost::serialization::make_array(dV.data(), dV.size());
            ar & boost::serialization::make_array(dP.data(), dP.size());
            ar & boost::serialization::make_array(JRg.data(), JRg.size());
            ar & boost::serialization::make_array(JVg.data(), JVg.size());
            ar & boost::serialization::make_array(JVa.data(), JVa.size());
            ar & boost::serialization::make_array(JPg.data(), JPg.size());
            ar & boost::serialization::make_array(JPa.data(), JPa.size());
            ar & boost::serialization::make_array(avgA.data(), avgA.size());
            ar & boost::serialization::make_array(avgW.data(), avgW.size());

            ar & bu;
            ar & boost::serialization::make_array(db.data(), db.size());
            ar & mvMeasurements;
        }

    public:
        Preintegrated(const Bias &b_, const Calib &calib) {
            Nga = calib.Cov;
        }

        Preintegrated(Preintegrated *pImuPre);

        Preintegrated() {}

        ~Preintegrated() {}

        void CopyFrom(Preintegrated *pImuPre);

        void Initialize(const Bias &b_);

        void
        IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt);

        void Reintegrate();

        void MergePrevious(Preintegrated *pPrev);

        void SetNewBias(const Bias &bu_);

        IMU::Bias GetDeltaBias(const Bias &b_);

        Eigen::Matrix3f GetDeltaRotation(const Bias &b_);

        Eigen::Vector3f GetDeltaVelocity(const Bias &b_);

        Eigen::Vector3f GetDeltaPosition(const Bias &b_);

        Eigen::Matrix3f GetUpdatedDeltaRotation();

        Eigen::Vector3f GetUpdatedDeltaVelocity();

        Eigen::Vector3f GetUpdatedDeltaPosition();

        Eigen::Matrix3f GetOriginalDeltaRotation();

        Eigen::Vector3f GetOriginalDeltaVelocity();

        Eigen::Vector3f GetOriginalDeltaPosition();

        Eigen::Matrix<float, 6, 1> GetDeltaBias();

        Bias GetOriginalBias();

        Bias GetUpdatedBias();

        void printMeasurements() const {
            std::cout << "pint meas:\n";
            for (int i = 0; i < mvMeasurements.size(); i++) {
                std::cout << "meas " << mvMeasurements[i].t << std::endl;
            }
            std::cout << "end pint meas:\n";
        }

    public:
        // the time between the first and last measurement in one preintegration
        float dT;
        // 协方差矩阵
        Eigen::Matrix<float, 15, 15> C;
        // 信息矩阵
        Eigen::Matrix<float, 15, 15> Info;
        Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk;

        // Values for the original bias (when integration was computed)
        // bias before the update
        Bias b;
        Eigen::Matrix3f dR;
        Eigen::Vector3f dV, dP;
        Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
        Eigen::Vector3f avgA, avgW;


    private:
        // Updated bias
        Bias bu;
        // Dif between original and updated bias
        // This is used to compute the updated values of the preintegration
        Eigen::Matrix<float, 6, 1> db;

        struct integrable {
            template<class Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::make_array(a.data(), a.size());
                ar & boost::serialization::make_array(w.data(), w.size());
                ar & t;
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            integrable() {}

            integrable(const Eigen::Vector3f &a_, const Eigen::Vector3f &w_, const float &t_) : a(a_), w(w_), t(t_) {}

            Eigen::Vector3f a, w;
            float t;
        };

        std::vector<integrable> mvMeasurements;

        std::mutex mMutex;
    };
}

int main(){
    std::cout << "Hello, World!" << std::endl;
    return 0;
}