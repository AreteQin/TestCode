////////////////////////////////////////////////////////////
// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyFrame数据类型
////////////////////////////////////////////////////////////

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sophus/se3.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
// generate random number for simulated IMU data
#include <utility>
#include <vector>
#include <string>
#include <random>
//#include <algorithm>

// 极小量的定义
const float eps = 1e-4;

// Gravity value
const float g = -9.81;

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
        const float d = std::sqrt(d2);
        Eigen::Vector3f v;
        v << x, y, z;
        Eigen::Matrix3f W = Sophus::SO3f::hat(v);
        if (d < eps) {
            return I;
        } else {
            // equation (1.6)
            return I - W * (1.0f - std::cos(d)) / d2 + W * W * (d - std::sin(d)) / (d2 * d);
        }
    }

    Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v) {
        return RightJacobianSO3(v(0), v(1), v(2));
    }

    Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z) {
        Eigen::Matrix3f I;
        I.setIdentity();
        const float d2 = x * x + y * y + z * z;
        const float d = std::sqrt(d2);
        Eigen::Vector3f v;
        v << x, y, z;
        Eigen::Matrix3f W = Sophus::SO3f::hat(v);

        if (d < eps) {
            return I;
        } else {
            // equation (1.7)
            return I + W / 2 + W * W * (1.0f / d2 - (1.0f + std::cos(d)) / (2.0f * d * std::sin(d)));
        }
    }

    Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v) {
        return InverseRightJacobianSO3(v(0), v(1), v(2));
    }

    class point {
    };

    class Bias {
        friend class boost::serialization::access;

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

        void CopyFrom(Bias &b) {
            bax = b.bax;
            bay = b.bay;
            baz = b.baz;
            bwx = b.bwx;
            bwy = b.bwy;
            bwz = b.bwz;
        }

        friend std::ostream &operator<<(std::ostream &out, const Bias &b) {
            if (b.bwx > 0)
                out << " ";
            out << b.bwx << ",";
            if (b.bwy > 0)
                out << " ";
            out << b.bwy << ",";
            if (b.bwz > 0)
                out << " ";
            out << b.bwz << ",";
            if (b.bax > 0)
                out << " ";
            out << b.bax << ",";
            if (b.bay > 0)
                out << " ";
            out << b.bay << ",";
            if (b.baz > 0)
                out << " ";
            out << b.baz;

            return out;
        }

    public:
        float bax, bay, baz;
        float bwx, bwy, bwz;
    };

    class Calib {
        // enable Boost serialization
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

        /**
        * @brief 设置参数
        * @param Tbc 位姿变换
        * @param ng 角速度计噪声
        * @param na 加速度计噪声
        * @param ngw 随机游走
        * @param naw 随机游走
        */
        Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na, const float &ngw, const float &naw) {
            Set(Tbc, ng, na, ngw, naw);
        }

        /**
        * @brief imu标定参数的构造函数
        * @param calib imu标定参数
        */
        Calib(const Calib &calib) {
            mbIsSet = calib.mbIsSet;
            // Sophus/Eigen parameters
            mTbc = calib.mTbc;
            mTcb = calib.mTcb;
            Cov = calib.Cov;
            CovWalk = calib.CovWalk;
        }

        Calib() { mbIsSet = false; }

        //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
        void
        Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw) {
            mbIsSet = true;
            const float ng2 = ng * ng;
            const float na2 = na * na;
            const float ngw2 = ngw * ngw;
            const float naw2 = naw * naw;

            // Sophus/Eigen
            mTbc = sophTbc;
            mTcb = mTbc.inverse();
            // 噪声协方差 TODO
            Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
            // 随机游走协方差 TODO
            CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
        }

    public:
        // Sophus/Eigen implementation
        // Tcb: Transformation from camera to IMU
        Sophus::SE3<float> mTcb;
        Sophus::SE3<float> mTbc;
        // IMU intrinsic parameters: 噪声 和 随机游走 的协方差矩阵
        Eigen::DiagonalMatrix<float, 6> Cov, CovWalk;
        bool mbIsSet;
    };

    //Integration of 1 gyro measurement
    class IntegratedRotation {
    public:
        IntegratedRotation() {}

        // calculate the rotation matrix R_{j-1,j} from one gyro measurement and corresponding right Jacobian
        // equation (5.1) and (5.6.1)
        IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
            const float x = (angVel(0) - imuBias.bwx) * time;
            const float y = (angVel(1) - imuBias.bwy) * time;
            const float z = (angVel(2) - imuBias.bwz) * time;

            const float d2 = x * x + y * y + z * z;
            const float d = std::sqrt(d2);

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
                deltaR = Eigen::Matrix3f::Identity() + W * std::sin(d) / d + W * W * (1.0f - std::cos(d)) / d2;
                // equation (1.6)
                rightJ = Eigen::Matrix3f::Identity() - W * (1.0f - std::cos(d)) / d2 +
                         W * W * (d - std::sin(d)) / (d2 * d);
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
            NgaWalk = calib.CovWalk;
            Initialize(b_);
        }

        // copy constructor
        Preintegrated(Preintegrated *pImuPre) : dT(pImuPre->dT), C(pImuPre->C), Info(pImuPre->Info),
                                                Nga(pImuPre->Nga), NgaWalk(pImuPre->NgaWalk), b(pImuPre->b),
                                                dR(pImuPre->dR), dV(pImuPre->dV),
                                                dP(pImuPre->dP), JRg(pImuPre->JRg), JVg(pImuPre->JVg),
                                                JVa(pImuPre->JVa), JPg(pImuPre->JPg), JPa(pImuPre->JPa),
                                                avgA(pImuPre->avgA), avgW(pImuPre->avgW), bu(pImuPre->bu),
                                                db(pImuPre->db), mvMeasurements(pImuPre->mvMeasurements) {

        }

        Preintegrated() {}

        ~Preintegrated() {}

        void CopyFrom(Preintegrated *pImuPre) {
            dT = pImuPre->dT;
            C = pImuPre->C;
            Info = pImuPre->Info;
            Nga = pImuPre->Nga;
            NgaWalk = pImuPre->NgaWalk;
            b.CopyFrom(pImuPre->b);
            dR = pImuPre->dR;
            dV = pImuPre->dV;
            dP = pImuPre->dP;
            JRg = pImuPre->JRg;
            JVg = pImuPre->JVg;
            JVa = pImuPre->JVa;
            JPg = pImuPre->JPg;
            JPa = pImuPre->JPa;
            avgA = pImuPre->avgA;
            avgW = pImuPre->avgW;
            bu.CopyFrom(pImuPre->bu);
            db = pImuPre->db;
            mvMeasurements = pImuPre->mvMeasurements;
        }

        void Initialize(const Bias &b_) {
            dR.setIdentity();
            dV.setZero();
            dP.setZero();
            JRg.setZero();
            JVg.setZero();
            JVa.setZero();
            JPg.setZero();
            JPa.setZero();
            C.setZero();
            Info.setZero();
            db.setZero();
            b = b_;
            bu = b_;
            avgA.setZero();
            avgW.setZero();
            dT = 0.0f;
            mvMeasurements.clear();
        }

        // 当偏置更新时，对预计分进行更新，以便迭代优化
        void Reintegrate() {
            std::unique_lock<std::mutex> lock(mMutex);
            const std::vector<integrable> aux = mvMeasurements;
        }

        // 累积新的IMU测量值到预计分结果，同时更新噪声
        void
        IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt) {
            mvMeasurements.push_back(integrable(acceleration, angVel, dt));

            // Position is updated firstly, as it depends on previously computed velocity and rotation.
            // Velocity is updated secondly, as it depends on previously computed rotation.
            // Rotation is the last to be updated.

            //Matrices to compute covariance
            Eigen::Matrix<float, 9, 9> A;
            A.setIdentity();
            Eigen::Matrix<float, 9, 6> B;
            B.setZero();

            // 减去偏置后的加速度和角速度
            Eigen::Vector3f acc, accW;
            acc << acceleration(0) - b.bax, acceleration(1) - b.bay, acceleration(2) - b.baz;
            accW << angVel(0) - b.bwx, angVel(1) - b.bwy, angVel(2) - b.bwz;

            // 计算增加新的IMU测量值后的平均加速度和角速度
//            avgA = (dT * avgA + dR * acc * dt) / (dT + dt);
//            avgW = (dT * avgW + accW * dt) / (dT + dt);

            // Update delta position dP and velocity dV (rely on no-updated delta rotation)
            // equation (5.3)
            dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
            // equation (5.2)
            dV = dV + dR * acc * dt;

            // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
            // equation (4.9)
            Eigen::Matrix<float, 3, 3> Wacc = Sophus::SO3f::hat(acc);

            A.block<3, 3>(3, 0) = -dR * dt * Wacc;
            A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * Wacc;
            A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);
            B.block<3, 3>(3, 3) = dR * dt;
            B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;

            // Update position and velocity jacobians wrt bias correction p 和 v 对偏置的雅克比
            // equation (5.13.1), but it is never used
//            JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;
            // equation (5.13)
            JPg = JPg + JVg * dt - 0.5f * dR * dt * dt * Wacc * JRg;
            // equation (5.11)
            JVa = JVa - dR * dt;
            // equation (5.10)
            JVg = JVg - dR * dt * Wacc * JRg;

            // Update delta rotation
            // Calculate R_{j-1,j}
            IntegratedRotation dRi(angVel, b, dt);
            // R_{i,j-1} * R_{j-1,j}
            dR = NormalizeRotation(dR * dRi.deltaR);

            // Compute rotation parts of matrices A and B
            A.block<3, 3>(0, 0) = dRi.deltaR.transpose();
            B.block<3, 3>(0, 0) = dRi.rightJ * dt;

            // Update covariance
            // equation (4.11)， 协方差矩阵 C 的理论上是9*9，但这里将bias的随机游走并入了该协方差矩阵
            C.block<9, 9>(0, 0) = A * C.block<9, 9>(0, 0) * A.transpose() + B * Nga * B.transpose();
            // 这一部分最开始是0矩阵，随着积分次数增加，每次都加上随机游走，偏置的信息矩阵
            C.block<6, 6>(9, 9) += NgaWalk;

            // Update rotation jacobian wrt bias correction
            // equation (5.8)
            JRg = dRi.deltaR.transpose() * JRg - dRi.rightJ * dt;

            // Total integrated time
            dT += dt;
        }

        /**
        * @brief 融合两个预积分，发生在删除关键帧的时候，3帧变2帧，需要把两段预积分融合后重新计算预计分
        * @param pPrev 前面的预积分
        */
        void MergePrevious(Preintegrated *pPrev) {
            if (pPrev == this)
                return;

            std::unique_lock<std::mutex> lock1(mMutex);
            std::unique_lock<std::mutex> lock2(pPrev->mMutex);
            // ？为什么不能直接用bu？ TODO
            Bias bav;
            bav.bwx = bu.bwx;
            bav.bwy = bu.bwy;
            bav.bwz = bu.bwz;
            bav.bax = bu.bax;
            bav.bay = bu.bay;
            bav.baz = bu.baz;

            const std::vector<integrable> aux1 = pPrev->mvMeasurements;
            const std::vector<integrable> aux2 = mvMeasurements;

            Initialize(bav);
            for (size_t i = 0; i < aux1.size(); i++)
                IntegrateNewMeasurement(aux1[i].a, aux1[i].w, aux1[i].t);
            for (size_t i = 0; i < aux2.size(); i++)
                IntegrateNewMeasurement(aux2[i].a, aux2[i].w, aux2[i].t);
        }

        // 更新bias
        void SetNewBias(const Bias &bu_) {
            std::unique_lock<std::mutex> lock(mMutex);
            bu = bu_;

            db(0) = bu_.bwx - b.bwx;
            db(1) = bu_.bwy - b.bwy;
            db(2) = bu_.bwz - b.bwz;
            db(3) = bu_.bax - b.bax;
            db(4) = bu_.bay - b.bay;
            db(5) = bu_.baz - b.baz;
        }

        // Calculate the delta bias between the new one and the last one
        IMU::Bias GetDeltaBias(const Bias &b_) {
            std::unique_lock<std::mutex> lock(mMutex);
            return IMU::Bias(b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz, b_.bwx - b.bwx, b_.bwy - b.bwy,
                             b_.bwz - b.bwz);
        }

        // Calculate the new delta rotation dR while updating the bias
        Eigen::Matrix3f GetDeltaRotation(const Bias &b_) {
            std::unique_lock<std::mutex> lock(mMutex);
            Eigen::Vector3f dbg;
            dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
            // equation (5.7)
            return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
        }

        // Calculate the new delta velocity dV while updating the bias
        Eigen::Vector3f GetDeltaVelocity(const Bias &b_) {
            std::unique_lock<std::mutex> lock(mMutex);
            Eigen::Vector3f dbg, dba;
            dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
            dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
            // equation (5.9)
            return dV + JVg * dbg + JVa * dba;
        }

        // Calculate the new delta position dP while updating the bias
        Eigen::Vector3f GetDeltaPosition(const Bias &b_) {
            std::unique_lock<std::mutex> lock(mMutex);
            Eigen::Vector3f dbg, dba;
            dbg << b_.bwx - b.bwx, b_.bwy - b.bwy, b_.bwz - b.bwz;
            dba << b_.bax - b.bax, b_.bay - b.bay, b_.baz - b.baz;
            // ? no corresponding equation TODO
            return dP + JPg * dbg + JPa * dba;
        }

//        // usage? TODO
//        Eigen::Matrix3f GetUpdatedDeltaRotation() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            // equation (5.7)
//            return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * db.head(3)).matrix());
//        }
//
//        Eigen::Vector3f GetUpdatedDeltaVelocity() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return dV + JVg * db.head(3) + JVa * db.tail(3);
//        }
//
//        Eigen::Vector3f GetUpdatedDeltaPosition() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return dP + JPg * db.head(3) + JPa * db.tail(3);
//        }
//
        Eigen::Matrix3f GetOriginalDeltaRotation() {
            std::unique_lock<std::mutex> lock(mMutex);
            return dR;
        }

        Eigen::Vector3f GetOriginalDeltaVelocity() {
            std::unique_lock<std::mutex> lock(mMutex);
            return dV;
        }

        Eigen::Vector3f GetOriginalDeltaPosition() {
            std::unique_lock<std::mutex> lock(mMutex);
            return dP;
        }
//
//        Bias GetOriginalBias() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return b;
//        }
//
//        Bias GetUpdatedBias() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return bu;
//        }
//
//        Eigen::Matrix<float, 6, 1> GetDeltaBias() {
//            std::unique_lock<std::mutex> lock(mMutex);
//            return db;
//        }
//
//        void printMeasurements() const {
//            std::cout << "pint meas:\n";
//            for (int i = 0; i < mvMeasurements.size(); i++) {
//                std::cout << "meas " << mvMeasurements[i].t << std::endl;
//            }
//            std::cout << "end pint meas:\n";
//        }

    public:
        // the time between the first and last measurement in one preintegration
        float dT;
        // 协方差矩阵
        Eigen::Matrix<float, 15, 15> C;
        // 信息矩阵
        Eigen::Matrix<float, 15, 15> Info;
        // Nga 6*6对角矩阵，3个陀螺仪噪声的平方，3个加速度计噪声的平方
        // 随机游走是已知的IMU内参，且不被更新
        Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk;

        // Values for the original bias (when integration was computed)
        // bias before the update
        Bias b;
        Eigen::Matrix3f dR;
        Eigen::Vector3f dV, dP;
        // Jacobians wrt bias
        Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
        Eigen::Vector3f avgA, avgW;


    private:
        // Updated bias
        Bias bu;
        // Dif between original and updated bias
        // This is used to compute the updated values of the preintegration
        Eigen::Matrix<float, 6, 1> db;

        // Store one IMU measurement
        struct integrable {
            template<class Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::make_array(a.data(), a.size());
                ar & boost::serialization::make_array(w.data(), w.size());
                ar & t;
            }

            integrable() {}

            integrable(Eigen::Vector3f a_, Eigen::Vector3f w_, const float &t_) : a(std::move(a_)), w(std::move(w_)),
                                                                                  t(t_) {}

            Eigen::Vector3f a, w;
            float t;
        };

        std::vector<integrable> mvMeasurements;

        std::mutex mMutex;
    };
}

int main() {

    clock_t start_time, end_time;

    // Guassian noise
    float noise_mean_real = 0;
    float noise_standard_deviation_real = 0.05;
    std::default_random_engine e; //引擎
    std::normal_distribution<double> n(noise_mean_real, noise_standard_deviation_real); //均值, 标准差

    Eigen::Matrix<float, 6, 1> noise;
    Eigen::Vector3f a, a_real, w, w_real;
    std::vector<Eigen::Matrix<float, 6, 1>> all_measurements;

    // IMU初始化的开始条件
    float min_time = 1;
    int min_KF = 10;

    // IMU bias and calibration
    IMU::Bias b(0, 0, 0, 0, 0, 0);
    Sophus::SE3<float> Tbc(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, 0));
    IMU::Calib c(Tbc, 0.04, 0.04, 0.01, 0.01);

    auto *preintegrated = new IMU::Preintegrated(b, c);

    // 重力系到世界系的旋转矩阵
    Eigen::Matrix3f Rwg;
    // 重力方向
    Eigen::Vector3f dirG;
    dirG.setZero();

    // main loop
    int have_imu_num = 0;
    float t = 0;
    end_time = clock();
    while (t < 3) { // 3s
        // Generate noise
        noise << n(e), n(e), n(e), n(e), n(e), n(e);
//        std::cout << "noise: " << std::endl << noise << std::endl;
        // Generate imu measurement
        a_real << sin(t), cos(t), cos(t);
        w_real << sin(t), cos(t), sin(t);
        a << sin(t) + noise[0], cos(t) + noise[1], cos(t) + noise[2];
        w << sin(t) + noise[3], cos(t) + noise[4], sin(t) + noise[5];
//        Eigen::Matrix<float, 6, 1> meas;
//        meas << a, w;
//        std::cout << "imu_measurement: " << std::endl << a << std::endl << w << std::endl;
//        all_measurements.push_back(meas);
        // Get system time
        start_time = clock();
        preintegrated->IntegrateNewMeasurement(a, w, (float) (start_time - end_time) / CLOCKS_PER_SEC);
        std::cout << "dR： " << std::endl << preintegrated->GetOriginalDeltaRotation() << std::endl;
        t = t + (float) (start_time - end_time) / CLOCKS_PER_SEC;
        std::cout << "t: " << t << std::endl;

        have_imu_num++;
        // 初始化时关于速度的预积分定义Ri.t()*(s*Vj - s*Vi - Rwg*g*tij)
        dirG -= preintegrated->GetOriginalDeltaRotation() * preintegrated->GetOriginalDeltaVelocity();
        // 求取实际的速度，位移/时间
        Eigen::Vector3f _vel =
                preintegrated->GetOriginalDeltaPosition() / preintegrated->dT;
        if (have_imu_num < 6) {
            std::cout << "imu初始化失败, 由于带有imu预积分信息的关键帧数量太少" << std::endl;
        } else {
            // dirG = sV1 - sVn + n*Rwg*g*t
            // 归一化，约等于重力在世界坐标系下的方向
            dirG = dirG / dirG.norm();
            // 原本的重力方向
            Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
            // 求重力在世界坐标系下的方向与重力在重力坐标系下的方向的叉乘
            Eigen::Vector3f v = gI.cross(dirG);
            // 求叉乘模长
            const float nv = v.norm();
            // 求转角大小
            const float cosg = gI.dot(dirG);
            const float ang = acos(cosg);
            // v/nv 表示垂直于两个向量的轴  ang 表示转的角度，组成角轴
            Eigen::Vector3f vzg = v * ang / nv;
            // 获得重力坐标系到世界坐标系的旋转矩阵的初值
            Rwg = Sophus::SO3f::exp(vzg).matrix();
            std::cout<< "Rwg: " << std::endl << Rwg << std::endl;
//            mRwg = Rwg.cast<double>();
//            mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
        }

        end_time = clock();
    }

    return 0;
}