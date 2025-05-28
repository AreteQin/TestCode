// Estimate similarity (scale, rotation, translation) between two sets of 3D points
// using G2O and least-squares optimization.

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

// G2O headers
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/base_unary_edge.h>

using namespace g2o;

// Helper: read "x,y,z" CSV into vector of Eigen::Vector3d
static bool readCSV3D(const std::string& filename, std::vector<Eigen::Vector3d>& out)
{
    std::ifstream in(filename);
    if (!in)
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return false;
    }
    std::string line;
    std::getline(in, line); // skip header
    while (std::getline(in, line))
    {
        std::stringstream ss(line);
        double x, y, z;
        char comma;
        ss >> x >> comma >> y >> comma >> z;
        out.emplace_back(x, y, z);
    }
    return true;
}

// Custom unary edge: transforms a point via Sim3 and compares to measurement
class EdgeSim3Point : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSim3Point(const Eigen::Vector3d& pA) : pA_(pA)
    {
    }

    void computeError() override
    {
        const VertexSim3Expmap* v = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        Eigen::Vector3d p_est = v->estimate().map(pA_);
        _error = p_est - _measurement;
    }

    void linearizeOplus() override
    {
    }

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }

private:
    Eigen::Vector3d pA_;
};

int main(int argc, char** argv)
{
    // 1) Load corresponding points
    std::vector<Eigen::Vector3d> ptsA, ptsB;
    if (!readCSV3D("points_frame_a.csv", ptsA) || !readCSV3D("points_frame_b.csv", ptsB))
    {
        return 1;
    }
    if (ptsA.size() != ptsB.size() || ptsA.empty())
    {
        std::cerr << "Point sets must have same non-zero size." << std::endl;
        return 1;
    }

    // 2) Set up optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // shorthand for the 7‐by‐3 block solver
    using BlockSolver_7_3 = g2o::BlockSolver<g2o::BlockSolverTraits<7, 3>>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolver_7_3::PoseMatrixType>;

    // create a *real* linear solver, not a null unique_ptr
    auto linearSolver = std::make_unique<LinearSolverType>();

    // wrap it in the block‐solver
    auto blockSolver = std::make_unique<BlockSolver_7_3>(std::move(linearSolver));

    // give that to Levenberg
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer.setAlgorithm(algorithm);

    // 3) Add Sim3 vertex
    VertexSim3Expmap* vSim3 = new VertexSim3Expmap();
    vSim3->setId(0);
    Sim3 initialSim3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), 1.0);
    vSim3->setEstimate(initialSim3);
    vSim3->setFixed(false);
    optimizer.addVertex(vSim3);

    // 4) Add edges
    for (size_t i = 0; i < ptsA.size(); ++i)
    {
        auto* rawEdge = new EdgeSim3Point(ptsA[i]);
        rawEdge->setVertex(0, vSim3);
        rawEdge->setMeasurement(ptsB[i]);
        rawEdge->setInformation(Eigen::Matrix3d::Identity());
        rawEdge->setId(static_cast<int>(i + 1)); // start IDs from 1 for edges
        optimizer.addEdge(rawEdge);
    }

    // 5) Optimize
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // 6) Retrieve results
    Sim3 result = vSim3->estimate();
    std::cout << "Estimated scale: " << result.scale() << std::endl;
    std::cout << "Estimated rotation matrix:\n" << result.rotation().toRotationMatrix() << std::endl;
    std::cout << "Estimated translation: " << result.translation().transpose() << std::endl;

    return 0;
}
