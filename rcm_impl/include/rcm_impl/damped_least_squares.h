#pragma once

#include <Eigen/Core>


// damped least squares solutions: http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
// trust region optimization https://en.wikipedia.org/wiki/Trust_region
// trust region https://people.maths.ox.ac.uk/nakatsukasa/preprints/TRSrev2.pdf
// adaptive lambda https://link.springer.com/article/10.1007/s12206-018-0240-7
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
dampedLeastSquares(const MatT &mat, typename MatT::Scalar lambda = typename MatT::Scalar{1e-2}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(mat.cols(), mat.rows());
    dampedSingularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        dampedSingularValuesInv(i, i) = singularValues(i) / (singularValues(i)*singularValues(i) + lambda*lambda);
    }
    return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
};
