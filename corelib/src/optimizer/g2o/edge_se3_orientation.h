/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RTAB_G2O_EDGE_SE3_ORIENTATION_H_
#define RTAB_G2O_EDGE_SE3_ORIENTATION_H_

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include <Eigen/Geometry>

namespace rtabmap {

  /*! \class EdgeSE3OrientationPrior
   * \brief g2o unary edge constraining the full orientation of a node to a target rotation.
   *
   * This is the Manhattan/Atlanta counterpart of EdgeSE3Gravity. Whereas the gravity edge
   * constrains only 2 DOF (roll/pitch, aligning the up vector), this edge constrains all 3
   * rotational DOF toward a measured target orientation (the world grid orientation), while
   * leaving the translation free. The error is the SO(3) log-map of the relative rotation
   * between the measured target and the current estimate. Jacobians are computed numerically
   * (the BaseUnaryEdge default), as is done for EdgeSE3Gravity.
   */
  class EdgeSE3OrientationPrior : public g2o::BaseUnaryEdge<3, Eigen::Matrix3d, g2o::VertexSE3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3OrientationPrior(){
        information().setIdentity();
        _measurement.setIdentity();
    }
    virtual bool read(std::istream& is) {return false;} // not implemented
    virtual bool write(std::ostream& os) const {return false;} // not implemented

    // return the orientation error estimate as a 3-vector (rotation vector)
    void computeError(){
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        const Eigen::Matrix3d & estimate = v1->estimate().linear();
        // Relative rotation between the measured target orientation and the estimate.
        Eigen::AngleAxisd error(_measurement.transpose() * estimate);
        _error = error.angle() * error.axis();
    }

    // The measurement is the target orientation (rotation matrix) in the world frame.
    virtual void setMeasurement(const Eigen::Matrix3d& m){
        _measurement = m;
    }
  };
}
#endif
