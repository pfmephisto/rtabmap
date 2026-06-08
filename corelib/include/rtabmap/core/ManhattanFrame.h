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

#ifndef RTABMAP_MANHATTANFRAME_H_
#define RTABMAP_MANHATTANFRAME_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <Eigen/Core>
#include <vector>

namespace rtabmap {

class SensorData;

/**
 * Detect a per-keyframe Manhattan frame (the dominant orthogonal scene directions) from the
 * depth/point-cloud data and snap the keyframe orientation onto a globally-consistent
 * Atlanta-world grid (a shared vertical axis plus a set of horizontal directions).
 *
 * This is a 3D scene-structure feature, analogous in spirit to a Feature2D detector: it
 * produces a per-node geometric observation. The observation is consumed by the graph
 * optimizer as a Link::kManhattan orientation prior (weighted by Optimizer/ManhattanSigma)
 * to reduce rotational drift, which is the dominant drift source in indoor SLAM.
 *
 * The detector is stateful: it accumulates the global Atlanta model (vertical axis and the
 * registered horizontal directions) across the frames of a session. It is not thread-safe;
 * it is meant to be owned and called from a single SLAM thread (Memory).
 */
class RTABMAP_CORE_EXPORT ManhattanFrame
{
public:
	ManhattanFrame(const ParametersMap & parameters = ParametersMap());

	void parseParameters(const ParametersMap & parameters);

	bool isEnabled() const {return enabled_;}

	// Number of distinct horizontal directions (Manhattan frames) registered so far in the
	// global Atlanta model: 1 for a single Manhattan world, more in Atlanta mode. The
	// registered yaw bases (radians, in [0, pi/2)) are the per-frame orientations.
	std::size_t numFrames() const {return horizontalYaws_.size();}
	const std::vector<float> & frameDirections() const {return horizontalYaws_;}

	/**
	 * Detect the dominant orthogonal directions in a frame from its depth/stereo data.
	 * @param data the sensor data (RGB-D or stereo with valid camera models).
	 * @param frameRotationOut on success, the detected Manhattan frame rotation expressed in
	 *        the sensor base frame of @p data (columns are the orthonormal frame axes).
	 * @param confidenceOut on success, the fraction of normals supporting the detected frame.
	 * @return true if a confident orthogonal frame was found.
	 */
	bool detectFrame(const SensorData & data, Transform & frameRotationOut, float & confidenceOut) const;

	/**
	 * Match a detected frame against the global Atlanta model and compute the snapped target
	 * world orientation for the node, suitable as a Link::kManhattan prior transform (rotation
	 * only, zero translation). Updates the global model (bootstraps the vertical axis and
	 * registers new horizontal directions in Atlanta mode).
	 * @param odomWorldPose the node's current (odometry) world pose.
	 * @param frameRotation the detected frame rotation in the base frame (from detectFrame()).
	 * @param snappedWorldOrientationOut on success, the snapped target world orientation.
	 * @return true if the snap is accepted (false if rejected, e.g. no consistent vertical or
	 *         the required correction exceeds Manhattan/MaxCorrection).
	 */
	bool matchAndSnap(const Transform & odomWorldPose, const Transform & frameRotation, Transform & snappedWorldOrientationOut);

	/**
	 * Reset the accumulated global Atlanta model (e.g. when starting a new session).
	 */
	void reset();

private:
	// Set the in-plane reference axes from the current vertical axis.
	void updateHorizontalBasis();
	// Reduce a yaw to the [0, pi/2) Manhattan cell and match/register against horizontalYaws_.
	// Returns the snapped yaw closest to the observed yaw.
	float snapYaw(float observedYaw);

private:
	// parameters
	bool enabled_;
	bool atlanta_;
	int decimation_;
	float maxDepth_;
	int normalK_;
	float normalRadius_;
	float bandwidth_;          // mean-shift / clustering bandwidth (rad)
	float orthoTolerance_;     // max deviation from 90 deg between the two dominant axes (rad)
	float verticalTolerance_;  // max deviation of the most-vertical axis from the vertical (rad)
	int minInliers_;
	float matchYawTolerance_;  // max yaw difference (mod 90 deg) to match a registered direction (rad)
	float maxCorrection_;      // max accepted orientation correction (rad)

	// global Atlanta model (world frame)
	Eigen::Vector3d verticalAxis_;
	Eigen::Vector3d horizontalRef_;  // e1, in-plane reference
	Eigen::Vector3d horizontalRef2_; // e2 = verticalAxis_ x e1
	std::vector<float> horizontalYaws_; // registered yaw bases in [0, pi/2)
	bool initialized_;
};

} // namespace rtabmap

#endif /* RTABMAP_MANHATTANFRAME_H_ */
