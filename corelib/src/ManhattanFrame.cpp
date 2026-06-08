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

#include "rtabmap/core/ManhattanFrame.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_surface.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace rtabmap {

namespace {
const double kHalfPi = M_PI / 2.0;

// One direction cluster on the sphere, with antipodal folding handled by the
// sign-invariant scatter matrix (n n^T).
struct DirectionCluster
{
	DirectionCluster(const Eigen::Vector3d & n) :
		axis(n), scatter(n * n.transpose()), count(1) {}
	Eigen::Vector3d axis;
	Eigen::Matrix3d scatter;
	int count;
};
}

ManhattanFrame::ManhattanFrame(const ParametersMap & parameters) :
	enabled_(Parameters::defaultManhattanEnabled()),
	atlanta_(Parameters::defaultManhattanAtlanta()),
	decimation_(Parameters::defaultManhattanDecimation()),
	maxDepth_(Parameters::defaultManhattanMaxDepth()),
	normalK_(Parameters::defaultManhattanNormalK()),
	normalRadius_(Parameters::defaultManhattanNormalRadius()),
	bandwidth_(Parameters::defaultManhattanBandwidth()),
	orthoTolerance_(Parameters::defaultManhattanOrthoTolerance()),
	verticalTolerance_(Parameters::defaultManhattanVerticalTolerance()),
	minInliers_(Parameters::defaultManhattanMinInliers()),
	matchYawTolerance_(Parameters::defaultManhattanMatchYawTolerance()),
	maxCorrection_(Parameters::defaultManhattanMaxCorrection())
{
	reset();
	parseParameters(parameters);
}

void ManhattanFrame::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kManhattanEnabled(), enabled_);
	Parameters::parse(parameters, Parameters::kManhattanAtlanta(), atlanta_);
	Parameters::parse(parameters, Parameters::kManhattanDecimation(), decimation_);
	Parameters::parse(parameters, Parameters::kManhattanMaxDepth(), maxDepth_);
	Parameters::parse(parameters, Parameters::kManhattanNormalK(), normalK_);
	Parameters::parse(parameters, Parameters::kManhattanNormalRadius(), normalRadius_);
	Parameters::parse(parameters, Parameters::kManhattanBandwidth(), bandwidth_);
	Parameters::parse(parameters, Parameters::kManhattanOrthoTolerance(), orthoTolerance_);
	Parameters::parse(parameters, Parameters::kManhattanVerticalTolerance(), verticalTolerance_);
	Parameters::parse(parameters, Parameters::kManhattanMinInliers(), minInliers_);
	Parameters::parse(parameters, Parameters::kManhattanMatchYawTolerance(), matchYawTolerance_);
	Parameters::parse(parameters, Parameters::kManhattanMaxCorrection(), maxCorrection_);

	if(decimation_ < 1)
	{
		decimation_ = 1;
	}
	if(normalK_ <= 0 && normalRadius_ <= 0.0f)
	{
		UWARN("%s and %s are both <= 0, falling back to K=20 for normal computation.",
				Parameters::kManhattanNormalK().c_str(), Parameters::kManhattanNormalRadius().c_str());
		normalK_ = 20;
	}
}

void ManhattanFrame::reset()
{
	verticalAxis_ = Eigen::Vector3d::UnitZ();
	horizontalYaws_.clear();
	initialized_ = false;
	updateHorizontalBasis();
}

void ManhattanFrame::updateHorizontalBasis()
{
	// Pick an in-plane reference orthogonal to the vertical axis (stable choice).
	Eigen::Vector3d ref = Eigen::Vector3d::UnitX();
	if(std::fabs(verticalAxis_.dot(ref)) > 0.9)
	{
		ref = Eigen::Vector3d::UnitY();
	}
	horizontalRef_ = (ref - ref.dot(verticalAxis_) * verticalAxis_).normalized();
	horizontalRef2_ = verticalAxis_.cross(horizontalRef_);
}

bool ManhattanFrame::detectFrame(const SensorData & data, Transform & frameRotationOut, float & confidenceOut) const
{
	// Build a point cloud from the depth/stereo data (in the sensor base frame).
	std::vector<int> validIndices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromSensorData(
			data, decimation_, maxDepth_, 0.0f, &validIndices);
	if(!cloud || (int)validIndices.size() < minInliers_)
	{
		return false;
	}

	pcl::IndicesPtr indices(new std::vector<int>(validIndices));
	pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(
			cloud, indices, normalK_, normalRadius_, Eigen::Vector3f(0, 0, 0));
	if(!normals || normals->size() != cloud->size())
	{
		return false;
	}

	// Collect the valid unit normals (antipodal folding is handled later by the scatter matrix).
	std::vector<Eigen::Vector3d> dirs;
	dirs.reserve(validIndices.size());
	for(size_t i = 0; i < validIndices.size(); ++i)
	{
		const pcl::Normal & pn = normals->at(validIndices[i]);
		if(!std::isfinite(pn.normal_x) || !std::isfinite(pn.normal_y) || !std::isfinite(pn.normal_z))
		{
			continue;
		}
		Eigen::Vector3d n(pn.normal_x, pn.normal_y, pn.normal_z);
		double norm = n.norm();
		if(norm < 1e-6)
		{
			continue;
		}
		dirs.push_back(n / norm);
	}
	if((int)dirs.size() < minInliers_)
	{
		return false;
	}

	// Single-pass leader clustering of the normal directions on the sphere.
	std::vector<DirectionCluster> clusters;
	const double cosBandwidth = std::cos(bandwidth_);
	for(size_t i = 0; i < dirs.size(); ++i)
	{
		const Eigen::Vector3d & n = dirs[i];
		int best = -1;
		double bestAbsDot = cosBandwidth;
		for(size_t c = 0; c < clusters.size(); ++c)
		{
			double absDot = std::fabs(n.dot(clusters[c].axis));
			if(absDot > bestAbsDot)
			{
				bestAbsDot = absDot;
				best = (int)c;
			}
		}
		if(best >= 0)
		{
			clusters[best].scatter += n * n.transpose();
			++clusters[best].count;
		}
		else
		{
			clusters.push_back(DirectionCluster(n));
		}
	}
	if(clusters.size() < 2)
	{
		return false;
	}

	// Refine each cluster axis as the dominant eigenvector of its scatter matrix, and sort by support.
	for(size_t c = 0; c < clusters.size(); ++c)
	{
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(clusters[c].scatter);
		clusters[c].axis = es.eigenvectors().col(2); // largest eigenvalue is last
	}
	std::sort(clusters.begin(), clusters.end(),
			[](const DirectionCluster & a, const DirectionCluster & b){return a.count > b.count;});

	// The dominant direction is the first axis; find the strongest near-orthogonal direction for the second.
	const Eigen::Vector3d a1 = clusters[0].axis;
	const int minSecondCount = std::max(1, clusters[0].count / 10);
	int second = -1;
	double bestOrthoDot = 1.0;
	for(size_t j = 1; j < clusters.size(); ++j)
	{
		if(clusters[j].count < minSecondCount)
		{
			continue;
		}
		double absDot = std::fabs(a1.dot(clusters[j].axis));
		if(absDot < bestOrthoDot)
		{
			bestOrthoDot = absDot;
			second = (int)j;
		}
	}
	if(second < 0 || bestOrthoDot > std::sin(orthoTolerance_))
	{
		// no consistent second axis within tolerance of 90 deg
		return false;
	}

	// Build an orthonormal frame from the two dominant directions.
	const Eigen::Vector3d a2 = (clusters[second].axis - clusters[second].axis.dot(a1) * a1).normalized();
	const Eigen::Vector3d a3 = a1.cross(a2); // unit, gives det(R)=+1 for columns (a1,a2,a3)

	// Count supporting normals (within the bandwidth of any of the +/- axes).
	int inliers = 0;
	for(size_t i = 0; i < dirs.size(); ++i)
	{
		const Eigen::Vector3d & n = dirs[i];
		double d = std::max(std::fabs(n.dot(a1)), std::max(std::fabs(n.dot(a2)), std::fabs(n.dot(a3))));
		if(d > cosBandwidth)
		{
			++inliers;
		}
	}
	if(inliers < minInliers_)
	{
		return false;
	}

	frameRotationOut = Transform(
			(float)a1[0], (float)a2[0], (float)a3[0], 0.0f,
			(float)a1[1], (float)a2[1], (float)a3[1], 0.0f,
			(float)a1[2], (float)a2[2], (float)a3[2], 0.0f);
	confidenceOut = (float)inliers / (float)dirs.size();
	return true;
}

float ManhattanFrame::snapYaw(float observedYaw)
{
	// Reduce to the [0, pi/2) Manhattan cell (the frame has a 90 deg rotational symmetry).
	float base = observedYaw - std::floor(observedYaw / (float)kHalfPi) * (float)kHalfPi;

	int best = -1;
	float bestDiff = matchYawTolerance_;
	for(size_t k = 0; k < horizontalYaws_.size(); ++k)
	{
		float d = std::fabs(base - horizontalYaws_[k]);
		d = std::min(d, (float)kHalfPi - d); // circular distance modulo pi/2
		if(d < bestDiff)
		{
			bestDiff = d;
			best = (int)k;
		}
	}

	float gridBase;
	if(best >= 0)
	{
		gridBase = horizontalYaws_[best];
	}
	else if(atlanta_ || horizontalYaws_.empty())
	{
		// Atlanta mode (or the very first direction): register a new horizontal direction.
		horizontalYaws_.push_back(base);
		gridBase = base;
	}
	else
	{
		// Strict single-grid mode: snap to the established direction even if beyond tolerance.
		gridBase = horizontalYaws_[0];
	}

	// Return the representative (gridBase + m*pi/2) closest to the observed yaw.
	float m = std::round((observedYaw - gridBase) / (float)kHalfPi);
	return gridBase + m * (float)kHalfPi;
}

bool ManhattanFrame::matchAndSnap(const Transform & odomWorldPose, const Transform & frameRotation, Transform & snappedWorldOrientationOut)
{
	if(odomWorldPose.isNull() || frameRotation.isNull())
	{
		return false;
	}

	const Eigen::Matrix3d Rwb = odomWorldPose.toEigen3d().linear();  // world <- base
	const Eigen::Matrix3d Rbm = frameRotation.toEigen3d().linear();  // base  <- manhattan frame
	const Eigen::Matrix3d Rwm = Rwb * Rbm;                           // world <- manhattan frame

	if(!initialized_)
	{
		// Bootstrap the global vertical axis from the most-vertical axis of the first confident
		// frame (the floor/ceiling normal), so the Atlanta grid is anchored to the scene. Only
		// commit if that axis is already close to the world up: a badly tilted first frame would
		// otherwise define a wrong global vertical for the whole session, so we wait for a better
		// frame instead.
		int vidx = 0;
		double bestAbs = -1.0;
		for(int i = 0; i < 3; ++i)
		{
			double a = std::fabs(Rwm.col(i).dot(Eigen::Vector3d::UnitZ()));
			if(a > bestAbs)
			{
				bestAbs = a;
				vidx = i;
			}
		}
		if(bestAbs < std::cos(verticalTolerance_))
		{
			return false;
		}
		Eigen::Vector3d v = Rwm.col(vidx);
		if(v.dot(Eigen::Vector3d::UnitZ()) < 0)
		{
			v = -v;
		}
		verticalAxis_ = v.normalized();
		updateHorizontalBasis();
		initialized_ = true;
	}

	// Identify the frame axis most aligned with the global vertical.
	int vidx = 0;
	double bestAbs = -1.0;
	for(int i = 0; i < 3; ++i)
	{
		double a = std::fabs(Rwm.col(i).dot(verticalAxis_));
		if(a > bestAbs)
		{
			bestAbs = a;
			vidx = i;
		}
	}
	if(bestAbs < std::cos(verticalTolerance_))
	{
		// No frame axis is close enough to the global vertical: not Atlanta-consistent.
		return false;
	}
	const double vsign = Rwm.col(vidx).dot(verticalAxis_) >= 0.0 ? 1.0 : -1.0;
	const Eigen::Vector3d vertCol = vsign * verticalAxis_;

	// Derive the horizontal yaw from another frame axis, projected into the horizontal plane.
	const int h1idx = (vidx + 1) % 3;
	const int h2idx = (vidx + 2) % 3;
	Eigen::Vector3d h1obs = Rwm.col(h1idx);
	h1obs -= h1obs.dot(verticalAxis_) * verticalAxis_;
	if(h1obs.norm() < 1e-6)
	{
		return false;
	}
	h1obs.normalize();
	const float yaw = std::atan2((float)h1obs.dot(horizontalRef2_), (float)h1obs.dot(horizontalRef_));

	// Snap the yaw onto the (registered) Atlanta grid and rebuild the orthonormal horizontal axes.
	const float yawSnap = snapYaw(yaw);
	const Eigen::Vector3d h1col = std::cos(yawSnap) * horizontalRef_ + std::sin(yawSnap) * horizontalRef2_;
	Eigen::Vector3d h2col = vertCol.cross(h1col);

	// Assemble the snapped grid orientation with the same column roles as the observation.
	Eigen::Matrix3d G;
	G.col(vidx) = vertCol;
	G.col(h1idx) = h1col;
	G.col(h2idx) = h2col;
	if(G.determinant() < 0.0)
	{
		G.col(h2idx) = -G.col(h2idx);
	}

	// Reject large corrections: a big disagreement between odometry and the grid usually means
	// a misdetection or an inconsistent vertical, and a wrong prior is worse than none.
	const Eigen::AngleAxisd correction(Rwm.transpose() * G);
	if(correction.angle() > maxCorrection_)
	{
		return false;
	}

	// Target node world orientation such that (target * Rbm) == G  =>  target = G * Rbm^T.
	const Eigen::Matrix3d Rtarget = G * Rbm.transpose();
	snappedWorldOrientationOut = Transform(
			(float)Rtarget(0,0), (float)Rtarget(0,1), (float)Rtarget(0,2), 0.0f,
			(float)Rtarget(1,0), (float)Rtarget(1,1), (float)Rtarget(1,2), 0.0f,
			(float)Rtarget(2,0), (float)Rtarget(2,1), (float)Rtarget(2,2), 0.0f);
	return true;
}

} // namespace rtabmap
