/**
 * @file drill_frames.hpp
 * @brief The transform math for the CR20 drill plunge, with no SDK, robot or
 * threading dependency of any kind - Eigen and the standard library only.
 *
 * FRAME CONVENTIONS
 *
 * A Pose is a row-major 4x4 homogeneous transform in a flat 16-array, the
 * layout the xCore SDK uses for both RtSupportedFields::tcpPose_m and
 * CartesianPosition::pos:
 *
 *    idx  0  1  2  3       R00 R01 R02 | tx
 *         4  5  6  7   =   R10 R11 R12 | ty
 *         8  9 10 11       R20 R21 R22 | tz
 *        12 13 14 15         0   0   0 |  1
 *
 * Translation lives at 3, 7 and 11. Column k of the rotation block is
 * {k, k+4, k+8} and is the frame's own axis k expressed in its parent's
 * coordinates. Nothing below indexes that by hand - Eigen::Map does it - but
 * the layout is written out so the mapping can be checked by eye.
 *
 * Every Pose in this test is base <- something. The two that matter:
 *
 *   B  the baseline TCP pose, snapshotted once when the control loop starts
 *      and frozen thereafter. It defines the hole axis. It must be frozen:
 *      a hole is straight, so if the push axis tracked the live TCP instead,
 *      a deflecting wrist would curve the hole.
 *
 *   M  the live TCP pose, read fresh every control cycle.
 */

#ifndef CR20_DRILL_FRAMES_HPP_
#define CR20_DRILL_FRAMES_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace cr20_drill
{

/// Row-major 4x4 homogeneous transform, parent <- child. Matches the xCore
/// SDK's own layout for tcpPose_m and CartesianPosition::pos, so a Pose can
/// be handed to and taken from the SDK without conversion.
using Pose = std::array<double, 16>;

/// Eigen view of a Pose. Row-major so that Map over a Pose's memory is a
/// reinterpretation rather than a transpose.
using PoseMatrix = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/**
 * @brief A rigid offset expressed in the SOURCE frame's own axes.
 *
 * Rotation is an axis-angle vector (unit axis scaled by the angle in
 * radians), deliberately not Euler angles: there is no convention to get
 * wrong, and for the sub-degree deviations this test expects it is also the
 * natural "how far has the tool tilted" number, since its magnitude is the
 * total rotation regardless of which axes it is spread across.
 */
struct ToolDelta
{
    double x_m{};
    double y_m{};
    double z_m{};
    double rx_rad{};
    double ry_rad{};
    double rz_rad{};
};

/// The bit points along the tool frame's +Z. Fixed, not a parameter: the
/// previous test made the axis and its sign tunable constants that then had
/// to be kept in agreement with the stiffness array index and the wrench
/// index by hand, and they drifted apart. A tool whose bit does not point
/// along tool +Z should have its TCP re-taught on the pendant rather than
/// compensated for here.
constexpr int kBitAxisColumn = 2;

namespace detail
{

inline Eigen::Map<const PoseMatrix> view(const Pose& pose)
{
    return Eigen::Map<const PoseMatrix>(pose.data());
}

inline Pose toPose(const PoseMatrix& matrix)
{
    Pose pose{};
    Eigen::Map<PoseMatrix>(pose.data()) = matrix;
    return pose;
}

/// Inverse of a rigid transform, written as R^T / -R^T t rather than a
/// general 4x4 inverse. Exact rather than merely accurate, and it documents
/// that these are rigid transforms and not arbitrary matrices.
inline PoseMatrix rigidInverse(const Eigen::Map<const PoseMatrix>& pose)
{
    const Eigen::Matrix3d rotation = pose.topLeftCorner<3, 3>();
    PoseMatrix inverse = PoseMatrix::Identity();
    inverse.topLeftCorner<3, 3>() = rotation.transpose();
    inverse.topRightCorner<3, 1>() = -rotation.transpose() * pose.topRightCorner<3, 1>();
    return inverse;
}

} // namespace detail

/**
 * @brief Advances a pose along its own bit axis (tool +Z).
 *
 * THE command function. Every pose this test sends to the controller comes
 * from here, so that no code anywhere has the opportunity to write a
 * base-frame translation component by hand. Grepping drill_test.cpp for
 * pos[3], pos[7] or pos[11] should return nothing.
 *
 * Computes baseline * Tz(depth_m), where Tz(d) is the identity with d in the
 * Z translation slot. Multiplying out:
 *
 *     baseline * Tz(d) = [ R_B   t_B + d * (column 2 of R_B) ]
 *                        [  0                 1              ]
 *
 * so the rotation block passes through untouched - the command restates the
 * baseline orientation every cycle, which is what holds the bit on axis
 * rather than letting it follow a deflecting wrist - and the translation is
 * the baseline position plus d along the bit axis, in base coordinates.
 *
 * Right-multiplying is what makes the offset mean the pose's OWN axes. Left-
 * multiplying would translate along the base frame's axes instead, which is
 * the specific failure this test was rewritten to make impossible.
 *
 * At depth_m == 0 the offset is the identity, so the result is the baseline
 * bit for bit. The first pose commanded by the control loop is therefore
 * exactly the pose the arm is already in, structurally rather than by a
 * special case.
 *
 * @param baseline Row-major 4x4, base <- tool.
 * @param depth_m Advance along the tool's own +Z, in metres. Negative
 *   retracts.
 * @return Row-major 4x4, base <- tool, ready for CartesianPosition::pos.
 */
inline Pose advanceAlongBit(const Pose& baseline, double depth_m)
{
    PoseMatrix offset = PoseMatrix::Identity();
    offset(kBitAxisColumn, 3) = depth_m;
    return detail::toPose(detail::view(baseline) * offset);
}

/**
 * @brief The rigid offset between two poses, in the first one's own axes.
 *
 * Computes from^-1 * to and reports it as a ToolDelta. This is the inverse
 * of advanceAlongBit() generalised to six degrees of freedom, and it is how
 * the control loop turns a fresh tcpPose_m reading into the numbers the feed
 * law and the abort checks actually use:
 *
 *   .z_m                  travel along the bit axis - the measured depth
 *   hypot(.x_m, .y_m)     wander perpendicular to the bit
 *   the rotation vector   how far the tool has tilted since the baseline
 *
 * Three safety signals from one call, none of them requiring a base-frame
 * component to be picked out by hand. The old design projected onto a frozen
 * axis vector and so could only ever produce the first of them.
 *
 * @param from Row-major 4x4, base <- frame. The reference, e.g. the baseline.
 * @param to Row-major 4x4, base <- frame, in the same parent frame as `from`.
 * @return The offset in `from`'s own axes; translation metres, rotation
 *   radians as an axis-angle vector.
 */
inline ToolDelta toolDeltaBetween(const Pose& from, const Pose& to)
{
    const PoseMatrix delta = detail::rigidInverse(detail::view(from)) * detail::view(to);

    const Eigen::AngleAxisd angle_axis{Eigen::Matrix3d(delta.topLeftCorner<3, 3>())};
    const Eigen::Vector3d rotation = angle_axis.axis() * angle_axis.angle();

    return ToolDelta{delta(0, 3), delta(1, 3), delta(2, 3), rotation.x(), rotation.y(), rotation.z()};
}

/**
 * @brief Angle between two poses' bit axes, in radians.
 *
 * The command frame (the frozen baseline) and the force-control frame (the
 * live tool frame, per setFcCoor) are not the same frame - they disagree by
 * however much the tool has rotated since the baseline was taken. This is
 * that disagreement, measured rather than assumed.
 *
 * It matters because the feed law reduces everything to one scalar along one
 * axis, and that reduction is only valid while the two frames nearly agree.
 * Below 5 degrees the cosine error on the projection is under 0.4%; above it
 * the single-scalar model has stopped being true and the run should not
 * continue.
 */
inline double bitAxisAngleRad(const Pose& a, const Pose& b)
{
    const Eigen::Vector3d axis_a = detail::view(a).topLeftCorner<3, 3>().col(kBitAxisColumn);
    const Eigen::Vector3d axis_b = detail::view(b).topLeftCorner<3, 3>().col(kBitAxisColumn);

    // Clamped because a dot product of two unit vectors can land a hair
    // outside [-1, 1] in floating point, and std::acos returns NaN there.
    return std::acos(std::clamp(axis_a.normalized().dot(axis_b.normalized()), -1.0, 1.0));
}

/**
 * @brief The pose's bit axis expressed in its parent frame. DISPLAY ONLY.
 *
 * Used by the plan block to tell the operator, in base-frame terms, which
 * way the arm is about to push. It must never be used to build a command -
 * advanceAlongBit() is the only thing allowed to do that - because the
 * moment there are two routes from "the bit axis" to "a commanded pose",
 * they can disagree, and in the previous test they did: the plan block
 * advertised one direction while the loop drove another.
 */
inline std::array<double, 3> bitAxisInParentForDisplay(const Pose& pose)
{
    const Eigen::Vector3d axis = detail::view(pose).topLeftCorner<3, 3>().col(kBitAxisColumn);
    return {axis.x(), axis.y(), axis.z()};
}

/// Translation part of a pose, in its parent frame. Display and logging only,
/// for the same reason as bitAxisInParentForDisplay().
inline std::array<double, 3> translationForDisplay(const Pose& pose)
{
    return {pose[3], pose[7], pose[11]};
}

} // namespace cr20_drill

#endif // CR20_DRILL_FRAMES_HPP_
