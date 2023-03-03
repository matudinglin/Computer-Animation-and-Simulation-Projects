#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <vector>

Interpolator::Interpolator()
{
	//Set default interpolation type
	m_InterpolationType = LINEAR;

	//set default angle representation to use for interpolation
	m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion* pInputMotion, Motion** pOutputMotion, int N)
{
	//Allocate new motion
	*pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton());

	//Perform the interpolation
	if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
		LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
		LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
		BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
		BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else
	{
		printf("Error: unknown interpolation / angle representation type.\n");
		exit(1);
	}
}

/// <summary>
/// conversion routines
/// </summary>
double Interpolator::Angle2Radian(double angle)
{
	return angle * M_PI / 180.0;
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
	double cy = sqrt(R[0] * R[0] + R[3] * R[3]);

	if (cy > 16 * DBL_EPSILON)
	{
		angles[0] = atan2(R[7], R[8]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = atan2(R[3], R[0]);
	}
	else
	{
		angles[0] = atan2(-R[5], R[4]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = 0;
	}

	for (int i = 0; i < 3; i++)
		angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	double x = Angle2Radian(angles[0]);
	double y = Angle2Radian(angles[1]);
	double z = Angle2Radian(angles[2]);

	R[0] = cos(y) * cos(z);
	R[1] = sin(x) * sin(y) * cos(z) - cos(x) * sin(z);
	R[2] = cos(x) * sin(y) * cos(z) + sin(x) * sin(z);
	R[3] = cos(y) * sin(z);
	R[4] = sin(x) * sin(y) * sin(z) + cos(x) * cos(z);
	R[5] = cos(x) * sin(y) * sin(z) - sin(x) * cos(z);
	R[6] = -sin(y);
	R[7] = sin(x) * cos(y);
	R[8] = cos(x) * cos(y);
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double>& q)
{
	double R[9];
	Euler2Rotation(angles, R);
	q = q.Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double>& q, double angles[3])
{
	double unitAxis[3];
	q.GetRotation(angles, unitAxis);
}

/// <summary>
/// interpolation
/// </summary>
vector Interpolator::Lerp(double t, vector vStart, vector vEnd)
{
	return vStart * (1 - t) + vEnd * t;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double>& qStart, Quaternion<double>& qEnd)
{
	Quaternion<double> result;

	// if dot < 0, move one quaternion to the other side
	double dot = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx() + qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();
	if (dot < 0)
	{
		qStart = -1 * qStart;
		dot = -dot;
	}

	// clamp
	if (dot > 1) dot = 1;
	if (dot < -1) dot = -1;
	double theta = acos(dot);

	// compute
	result = sin((1 - t) * theta) / sin(theta) * qStart + sin(t * theta) / sin(theta) * qEnd;

	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
	Quaternion<double> result;

	result = 2 * (p * q) * q - p;

	return result;
}

/// <summary>
/// interpolation routines
/// </summary>
void Interpolator::LinearInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames();
	
	int q1Keyframe = 0;
	while (q1Keyframe + N + 1 < inputLength)
	{
		// original keyframe
		int q0Keyframe = q1Keyframe - N - 1;
		int q2Keyframe = q1Keyframe + N + 1;
		int q3Keyframe = q2Keyframe + N + 1;

		// original posture
		Posture* q0;
		Posture* q1 = pInputMotion->GetPosture(q1Keyframe);
		Posture* q2 = pInputMotion->GetPosture(q2Keyframe);
		Posture* q3;

		// copy start and end keyframe
		pOutputMotion->SetPosture(q1Keyframe, *q1);
		pOutputMotion->SetPosture(q2Keyframe, *q2);

		// special cases
		if (q1Keyframe == 0)
		{
			q0 = q1;
			q3 = pInputMotion->GetPosture(q3Keyframe);
		}
		else if (q2Keyframe + N + 1 >= inputLength)
		{
			q3 = q2;
			q0 = pInputMotion->GetPosture(q0Keyframe);
		}
		else
		{
			q0 = pInputMotion->GetPosture(q0Keyframe);
			q3 = pInputMotion->GetPosture(q3Keyframe);
		}

		// prepare a1 and b2 for interpolating
		Posture a1, b2;
		{
			vector _a1;
			// compute a1
			_a1 = Lerp(0.5, Lerp(2, q0->root_pos, q1->root_pos), q2->root_pos);
			a1.root_pos = Lerp(1.0 / 3, q1->root_pos, _a1);
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				_a1 = Lerp(0.5, Lerp(2, q0->bone_rotation[bone], q1->bone_rotation[bone]), q2->bone_rotation[bone]);
				a1.bone_rotation[bone] = Lerp(1.0 / 3, q1->bone_rotation[bone], _a1);
			}

			// compute b2
			_a1 = Lerp(0.5, Lerp(2, q1->root_pos, q2->root_pos), q3->root_pos);
			b2.root_pos = Lerp(-1.0 / 3, q2->root_pos, _a1);
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				_a1 = Lerp(0.5, Lerp(2, q1->bone_rotation[bone], q2->bone_rotation[bone]), q3->bone_rotation[bone]);
				b2.bone_rotation[bone] = Lerp(-1.0 / 3, q2->bone_rotation[bone], _a1);
			}
		}

		// interpolate between q1, a1, b2, q2
		for (int frame = 1; frame <= N; ++frame)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = DeCasteljauEuler(t, q1->root_pos, a1.root_pos, b2.root_pos, q2->root_pos);

			// interpolate bone rotation
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, q1->bone_rotation[bone], a1.bone_rotation[bone], b2.bone_rotation[bone], q2->bone_rotation[bone]);
			}

			pOutputMotion->SetPosture(q1Keyframe + frame, interpolatedPosture);
		}

		q1Keyframe = q2Keyframe;
	}
	

	// copy original to remain frames 
	for (int frame = q1Keyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
	// students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
	// students should implement this
}

/// <summary>
/// Bezier spline evaluation
/// </summary>
vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
	vector result;
	vector q0, q1, q2, r0, r1;

	q0 = Lerp(t, p0, p1);
	q1 = Lerp(t, p1, p2);
	q2 = Lerp(t, p2, p3);
	r0 = Lerp(t, q0, q1);
	r1 = Lerp(t, q1, q2);
	result = Lerp(t, r0, r1);

	return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
	Quaternion<double> result;
	Quaternion<double> q0, q1, q2, r0, r1;

	q0 = Slerp(t, p0, p1);
	q1 = Slerp(t, p1, p2);
	q2 = Slerp(t, p2, p3);
	r0 = Slerp(t, q0, q1);
	r1 = Slerp(t, q1, q2);
	result = Slerp(t, r0, r1);

	return result;
}

