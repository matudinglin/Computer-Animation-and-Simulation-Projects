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
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
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
	if (dot < 0.0)
	{
		qStart = -1 * qStart;
		dot = -dot;
	}

	// get angle
	double angle = acos(dot);

	//const double EPSILON = 2.22507e-308;
	//if (sin(angle) - EPSILON < 0.0) return qStart * (1.0 - t) + qEnd * t;

	// compute
	double factor1 = sin((1 - t) * angle) / sin(angle);
	double factor2 = sin(t * angle) / sin(angle);
	result = factor1 * qStart + factor2 * qEnd;

	if(isnan(result.Gets())) return qStart * (1.0 - t) + qEnd * t;

	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
	return Slerp(2.0, p, q);
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
	
	int p1Keyframe = 0;
	while (p1Keyframe + N + 1 < inputLength)
	{
		// original keyframe
		int p0Keyframe = p1Keyframe - N - 1;
		int p2Keyframe = p1Keyframe + N + 1;
		int p3Keyframe = p2Keyframe + N + 1;

		// original posture
		Posture* p0;
		Posture* p1 = pInputMotion->GetPosture(p1Keyframe);
		Posture* p2 = pInputMotion->GetPosture(p2Keyframe);
		Posture* p3;

		// copy start and end keyframe
		pOutputMotion->SetPosture(p1Keyframe, *p1);
		pOutputMotion->SetPosture(p2Keyframe, *p2);

		// special cases
		if (p1Keyframe == 0)
		{
			p0 = p1;
			p3 = pInputMotion->GetPosture(p3Keyframe);
		}
		else if (p2Keyframe + N + 1 >= inputLength)
		{
			p3 = p2;
			p0 = pInputMotion->GetPosture(p0Keyframe);
		}
		else
		{
			p0 = pInputMotion->GetPosture(p0Keyframe);
			p3 = pInputMotion->GetPosture(p3Keyframe);
		}

		// prepare a1 and b2 for interpolating
		Posture a1, b2;
		{
			vector _pos;
			vector _rot;
			// compute a1
			_pos = Lerp(0.5, Lerp(2.0, p0->root_pos, p1->root_pos), p2->root_pos);
			a1.root_pos = Lerp(1.0 / 3.0, p1->root_pos, _pos);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				_rot = Lerp(0.5, Lerp(2.0, p0->bone_rotation[bone], p1->bone_rotation[bone]), p2->bone_rotation[bone]);
				a1.bone_rotation[bone] = Lerp(1.0 / 3.0, p1->bone_rotation[bone], _rot);
			}

			// compute b2
			_pos = Lerp(0.5, Lerp(2.0, p1->root_pos, p2->root_pos), p3->root_pos);
			b2.root_pos = Lerp(1.0 / 3, p2->root_pos, _pos);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				_rot = Lerp(0.5, Lerp(2.0, p1->bone_rotation[bone], p2->bone_rotation[bone]), p3->bone_rotation[bone]);
				b2.bone_rotation[bone] = Lerp(1.0 / 3.0, p2->bone_rotation[bone], _rot);
			}
		}

		// interpolate between p1, a1, b2, p2
		for (int frame = 1; frame <= N; ++frame)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1->root_pos, a1.root_pos, b2.root_pos, p2->root_pos);

			// interpolate bone rotation
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1->bone_rotation[bone], a1.bone_rotation[bone], b2.bone_rotation[bone], p2->bone_rotation[bone]);
			}

			pOutputMotion->SetPosture(p1Keyframe + frame, interpolatedPosture);
		}

		p1Keyframe = p2Keyframe;
	}
	

	// copy original to remain frames 
	for (int frame = p1Keyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames();

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
			{
				double interpolatedAngle[3], startAngle[3], endAngle[3];
				Quaternion<double> interpolatedQuaternion, startQuaternion, endQuaternion;

				startPosture->bone_rotation[bone].getValue(startAngle);
				endPosture->bone_rotation[bone].getValue(endAngle);

				// convert euler to quaternion
				Euler2Quaternion(startAngle, startQuaternion);
				Euler2Quaternion(endAngle, endQuaternion);

				// interpolating
				interpolatedQuaternion = Slerp(t, startQuaternion, endQuaternion);

				// convert quaternion back to euler
				Quaternion2Euler(interpolatedQuaternion, interpolatedAngle);

				interpolatedPosture.bone_rotation[bone] = interpolatedAngle;
			}
				
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames();

	int p1Keyframe = 0;
	while (p1Keyframe + N + 1 < inputLength)
	{
		// original keyframe
		int p0Keyframe = p1Keyframe - N - 1;
		int p2Keyframe = p1Keyframe + N + 1;
		int p3Keyframe = p2Keyframe + N + 1;

		// original posture
		Posture* p0;
		Posture* p1 = pInputMotion->GetPosture(p1Keyframe);
		Posture* p2 = pInputMotion->GetPosture(p2Keyframe);
		Posture* p3;

		// copy start and end keyframe
		pOutputMotion->SetPosture(p1Keyframe, *p1);
		pOutputMotion->SetPosture(p2Keyframe, *p2);

		// special cases
		if (p1Keyframe == 0)
		{
			p0 = p1;
			p3 = pInputMotion->GetPosture(p3Keyframe);
		}
		else if (p2Keyframe + N + 1 >= inputLength)
		{
			p3 = p2;
			p0 = pInputMotion->GetPosture(p0Keyframe);
		}
		else
		{
			p0 = pInputMotion->GetPosture(p0Keyframe);
			p3 = pInputMotion->GetPosture(p3Keyframe);
		}

		// prepare quaternion rotation of p1, p2, p3
		Quaternion<double> p0Quaternion[MAX_BONES_IN_ASF_FILE], p1Quaternion[MAX_BONES_IN_ASF_FILE], p2Quaternion[MAX_BONES_IN_ASF_FILE], p3Quaternion[MAX_BONES_IN_ASF_FILE];
		{
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				Quaternion<double> p0Quater, p1Quater, p2Quater, p3Quater;

				Euler2Quaternion(p0->bone_rotation[bone].p, p0Quater);
				Euler2Quaternion(p1->bone_rotation[bone].p, p1Quater);
				Euler2Quaternion(p2->bone_rotation[bone].p, p2Quater);
				Euler2Quaternion(p3->bone_rotation[bone].p, p3Quater);

				p0Quaternion[bone] = p0Quater;
				p1Quaternion[bone] = p1Quater;
				p2Quaternion[bone] = p2Quater;
				p3Quaternion[bone] = p3Quater;
			}
		}

		// prepare posture a1, b2 & quaternion rotation of a1, b2
		Posture a1, b2;
		Quaternion<double> a1Quaternion[MAX_BONES_IN_ASF_FILE], b2Quaternion[MAX_BONES_IN_ASF_FILE];
		{
			vector _pos;
			Quaternion<double> _rot;
			// compute a1
			_pos = Lerp(0.5, Lerp(2.0, p0->root_pos, p1->root_pos), p2->root_pos);
			a1.root_pos = Lerp(1.0 / 3.0, p1->root_pos, _pos);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				_rot = Slerp(0.5, Double(p0Quaternion[bone], p1Quaternion[bone]), p2Quaternion[bone]);
				a1Quaternion[bone] = Slerp(1.0 / 3.0, p1Quaternion[bone], _rot);
			}

			// compute b2
			_pos = Lerp(0.5, Lerp(2.0, p1->root_pos, p2->root_pos), p3->root_pos);
			b2.root_pos = Lerp(1.0 / 3, p2->root_pos, _pos);

			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				_rot = Double(p0Quaternion[bone], p1Quaternion[bone]);
				b2Quaternion[bone] = Slerp(1.0 / 3.0, p2Quaternion[bone], _rot);
			}
		}
		// interpolate between p1, a1, b2, p2
		for (int frame = 1; frame <= N; ++frame)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1->root_pos, a1.root_pos, b2.root_pos, p2->root_pos);

			// interpolate bone rotation
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
			{
				// interpolating quaternion
				Quaternion<double> interpolatedQuaternion = DeCasteljauQuaternion(t, p1Quaternion[bone], a1Quaternion[bone], b2Quaternion[bone], p2Quaternion[bone]);

				// convert quaternion back to euler
				double interpolatedAngle[3];
				Quaternion2Euler(interpolatedQuaternion, interpolatedAngle);

				// set rotation
				interpolatedPosture.bone_rotation[bone] = interpolatedAngle;
			}

			pOutputMotion->SetPosture(p1Keyframe + frame, interpolatedPosture);
		}

		p1Keyframe = p2Keyframe;
	}


	// copy original to remain frames 
	for (int frame = p1Keyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

/// <summary>
/// Bezier spline evaluation
/// </summary>
vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
	vector q0, q1, q2, r0, r1;

	q0 = Lerp(t, p0, p1);
	q1 = Lerp(t, p1, p2);
	q2 = Lerp(t, p2, p3);
	r0 = Lerp(t, q0, q1);
	r1 = Lerp(t, q1, q2);

	vector result;
	result = Lerp(t, r0, r1);
	return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
	Quaternion<double> q0, q1, q2, r0, r1;

	q0 = Slerp(t, p0, p1);
	q1 = Slerp(t, p1, p2);
	q2 = Slerp(t, p2, p3);
	r0 = Slerp(t, q0, q1);
	r1 = Slerp(t, q1, q2);

	Quaternion<double> result;
	result = Slerp(t, r0, r1);
	return result;
}

