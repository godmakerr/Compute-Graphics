#include "curve.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
	// Approximately equal to.  We don't want to use == because of
	// precision issues with floating point.
	inline bool approx(const Vector3f &lhs, const Vector3f &rhs)
	{
		const float eps = 1e-8f;
		return (lhs - rhs).absSquared() < eps;
	}

}
inline float clamp(float x, float low, float high)
{
	return std::max(low, std::min(x, high));
}

Curve evalBezier(const vector<Vector3f> &P, unsigned steps)
{
	// Check
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.

	// cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	// cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	// for (int i = 0; i < (int)P.size(); ++i)
	// {
	// 	cerr << "\t>>> " << P[i] << endl;
	// }

	// cerr << "\t>>> Steps (type steps): " << steps << endl;
	// cerr << "\t>>> Returning empty curve." << endl;

	int control_group = P.size() / 3;
	Curve Bez(control_group * steps);
	for (int i = 0; i < control_group; i++)
	{

		for (unsigned j = 0; j < steps; ++j) {
			float t = static_cast<float>(j) / steps;
			// 提取控制点
			Vector3f p0 = P[3 * i];
			Vector3f p1 = P[3 * i + 1];
			Vector3f p2 = P[3 * i + 2];
			Vector3f p3 = P[3 * i + 3];
		
			// De Casteljau
			Vector3f a = (1 - t) * p0 + t * p1;
			Vector3f b = (1 - t) * p1 + t * p2;
			Vector3f c = (1 - t) * p2 + t * p3;
		
			Vector3f d = (1 - t) * a + t * b;
			Vector3f e = (1 - t) * b + t * c;
		
			Vector3f V = (1 - t) * d + t * e;
			Vector3f T = (e - d).normalized();
		
			Vector3f N;
			if (i == 0 && j == 0)
				N = Vector3f::cross(Vector3f(0, 0, 1), T).normalized();
			else
				N = Vector3f::cross(Bez[steps * i + j - 1].B, T).normalized();
		
			Vector3f B = Vector3f::cross(T, N).normalized();
		
			Bez[steps * i + j] = CurvePoint{V, T, N, B};
		}
	}

	// Right now this will just return this empty curve.
	return Bez;
}

Curve evalBspline(const vector<Vector3f> &P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier function.

	// cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

	// cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	// for (int i = 0; i < (int)P.size(); ++i)
	// {
	// 	cerr << "\t>>> " << P[i] << endl;
	// }

	// cerr << "\t>>> Steps (type steps): " << steps << endl;
	// cerr << "\t>>> Returning empty curve." << endl;
	int segmentCount = P.size() - 3;
	Curve result;
	result.reserve(segmentCount * steps);

	for (int i = 0; i < segmentCount; ++i)
	{
		const Vector3f &p0 = P[i];
		const Vector3f &p1 = P[i + 1];
		const Vector3f &p2 = P[i + 2];
		const Vector3f &p3 = P[i + 3];

		for (unsigned j = 0; j < steps; ++j)
		{
			float t = static_cast<float>(j) / steps;
			float t2 = t * t;
			float t3 = t2 * t;

			// basis functions
			float b0 = (1 - t) * (1 - t) * (1 - t) / 6.0f;
			float b1 = (3 * t3 - 6 * t2 + 4) / 6.0f;
			float b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0f;
			float b3 = t3 / 6.0f;

			Vector3f point = b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3;

			// derivative of basis functions
			float db0 = -0.5f * (1 - t) * (1 - t);
			float db1 = (9 * t2 - 12 * t) / 6.0f;
			float db2 = (-9 * t2 + 6 * t + 3) / 6.0f;
			float db3 = 0.5f * t2;

			Vector3f tangent = db0 * p0 + db1 * p1 + db2 * p2 + db3 * p3;
			tangent.normalize();

			Vector3f normal, binormal;
			if (result.empty())
			{
				Vector3f arbitrary(0, 0, 1);
				if (fabs(Vector3f::dot(arbitrary, tangent)) > 0.99f)
					arbitrary = Vector3f(1, 0, 0);
				normal = Vector3f::cross(arbitrary, tangent).normalized();
			}
			else
			{
				normal = Vector3f::cross(result.back().B, tangent).normalized();
			}
			binormal = Vector3f::cross(tangent, normal).normalized();

			result.emplace_back(CurvePoint{point, tangent, normal, binormal});
		}
	}

	// optional: loop closing
	if (P[segmentCount] == P[0] &&
		P[segmentCount + 1] == P[1] &&
		P[segmentCount + 2] == P[2])
	{
		result.push_back(result.front());
	}

	return result;
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve &curve, VertexRecorder *recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve &curve, VertexRecorder *recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);

	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));

		// Transform orthogonal frames into model space
		Vector4f MORGN = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}
