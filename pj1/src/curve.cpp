#include "curve.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}


Curve evalBezier(const vector< Vector3f >& P, unsigned steps) 
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

	Curve bezier;
	Matrix4f Matrix_bezier;
	Matrix_bezier.setRow(0, Vector4f(1, -3, 3, -1));
	Matrix_bezier.setRow(1, Vector4f(0, 3, -6, 3));
	Matrix_bezier.setRow(2, Vector4f(0, 0, 3, -3));
	Matrix_bezier.setRow(3, Vector4f(0, 0, 0, 1));

	for (int i = 0; i < (int)P.size() - 1; i += 3) {
		Vector4f p0(P[i], 0), p1(P[i + 1], 0), p2(P[i + 2], 0), p3(P[i + 3], 0);
		Matrix4f G_bez(p0, p1, p2, p3, true);

		for (unsigned j = 0; j <= steps; j++) {
			float t_normalized = static_cast<float>(j) / steps;
			Vector4f t_vec(1, t_normalized, t_normalized * t_normalized, t_normalized * t_normalized * t_normalized);
			Vector4f t_vec_derivative(0, 1, 2 * t_normalized, 3 * t_normalized * t_normalized);

			Vector3f V = (G_bez * Matrix_bezier * t_vec).xyz();
			Vector3f T = (G_bez * Matrix_bezier * t_vec_derivative).xyz();
			T.normalize();

			Vector3f N, B;
			if (bezier.empty()) {
				Vector3f B0(0, 0, 1);
				N = Vector3f::cross(B0, T).normalized();
				B = Vector3f::cross(T, N).normalized();
			} else {
				N = Vector3f::cross(bezier.back().B, T).normalized();
				B = Vector3f::cross(T, N).normalized();
			}

			bezier.push_back(CurvePoint{V, T, N, B});
		}
	}

	if (approx(bezier.front().V, bezier.back().V) && !approx(bezier.front().N, bezier.back().N)) {
		float theta = acos(Vector3f::dot(bezier.front().N, bezier.back().N) / (bezier.front().N.abs() * bezier.back().N.abs()));
		for (int i = 0; i < (int)bezier.size(); i++) {
			Matrix3f R = Matrix3f::rotation(bezier[i].T, -theta * i / (bezier.size() - 1));
			bezier[i].N = R * bezier[i].N;
			bezier[i].B = R * bezier[i].B;
		}
		bezier.back() = bezier.front();
	}
	return bezier;
}

Curve evalBspline(const vector< Vector3f >& P, unsigned steps) 
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

	Curve bspline;
	Matrix4f Matrix_bsp;
	Matrix_bsp.setRow(0, Vector4f(1, -3, 3, -1));
	Matrix_bsp.setRow(1, Vector4f(4, 0, -6, 3));
	Matrix_bsp.setRow(2, Vector4f(1, 3, 3, -3));
	Matrix_bsp.setRow(3, Vector4f(0, 0, 0, 1));
	Matrix_bsp /= 6.0f;

	for (int i = 0; i < (int)P.size() - 3; i++) {
		Vector4f p0(P[i], 0), p1(P[i + 1], 0), p2(P[i + 2], 0), p3(P[i + 3], 0);
		Matrix4f G(p0, p1, p2, p3, true);

		for (unsigned j = 0; j <= steps; j++) {
			float t_normalized = static_cast<float>(j) / steps;
			Vector4f t_vec(1, t_normalized, t_normalized * t_normalized, t_normalized * t_normalized * t_normalized);
			Vector4f t_vec_derivative(0, 1, 2 * t_normalized, 3 * t_normalized * t_normalized);

			Vector3f V = (G * Matrix_bsp * t_vec).xyz();
			Vector3f T = (G * Matrix_bsp * t_vec_derivative).xyz();
			T.normalize();

			Vector3f N, B;
			if (bspline.empty()) {
				Vector3f B0(0, 0, 1);
				N = Vector3f::cross(B0, T).normalized();
				B = Vector3f::cross(T, N).normalized();
			} else {
				N = Vector3f::cross(bspline.back().B, T).normalized();
				B = Vector3f::cross(T, N).normalized();
			}

			bspline.push_back(CurvePoint{V, T, N, B});
		}
	}

	if (approx(bspline.front().V, bspline.back().V) && !approx(bspline.front().N, bspline.back().N)) {
		float theta = acos(Vector3f::dot(bspline.front().N, bspline.back().N) / (bspline.front().N.abs() * bspline.back().N.abs()));
		for (int i = 0; i < (int)bspline.size(); i++) {
			Matrix3f R = Matrix3f::rotation(bspline[i].T, -theta * i / (bspline.size() - 1));
			bspline[i].N = R * bspline[i].N;
			bspline[i].B = R * bspline[i].B;
		}
		bspline.back() = bspline.front();
	}
	return bspline;
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

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
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
		Vector4f MORGN  = T * ORGN;
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