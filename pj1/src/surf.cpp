#include "surf.h"
#include "vertexrecorder.h"
#include <iostream>
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{

    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i = 0; i < profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;

        return true;
    }
}

// DEBUG HELPER
Surface quad()
{
    Surface ret;
    ret.VV.push_back(Vector3f(-1, -1, 0));
    ret.VV.push_back(Vector3f(+1, -1, 0));
    ret.VV.push_back(Vector3f(+1, +1, 0));
    ret.VV.push_back(Vector3f(-1, +1, 0));

    ret.VN.push_back(Vector3f(0, 0, 1));
    ret.VN.push_back(Vector3f(0, 0, 1));
    ret.VN.push_back(Vector3f(0, 0, 1));
    ret.VN.push_back(Vector3f(0, 0, 1));

    ret.VF.push_back(Tup3u(0, 1, 2));
    ret.VF.push_back(Tup3u(0, 2, 3));
    return ret;
}

// 将规则排列的点连接为三角形网格
void addGridFaces(Surface &surface, int stripLength)
{
    int vertCount = surface.VV.size();
    int rows = vertCount / stripLength;

    for (int i = 0; i < rows - 1; i++) {
        for (int j = 0; j < stripLength - 1; j++) {
            int curr = i * stripLength + j;
            int next = (i + 1) * stripLength + j;

            surface.VF.push_back(Tup3u(curr, curr + 1, next));
            surface.VF.push_back(Tup3u(next, curr + 1, next + 1));
        }
    }
}

// 构造旋转曲面
Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.
    int n = profile.size();
    for (int i = 0; i < steps; i++)
    {
        float t = 2.0f * c_pi * float(i) / steps;
        Matrix4f rotY = Matrix4f::rotateY(t);

        for (const auto &p : profile) {
            Vector3f V = (rotY * Vector4f(p.V, 1)).xyz();
            Vector3f N = (rotY.getSubmatrix3x3(0, 0) * p.N).normalized();
            surface.VV.push_back(V);
            surface.VN.push_back(-N); // 注意法线方向
        }
    }

    // 补上首行作为最后一行，形成闭环
    for (int j = 0; j < n; j++)
    {
        surface.VV.push_back(surface.VV[j]);
        surface.VN.push_back(surface.VN[j]);
    }

    addGridFaces(surface, n);
    return surface;
}

// 构造广义柱面
Surface makeGenCyl(const Curve &profile, const Curve &sweep)
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.
    const int profSize = profile.size();
    const int sweepSize = sweep.size();

    // 利用 TNB 构造变换矩阵
    auto buildFrame = [](const CurvePoint &cp) -> Matrix4f {
        Matrix4f mat;
        mat.setCol(0, Vector4f(cp.N, 0));
        mat.setCol(1, Vector4f(cp.B, 0));
        mat.setCol(2, Vector4f(cp.T, 0));
        mat.setCol(3, Vector4f(cp.V, 1));
        return mat;
    };

    for (int i = 0; i < sweepSize; ++i) {
        Matrix4f frame = buildFrame(sweep[i]);
        Matrix3f rot = frame.getSubmatrix3x3(0, 0);

        for (int j = 0; j < profSize; ++j) {
            Vector3f transformedV = (frame * Vector4f(profile[j].V, 1)).xyz();
            Vector3f transformedN = -(rot * profile[j].N).normalized();

            surface.VV.push_back(transformedV);
            surface.VN.push_back(transformedN);
        }
    }

    // 若首尾位置重合，闭合一圈防止缝隙
    bool isClosed = (sweep.front().V - sweep.back().V).abs() < 1e-6f;
    if (isClosed) {
        for (int j = 0; j < profSize; ++j) {
            surface.VV.push_back(surface.VV[j]);
            surface.VN.push_back(surface.VN[j]);
        }
    }

    addGridFaces(surface, profSize);
    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder *recorder)
{
    const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i = 0; i < (int)surface.VF.size(); i++)
    {
        recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
        recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
        recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder *recorder, float len)
{
    const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i = 0; i < (int)surface.VV.size(); i++)
    {
        recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
        recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{

    for (int i = 0; i < (int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i = 0; i < (int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;

    for (int i = 0; i < (int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j = 0; j < 3; j++)
        {
            unsigned a = surface.VF[i][j] + 1;
            out << a << "/"
                << "1"
                << "/" << a << " ";
        }
        out << endl;
    }
}