#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
{
}

void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;

    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    srand(time(NULL));
    // 确定采样倍数k：如果启用了滤波(filter)，则使用3x3超采样用于抗锯齿；否则使用1x1标准采样
    int k = 1;
    if (_args.filter)
        k = 3;
    // 如果使用超采样，则实际渲染的图像尺寸会扩大k倍
    w *= k;
    h *= k;
    Image kimage(w, h);   // 超采样颜色图
    Image knimage(w, h);  // 超采样法线图
    Image kdimage(w, h);  // 超采样深度图

    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.
    Camera* cam = _scene.getCamera();
    for (int y = 0; y < h; ++y) {
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            // 随机数生成，用于抖动采样(jitter sampling)
            double random_x = (double)rand() / RAND_MAX;
            double random_y = (double)rand() / RAND_MAX;
            if (!_args.jitter)
                random_x = random_y = 0.0;
            random_x = ndcx + 2 * (random_x / (w - 1.0f));
            random_y = ndcy + 2 * (random_y / (h - 1.0f));
            // 生成从相机到当前像素的光线
            Ray r = cam->generateRay(Vector2f(random_x, random_y));
                

            Hit h;
            Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, h);

            kimage.setPixel(x, y, color);
            knimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                kdimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }
    // 超采样抗锯齿处理
    if (k == 3)  
    {
        // 遍历最终输出图像的每个像素
        for (int y = 0; y < h / k; ++y)
            for (int x = 0; x < w / k; ++x)
            {
                {
                    // 计算超采样图像中对应的中心像素位置
                    int center_x = x * k + 1;
                    int center_y = y * k + 1;
                    
                    // 初始化累积颜色、法线和深度
                    Vector3f color(0, 0, 0);
                    Vector3f normal(0, 0, 0);
                    Vector3f depth(0, 0, 0);
                    
                    // 使用加权滤波器处理3x3区域内的像素
                    // 实现了一个类似高斯滤波的加权平均
                    for (int i = -1; i <= 1; i++)
                        for (int j = -1; j <= 1; j++)
                        {
                            // 对角线上的像素权重为1/16
                            if (abs(i) + abs(j) == 2)  // 对角线位置
                            {
                                color += kimage.getPixel(center_x + i, center_y + j) / 16;
                                normal += knimage.getPixel(center_x + i, center_y + j) / 16;
                                depth += kdimage.getPixel(center_x + i, center_y + j) / 16;
                            }
                            // 十字方向上的像素权重为1/8
                            else if (abs(i) + abs(j) == 1)  // 上下左右位置
                            {
                                color += kimage.getPixel(center_x + i, center_y + j) / 8;
                                normal += knimage.getPixel(center_x + i, center_y + j) / 8;
                                depth += kdimage.getPixel(center_x + i, center_y + j) / 8;
                            }
                            // 中心像素权重为1/4
                            else  // 中心位置
                            {
                                color += kimage.getPixel(center_x + i, center_y + j) / 4;
                                normal += knimage.getPixel(center_x + i, center_y + j) / 4;
                                depth += kdimage.getPixel(center_x + i, center_y + j) / 4;
                            }
                        }
                    
                    // 将滤波后的结果写入最终输出图像
                    image.setPixel(x, y, color);
                    nimage.setPixel(x, y, normal.normalized());
                    dimage.setPixel(x, y, depth);
                }
            }
    }
    else if (k == 1)  // 如果没有使用超采样，直接使用原始图像
    {
        image = kimage;
        dimage = kdimage;
        nimage = knimage;
    }
    // END SOLN

    // save the files 
    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}



Vector3f
Renderer::traceRay(const Ray &r,
    float tmin,
    int bounces,
    Hit &h) const
{
    // The starter code only implements basic drawing of sphere primitives.
    // You will implement phong shading, recursive ray tracing, and shadow rays.

    // TODO: IMPLEMENT 
    if (_scene.getGroup()->intersect(r, tmin, h)) {
        Vector3f color(0, 0, 0);
        // 环境光照贡献：环境光强度 * 物体漫反射颜色
        color = _scene.getAmbientLight() * h.getMaterial()->getDiffuseColor();
        // 遍历所有光源
        for (int i = 0; i < _scene.lights.size(); i++)
        {
            
            Vector3f dirToLight; // 从交点指向光源的单位向量
            Vector3f lightIntensity; // 光源强度
            float distToLight; // 交点到光源的距离
            _scene.lights[i]->getIllumination(r.pointAtParameter(h.getT()), dirToLight, lightIntensity, distToLight);

            if (_args.shadows)
            {
                // 创建阴影射线：从交点沿着指向光源的方向
                // 起点稍微偏移以避免自相交问题
                Vector3f shadowRayOrigin = r.pointAtParameter(h.getT()) + 0.01 * dirToLight;
                Ray shadowRay(shadowRayOrigin, dirToLight);
                Hit shadowHit = Hit();

                bool is_shadowIntersected = shadowHit.getT() < std::numeric_limits<float>::max();
                // 计算阴影射线与物体的相交距离
                float distToIntersection = (shadowRay.pointAtParameter(shadowHit.getT()) - shadowRayOrigin).abs();
                
                // 如果阴影射线在到达光源前与物体相交，则该光源被遮挡，跳过该光源的贡献
                if (is_shadowIntersected && distToIntersection < distToLight)
                    continue;
            }
            // 计算当前光源对物体的着色贡献（漫反射和镜面反射）
            color += h.getMaterial()->shade(r, h, dirToLight, lightIntensity);
        }

        // 如果还有反射次数，递归调用traceRay
        if (bounces > 0)
        {
            Vector3f V = r.getDirection();
            Vector3f N = h.getNormal().normalized();
            Vector3f R = (V - (2 * Vector3f::dot(V, N) * N)).normalized();
            Hit reflectHit = Hit();
            
            // 创建反射光线，起点稍微偏移以避免自相交
            Ray reflectRay(r.pointAtParameter(h.getT()) + 0.01 * R, R);
            color += (h.getMaterial()->getSpecularColor()) * traceRay(reflectRay, 0.0f, bounces - 1, reflectHit);
        }

        return color;
    } else {
        return _scene.getBackgroundColor(r.getDirection());
    };
}

