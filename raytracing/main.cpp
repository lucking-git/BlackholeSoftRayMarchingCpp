
#include "stb_image.h"
#include "hittable_list.h"
#include "sphere.h"
#include "camera.h"
#include <iostream>
#include "material.h"
#include "constant_medium.h"
#include "box.h"
#include "bvh.h"
const int image_width = 800;
const int image_height = 400;
const int samples_per_pixel = 100;//每个像素采样100次
const int max_depth = 50;
const auto aspect_ratio = double(image_width) / image_height;
vec3 lookfrom(13, 2, 3);
vec3 lookat(0, 0, 0);
vec3 vup(0, 1, 0);
auto dist_to_focus = 10.0;
auto aperture = 0.0;
//vec3 lookfrom(278, 278, -800);
//vec3 lookat(278, 278, 0);
//vec3 vup(0, 1, 0);
//auto dist_to_focus = 10.0;
//auto aperture = 0.0;
auto vfov = 40.0;

vec3 blackhole_pos = vec3(0, 0, 0);
double radius = 1.0;

camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0);
int nx, ny, nn;
unsigned char* data = stbi_load("space.jpg", &nx, &ny, &nn, 0);
sphere skyball(lookfrom, 1000);
hit_record  rec;
perlin fbm;

vec3 background(const double u, const double v)
{

    if (data == nullptr)
        return vec3(0, 1, 1);

    auto i = static_cast<int>((u)*nx);
    auto j = static_cast<int>((1 - v) * ny - 0.001);

    if (i < 0) i = 0;
    if (j < 0) j = 0;
    if (i > nx - 1) i = nx - 1;
    if (j > ny - 1) j = ny - 1;

    auto r = static_cast<int>(data[3 * i + 3 * nx * j + 0]) / 255.0;
    auto g = static_cast<int>(data[3 * i + 3 * nx * j + 1]) / 255.0;
    auto b = static_cast<int>(data[3 * i + 3 * nx * j + 2]) / 255.0;

    return vec3(r, g, b);
}

double hitlength(vec3 p)
{
    vec3 length = p - blackhole_pos;
    return length.length() - radius;
}

bool ringHit(vec3 p)
{
    vec3 ringCenter = blackhole_pos;//环坐标  内径  外径  厚度
    double rin = 4.0;
    double rout = 8.0;
    double h = 0.4;

    if (p.y() >= ringCenter.y() - h / 2 && p.y() <= ringCenter.y() + h / 2)
    {
        double hit = hitlength(p);
        if (hit >= (rin-radius) && hit <= (rout-radius))
            return true;
    }
    return false;

}

vec3 gravitationalLensing(vec3 p,double h2)
{
    vec3 r = p- blackhole_pos;
    double rvalue = r.length();
    double r5 = pow(rvalue, 5);
    return -1.5 * h2 * r / r5;

}


vec3 ray_marching(const ray& r,double h2)
{
    

    vec3 v = unit_vector(r.direction());
    vec3 ori = r.origin();

   
    const int maxStep = 300;
    double dt = 0.2;

    vec3 p = ori + v * dt;
    vec3 color = vec3(0, 0, 0);

    double cloudDensity = 0.0;
    double DensityScale = 0.005;
    
    for (int i = 0; i < maxStep; ++i) 
    {
        

        double hitt = hitlength(p);
        if (ringHit(p))
            cloudDensity += fbm.turb(p, 7) * DensityScale;
        color += vec3(1, 0.5, 0.4) * cloudDensity;
        if (hitt < 0.01)
            return color;
         p +=  v * dt;
        

        vec3 a = gravitationalLensing(p,h2);
        v = v + a * dt;
       

    }
    ray newr = ray(p, v, 0);
    skyball.hit(newr, 0.001, infinity, rec);//这一步映射uv坐标
     
    

    return background(rec.u, rec.v)+color;


}


int main()
{
    
    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";


    for (int j = image_height - 1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            vec3 color(0, 0, 0);
            for (int s = 0; s < samples_per_pixel; ++s)
            {
                auto u = (i + random_double()) / image_width;
                auto v = (j + random_double()) / image_height;
                ray r=cam.get_ray(u,v);
                vec3 h = cross(r.origin(), unit_vector(r.direction()));
                double h2 = dot(h, h);
               
                color += ray_marching(r,h2);
            }
            color.write_color(std::cout,samples_per_pixel);
            
        }
       // std::cout << j << std::endl;
    }

    std::cerr << "\nDone.\n";



	return 0;
}