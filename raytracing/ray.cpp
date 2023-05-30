#include "ray.h"
//#include "vec3.h"

ray::ray() {}
ray::ray(const vec3& origin, const vec3& direction)
	:orig(origin), dir(direction) {}//��ʼ������  ԭ��+t*����
vec3 ray::origin() const { return orig; }//���ع���ԭ��
vec3 ray::direction() const { return dir; }//���ع��߷���

vec3 ray::at(double t)const  //���ع���
{
	return orig + t * dir;
}