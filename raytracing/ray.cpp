#include "ray.h"
//#include "vec3.h"

ray::ray() {}
ray::ray(const vec3& origin, const vec3& direction,double time=0.0)
	:orig(origin), dir(direction),tm(time){}//��ʼ������  ԭ��+t*����
vec3 ray::origin() const { return orig; }//���ع���ԭ��
vec3 ray::direction() const { return dir; }//���ع��߷���

vec3 ray::at(double t)const  //���ع���
{
	return orig + t * dir;
}
double ray::time()const
{
	return tm;
}
