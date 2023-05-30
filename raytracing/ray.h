
#ifndef RAY_H
#define RAY_H
#include "vec3.h"



class ray
{
public:
	ray();
	ray(const vec3& origin, const vec3& direction);//��ʼ������  ԭ��+t*����
	vec3 origin() const;//���ع���ԭ��
	vec3 direction() const;//���ع��߷���

	vec3 at(double t)const; //���ع���



public:
	vec3 orig;
	vec3 dir;


};

#endif

