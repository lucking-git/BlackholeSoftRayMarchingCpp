#pragma once
#include "vec3.h"
class ray
{
public:
	ray() {}
	ray(const vec3& origin, const vec3& direction)
		:orig(origin), dir(direction)
	{}//��ʼ������  ԭ��+t*����
	vec3 origin() const { return orig; }//���ع���ԭ��
	vec3 direction() const { return dir; }//���ع��߷���

	vec3 at(double t)const  //���ع���
	{
		return orig + t * dir;
	}


public:
	vec3 orig;
	vec3 dir;


};

