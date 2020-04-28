#pragma once#
#include "RayTracingHlslCompat.h"
#include "stdafx.h"
class Primitive
{
private:
	PrimitiveConstantBuffer materialCoeffs;
	AnalyticPrimitive::Enum primitiveType;
	XMFLOAT3 offset;
	XMFLOAT3 size;
public:

	Primitive(AnalyticPrimitive::Enum type, PrimitiveConstantBuffer materials, XMFLOAT3 translation, XMFLOAT3 size);
	 XMFLOAT3 getSize();
	XMFLOAT3 getIndex();
	AnalyticPrimitive::Enum getType();
	PrimitiveConstantBuffer getMaterial();
};


