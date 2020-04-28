#pragma once
#include "Primitive.h"
class Quadric :
	public Primitive
{
	XMFLOAT4X4 quadricCoeffs;

	Quadric(XMFLOAT4X4 coeffs, AnalyticPrimitive::Enum type, PrimitiveConstantBuffer materials, XMFLOAT3 translation, XMFLOAT3 size) : Primitive(type, materials, translation, size), quadricCoeffs(coeffs) {

	}

};

