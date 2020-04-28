#include "stdafx.h"
#include "Primitive.h"

XMFLOAT3 Primitive::getIndex()
{
	return offset;
}
XMFLOAT3 Primitive::getSize()
{
	return size;
}
Primitive::Primitive(AnalyticPrimitive::Enum type, PrimitiveConstantBuffer materials, XMFLOAT3 translation, XMFLOAT3 size) : primitiveType(type), materialCoeffs(materials), offset(translation), size(size)
{
}
AnalyticPrimitive::Enum Primitive::getType()
{
	return primitiveType;
}
PrimitiveConstantBuffer Primitive::getMaterial()
{
	return materialCoeffs;
}
