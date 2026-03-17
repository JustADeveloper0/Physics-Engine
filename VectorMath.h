#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include "Vector.h"

class PHYSICS_API VectorMath {
public:
	static Vector Add(const Vector& v1, const Vector& v2);

	static Vector Subtract(const Vector& v1, const Vector& v2);

	static Vector Multiply(const Vector& v, const float scalar);

	static Vector Divide(const Vector& v, const float scalar);

	static Vector Normalize(const Vector& v);

	static float Norm(const Vector& v);

	static float Dot(const Vector& v1, const Vector& v2);

	static float Cross(const Vector& v1, const Vector& v2);

	static float Distance(const Vector& v1, const Vector& v2);

	static bool Equal(const Vector& v1, const Vector& v2);

	static Vector Rotate(const Vector& v, float radians);

	static float Min(float a, float b);

	static float Max(float a, float b);
};
