#include "pch.h"
#include "VectorMath.h"
#include <iostream>

Vector VectorMath::Add(const Vector& v1, const Vector& v2) {
	return Vector(v1.GetX() + v2.GetX(), v1.GetY() + v2.GetY());
}

Vector VectorMath::Subtract(const Vector& v1, const Vector& v2) {
	return Vector(v1.GetX() - v2.GetX(), v1.GetY() - v2.GetY());
}

Vector VectorMath::Multiply(const Vector& v, const float scalar) {
	return Vector(v.GetX() * scalar, v.GetY() * scalar);
}

Vector VectorMath::Divide(const Vector& v, const float scalar) {
	return Vector(v.GetX() / scalar, v.GetY() / scalar);
}

Vector VectorMath::Normalize(const Vector& v) {
	float length = Norm(v);
	if (length == 0.0f) return v;
	return Vector(v.GetX() / length, v.GetY() / length);
}

float VectorMath::Norm(const Vector& v) {
	return sqrtf(v.GetX() * v.GetX() + v.GetY() * v.GetY());
}

float VectorMath::Dot(const Vector& v1, const Vector& v2) {
	return v1.GetX() * v2.GetX() + v1.GetY() * v2.GetY();
}

float VectorMath::Cross(const Vector& v1, const Vector& v2) {
	return v1.GetX() * v2.GetY() - v1.GetY() * v2.GetX();
}

float VectorMath::Distance(const Vector& v1, const Vector& v2) {
	return sqrtf(powf(v2.GetX() - v1.GetX(), 2) + powf(v2.GetY() - v1.GetY(), 2));
}

bool VectorMath::Equal(const Vector& v1, const Vector& v2) {
	return (v1.GetX() == v2.GetX()) && (v1.GetY() == v2.GetY());
}

Vector VectorMath::Rotate(const Vector& v, float radians) {
	float cosA = cosf(radians);
	float sinA = sinf(radians);

	float x = v.GetX() * cosA - v.GetY() * sinA;
	float y = v.GetX() * sinA + v.GetY() * cosA;

	return Vector(x, y);
}

float VectorMath::Min(float a, float b) {
	return (a < b) ? a : b;
}

float VectorMath::Max(float a, float b) {
	return (a > b) ? a : b;
}

