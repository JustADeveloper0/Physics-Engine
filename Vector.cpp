#include "pch.h"
#include "Vector.h"
#include <iostream>

Vector::Vector(float x_, float y_) : x(x_), y(y_) {}

float Vector::GetX() const {
	return x;
}

float Vector::GetY() const {
	return y;
}

void Vector::SetX(float x_) {
	x = x_;
}

void Vector::SetY(float y_) {
	y = y_;
}

bool Vector::Zero() {
	return (x == 0 && y == 0);
}

char* Vector::ToString() const { // const is used here so when returned, the string cannot be changed
	static char output[50]; // static so it doesn't self delete after this method
	snprintf(output, sizeof(output), "(%.2f, %.2f)", x, y); // safe alternative to sprintf where it uses size of to reduce buffer overflow
	return output;
}
