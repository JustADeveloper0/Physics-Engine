#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

class PHYSICS_API Vector {
private:
	float x, y;

public:
	Vector(float x_ = 0, float y_ = 0);

	float GetX() const; // has to make a copy to dereference and not change any of the original values
	float GetY() const;

	void SetX(float x_);
	void SetY(float y_);

	bool Zero();

	char* ToString() const;
};
