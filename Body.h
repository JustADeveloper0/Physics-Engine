#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include <vector>
#include "Vector.h"

#define M_PI 3.14159265358979323846

class PHYSICS_API Body {
private:
	Vector position;
	float rotation;

	std::vector<Vector> points;

	mutable std::vector<Vector> globalPoints; // cache (mutable cuz it works with const for some reason)
	mutable bool globalPointsDirty = true;

	mutable std::vector<Vector> cachedCirclePoints;
	mutable bool circlePointsDirty = true;

	float radiusX;
	float radiusY;

	float area = -1.0f;

	int shapeType;
	int bodyType;

	float coefficientOfFriction;

public:
	struct ShapeTypes { // needed since smth in an enum isnt classified as an int for some reason
		static constexpr int Polygon = 1;
		static constexpr int Circle = 2;
	};

	struct BodyTypes {
		static constexpr int StaticBody = 1;
		static constexpr int RigidBody = 2;
	};

	Body(Vector& position_, const std::vector<Vector>& points_, float rotation, int bodyType_, float coefficientOfFriction_); // reference to not copy and save memory, maybe remove idk (okay for now, might be annoying)

	Body(Vector& position_, float radiusX_, float radiusY_, float rotation, int bodyType_, float coefficientOfFriction_);

	virtual ~Body();

	const Vector& GetPosition() const;

	void SetPosition(const Vector position_);

	const std::vector<Vector>& GetPoints() const;

	const std::vector<Vector>& GetGlobalPoints() const;
	const std::vector<Vector>& GetCirclePoints() const;

	float GetRadiusX();
	float GetRadiusY();

	void SetRadiusX(float radiusX);
	void SetRadiusY(float radiusY);

	float GetArea();

	float GetRotation();
	void SetRotation(float rotation_);

	int GetShapeType();
	int GetBodyType();

	float getCoefficientOfFriction() const;
};