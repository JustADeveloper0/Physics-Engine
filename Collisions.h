#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include <vector>
#include "Vector.h"
#include "RigidBody.h"

#define DISTANCE_THRESHOLD 0.01f
#define CONTACT_EPSILON 0.0001f

class PHYSICS_API Collisions {
public:
	struct Projection {
		float min;
		float max;
	};

	static Vector RigidBodyCollision(Body* body1, Body* body2, std::vector<Vector>& contactPoints);

	static Vector GetEdgeNormal(const Vector& point1, const Vector& point2);

	static bool CheckAxis(const std::vector<Vector>& points1, const std::vector<Vector>& points2, const Vector& axis,
		Vector& minimumNormal, float& minimumOverlap, const Vector& distance);

	static Projection ProjectPoints(const std::vector<Vector>& points, const Vector& axis);

	static std::vector<Vector> GetCirclePoints(Body* circle);

	static bool PointOnSegment(const Vector& point, const Vector& a, const Vector& b);

	static bool PointInPolygon(const Vector& point, const std::vector<Vector>& polygon);

	static void ResolveCollisionWithRotation(Body* body, Body* otherBody, RigidBody* rigidBody, RigidBody* futureRigidBody, Vector penetrationVector, const std::vector<Vector>& points1, const std::vector<Vector>& points2);
};
