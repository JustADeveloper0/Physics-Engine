#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include <vector>
#include "Vector.h"
#include "Body.h"
#include "Connection.h"
#include "Hinge.h"

#define SMALL_VELOCITY_THRESHOLD 0.001f
#define REST_VELOCITY_THRESHOLD 0.001f

class PHYSICS_API World {
private:
	std::vector<Body*> bodies;
	std::vector<Connection*> connections;

	Vector gravity;

	int width;
	int height;

	std::vector<Vector> contactPoints;

	const float COLLISION_THRESHOLD = 0.001f;

	struct AABB {
		float minX, minY, maxX, maxY;
	};

	AABB GetAABB(Body* body) const;
	bool AABBOverlap(const AABB& a, const AABB& b) const;

public:
	World(Vector gravity_, int width_, int height_);

	~World();

	const Vector& GetGravity() const;

	void SetGravity(const Vector gravity_);

	const std::vector<Body*>& GetBodies() const;

	const std::vector<Connection*>& GetConnections() const;

	const std::vector<Vector>& GetContactPoints() const;

	void AddBody(Body* body);

	void AddConnection(Connection* connection);

	void RemoveBody(Body* body);

	void RemoveConnection(Connection* rope);

	void RemoveBodyAndConnections(Body* body);

	bool OffScreen(Body* body);

	bool AreHinged(Body* body1, Body* body2) const;

	void Update(float deltaTime, int iterations);
};
