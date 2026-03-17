#include "pch.h"
#include <iostream>

#include "World.h"
#include "RigidBody.h"
#include "Collisions.h"
#include "VectorMath.h"
#include "Constants.h"

World::World(Vector gravity_, int width_, int height_) : gravity(gravity_), width(width_), height(height_) {}
World::~World() {
	for (Connection* connection: connections) delete connection;
	for (Body* body : bodies) delete body;
}

World::AABB World::GetAABB(Body* body) const {
	if (body->GetShapeType() == Body::ShapeTypes::Circle) {
		float rx = body->GetRadiusX();
		float ry = body->GetRadiusY();
		float r = rx > ry ? rx : ry;

		return {
			body->GetPosition().GetX() - r,
			body->GetPosition().GetY() - r,
			body->GetPosition().GetX() + r,
			body->GetPosition().GetY() + r
		};
	}

	const std::vector<Vector>& pts = body->GetGlobalPoints();
	float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
	for (const Vector& p : pts) {
		if (p.GetX() < minX) minX = p.GetX();
		if (p.GetY() < minY) minY = p.GetY();
		if (p.GetX() > maxX) maxX = p.GetX();
		if (p.GetY() > maxY) maxY = p.GetY();
	}
	return { minX, minY, maxX, maxY };
}

bool World::AABBOverlap(const AABB& a, const AABB& b) const {
	return !(a.maxX < b.minX || b.maxX < a.minX ||
		a.maxY < b.minY || b.maxY < a.minY);
}

const Vector& World::GetGravity() const {
	return gravity;
}

void World::SetGravity(const Vector gravity_) {
	gravity = gravity_;
}

const std::vector<Body*>& World::GetBodies() const {
	return bodies;
}

const std::vector<Connection*>& World::GetConnections() const {
	return connections;
}

const std::vector<Vector>& World::GetContactPoints() const {
	return contactPoints;
}

void World::AddBody(Body* body) {
	bodies.push_back(body);
}

void World::AddConnection(Connection* connection) {
	connections.push_back(connection);
}

void World::RemoveBody(Body* body) {
	for (int i = 0; i < (int) bodies.size(); i++) {
		if (bodies[i] == body) {
			std::swap(bodies[i], bodies.back());
			bodies.pop_back();
			return;
		}
	}
}

void World::RemoveConnection(Connection* connection) {
	for (int i = 0; i < (int) connections.size(); i++) {
		if (connections[i] == connection) {
			delete connections[i];
			std::swap(connections[i], connections.back());
			connections.pop_back();
			return;
		}
	}
}

void World::RemoveBodyAndConnections(Body* body) {
	for (int i = (int) connections.size() - 1; i >= 0; i--) {
		if (connections[i]->GetBody1() == body || connections[i]->GetBody2() == body) {
			delete connections[i];
			std::swap(connections[i], connections.back());
			connections.pop_back();
		}
	}

	delete body;
	RemoveBody(body);
}

bool World::OffScreen(Body* body) {
	if (body->GetBodyType() == Body::BodyTypes::RigidBody) {
		if (body->GetShapeType() == Body::ShapeTypes::Circle) {
			float circleY = body->GetPosition().GetY();

			float rotation = body->GetRotation();

			float radiusX = body->GetRadiusX();
			float radiusY = body->GetRadiusY();

			float yPartA = powf(sinf(rotation), 2) / powf(radiusX, 2);
			float yPartB = powf(cosf(rotation), 2) / powf(radiusY, 2);
			float intersectionY = circleY - (1 / sqrtf(yPartA + yPartB));

			return intersectionY > height;
		}
		else if (body->GetShapeType() == Body::ShapeTypes::Polygon) {
			const std::vector<Vector>& points = body->GetGlobalPoints();

			float maxHeight = 1e09;

			for (const Vector& point : points) {
				if (point.GetY() < maxHeight)
					maxHeight = point.GetY();
			}

			if (maxHeight != 1e09)
				return maxHeight > height;
		}
	}

	return false;
}

bool World::AreHinged(Body* body1, Body* body2) const {
	for (Connection* connection : connections) {
		if (dynamic_cast<Hinge*>(connection) == nullptr) continue;
		if ((connection->GetBody1() == body1 && connection->GetBody2() == body2) || (connection->GetBody1() == body2 && connection->GetBody2() == body1))
			return true;
	}
	return false;
}

void ResolveCollision(Body* body, Body* otherBody, RigidBody* rigidBody, RigidBody* futureRigidBody, Vector penetrationVector) {
	Vector normal = VectorMath::Normalize(penetrationVector);
	float depth = VectorMath::Norm(penetrationVector);

	Vector relativeVelocity = VectorMath::Subtract(futureRigidBody->GetVelocity(), rigidBody->GetVelocity());
	float restitution;

	if (body->GetBodyType() == Body::BodyTypes::StaticBody)
		restitution = futureRigidBody->getCoefficientOfRestitution();
	else if (otherBody->GetBodyType() == Body::BodyTypes::StaticBody)
		restitution = rigidBody->getCoefficientOfRestitution();
	else
		restitution = std::fminf(rigidBody->getCoefficientOfRestitution(), futureRigidBody->getCoefficientOfRestitution());

	float impulseMagnitude = -(1.0f + restitution) * VectorMath::Dot(relativeVelocity, normal);
	impulseMagnitude /= (1.0f / rigidBody->GetMass()) + (1.0f / futureRigidBody->GetMass());

	if (body->GetBodyType() == Body::BodyTypes::StaticBody) {
		futureRigidBody->SetPosition(VectorMath::Add(futureRigidBody->GetPosition(), penetrationVector));

		futureRigidBody->ApplyVelocity(VectorMath::Multiply(normal, impulseMagnitude / rigidBody->GetMass()));
	}
	else if (otherBody->GetBodyType() == Body::BodyTypes::StaticBody) {
		rigidBody->SetPosition(VectorMath::Subtract(rigidBody->GetPosition(), penetrationVector));

		rigidBody->ApplyVelocity(VectorMath::Multiply(normal, -impulseMagnitude / rigidBody->GetMass()));
	}
	else {
		rigidBody->SetPosition(VectorMath::Subtract(rigidBody->GetPosition(), VectorMath::Multiply(penetrationVector, 0.5f)));
		futureRigidBody->SetPosition(VectorMath::Add(futureRigidBody->GetPosition(), VectorMath::Multiply(penetrationVector, 0.5f)));

		rigidBody->ApplyVelocity(VectorMath::Multiply(normal, -impulseMagnitude / rigidBody->GetMass()));
		futureRigidBody->ApplyVelocity(VectorMath::Multiply(normal, impulseMagnitude / rigidBody->GetMass()));
	}
}

void World::Update(float deltaTime, int iterations) {
	float timeStep = deltaTime / iterations;

	for (int iter = 0; iter < iterations; iter++) {
		contactPoints.clear();

		for (Body* b : bodies)
			if (b->GetBodyType() == Body::BodyTypes::RigidBody)
				static_cast<RigidBody*>(b)->ClearContactList();

		for (Body* b : bodies) {
			if (b->GetBodyType() != Body::BodyTypes::RigidBody) continue;
			RigidBody* rb = static_cast<RigidBody*>(b);

			rb->ApplyGravity(gravity, timeStep);
			rb->UpdateVelocity(timeStep);
			rb->UpdatePositionFromVelocity(timeStep);
		}

		std::vector<AABB> aabbs(bodies.size());
		for (int i = 0; i < bodies.size(); i++)
			aabbs[i] = GetAABB(bodies[i]);

		struct CollisionInfo {
			Body* bodyA;
			Body* bodyB;
			Vector penetration;
		};
		std::vector<CollisionInfo> collisions;

		for (int i = 0; i < bodies.size(); i++) {
			Body* bodyA = bodies[i];
			if (bodyA->GetBodyType() != Body::BodyTypes::RigidBody) continue;
			RigidBody* rbA = static_cast<RigidBody*>(bodyA);

			for (int j = 0; j < bodies.size(); j++) {
				if (i == j) continue;

				Body* bodyB = bodies[j];
				if (!AABBOverlap(aabbs[i], aabbs[j])) continue;

				if (AreHinged(bodyA, bodyB)) continue;

				Vector penetration = Collisions::RigidBodyCollision(rbA, bodyB, contactPoints);
				if (VectorMath::Norm(penetration) > COLLISION_THRESHOLD)
					collisions.push_back({ bodyA, bodyB, penetration });
			}
		}

		for (auto& col : collisions) {
			RigidBody* rbA = static_cast<RigidBody*>(col.bodyA);
			RigidBody* rbB = (col.bodyB->GetBodyType() == Body::BodyTypes::RigidBody)
				? static_cast<RigidBody*>(col.bodyB)
				: nullptr;

			Collisions::ResolveCollisionWithRotation(
				col.bodyA, col.bodyB, rbA, rbB,
				col.penetration,
				col.bodyA->GetGlobalPoints(),
				col.bodyB->GetGlobalPoints()
			);
		}

		for (Connection* c : connections)
			c->Update(timeStep, gravity);
		
		/*
		for (Body* b : bodies) {
			if (b->GetBodyType() != Body::BodyTypes::RigidBody) continue;
			static_cast<RigidBody*>(b)->UpdatePositionFromVelocity(timeStep);
		}
		*/

		for (int i = 0; i < bodies.size(); i++) {
			if (OffScreen(bodies[i])) {
				RemoveBodyAndConnections(bodies[i]);
				i--;
			}
		}
	}
}
