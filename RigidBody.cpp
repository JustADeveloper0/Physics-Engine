#include "pch.h"
#include <iostream>

#include "RigidBody.h"
#include "VectorMath.h"

RigidBody::RigidBody(Vector position_, const std::vector<Vector>& points_, float rotation_, float mass_,
	float coefficientOfFriction_, float coefficientOfRestitution_,
	Vector velocity_, Vector acceleration_, Vector force_, float angularVelocity_, float angularAcceleration_,
	float torque_) :
	Body(position_, points_, rotation_, Body::BodyTypes::RigidBody, coefficientOfFriction_), mass(mass_), velocity(velocity_),
	acceleration(acceleration_), force(force_), angularVelocity(angularVelocity_), angularAcceleration(angularAcceleration_),
	torque(torque_), coefficientOfRestitution(coefficientOfRestitution_) {
}

RigidBody::RigidBody(Vector position_, float radius_x, float radius_y, float rotation_, float mass_,
	float coefficientOfFriction_, float coefficientOfRestitution_,
	Vector velocity_, Vector acceleration_, Vector force_, float angularVelocity_, float angularAcceleration_, 
	float torque_) :
	Body(position_, radius_x, radius_y, rotation_, Body::BodyTypes::RigidBody, coefficientOfFriction_), mass(mass_), velocity(velocity_),
	acceleration(acceleration_), force(force_), angularVelocity(angularVelocity_), angularAcceleration(angularAcceleration_), 
	torque(torque_), coefficientOfRestitution(coefficientOfRestitution_) {
}

RigidBody::~RigidBody() = default;

float RigidBody::GetMass() {
	return mass;
}

float RigidBody::GetInertia() {
	if (inertia < 0) {
		if (GetShapeType() == Body::ShapeTypes::Circle)
			inertia = 0.25f * mass * (GetRadiusX() * GetRadiusX() + GetRadiusY() * GetRadiusY());

		if (GetShapeType() == Body::ShapeTypes::Polygon) {
			float sum = 0;

			for (int i = 0; i < GetPoints().size(); i++) {
				const Vector& p1 = GetPoints()[i];
				const Vector& p2 = GetPoints()[(i + 1) % GetPoints().size()];
				float edgeCross = VectorMath::Cross(p1, p2);
				float vertexDotSum = VectorMath::Dot(p1, p1) + VectorMath::Dot(p1, p2) + VectorMath::Dot(p2, p2);
				sum += edgeCross * vertexDotSum;
			}

			inertia = (mass * std::abs(sum)) / (12.0f * GetArea());
		}
	}

	return inertia;
}

void RigidBody::SetMass(const float mass_) {
	mass = mass_;
}

const Vector& RigidBody::GetVelocity() const {
	return velocity;
}

void RigidBody::SetVelocity(const Vector velocity_) {
	velocity = velocity_;
}

void RigidBody::ApplyVelocity(const Vector velocity_) {
	velocity = VectorMath::Add(velocity_, velocity);
}

void RigidBody::ApplyImpulse(const Vector impulse) {
	velocity = VectorMath::Add(velocity, VectorMath::Divide(impulse, mass));
}

const Vector& RigidBody::GetAcceleration() const {
	return acceleration;
}

void RigidBody::SetAcceleration(const Vector acceleration_) {
	acceleration = acceleration_;
}

void RigidBody::ApplyAcceleration(const Vector acceleration_) {
	acceleration = VectorMath::Add(acceleration_, acceleration);
}

const Vector& RigidBody::GetForce() const {
	return force;
}

void RigidBody::SetForce(const Vector force_) {
	force = force_;
}

void RigidBody::ApplyForce(const Vector force_) {
	force = VectorMath::Add(force_, force);
}

float RigidBody::GetAngularVelocity() {
	return angularVelocity;
}

void RigidBody::SetAngularVelocity(float angularVelocity_) {
	angularVelocity = angularVelocity_;
}

void RigidBody::ApplyAngularVelocity(float angularVelocity_) {
	angularVelocity += angularVelocity_;
}

float RigidBody::GetAngularAcceleration() {
	return angularAcceleration;
}

void RigidBody::SetAngularAcceleration(float angularAcceleration_) {
	angularAcceleration = angularAcceleration_;
}

void RigidBody::ApplyAngularAcceleration(float angularAcceleration_) {
	angularAcceleration += angularAcceleration_;
}

float RigidBody::GetTorque() {
	return torque;
}

void RigidBody::SetTorque(float torque_) {
	torque = torque_;
}

void RigidBody::ApplyTorque(float torque_) {
	torque += torque_;
}

void RigidBody::SetContactList(const std::vector<Vector>& contactList_) {
	contactList = contactList_;
}

void RigidBody::ClearContactList() {
	contactList.clear();
}

void RigidBody::AddContactPoint(const Vector& point) {
	contactList.push_back(point);
}

bool RigidBody::HasContactPoint(const Vector& point) const {
	for (const Vector& contactPoint : contactList) {
		if (VectorMath::Equal(contactPoint, point))
			return true;
	}
	return false;
}

const std::vector<Vector>* RigidBody::GetContactList() const {
	return &contactList;
}

float RigidBody::getCoefficientOfRestitution() const {
	return coefficientOfRestitution;
}

void RigidBody::ApplyGravity(const Vector& gravity, float deltaTime) {
	force = VectorMath::Add(force, VectorMath::Multiply(gravity, mass));
}

Vector RigidBody::GetPointVelocity(const Vector& point) const {
	Vector r = VectorMath::Subtract(point, GetPosition());
	return VectorMath::Add(velocity, Vector(-angularVelocity * r.GetY(), angularVelocity * r.GetX()));
}

void RigidBody::UpdateVelocity(float deltaTime) {
	acceleration = VectorMath::Divide(force, mass);
	velocity = VectorMath::Add(velocity, VectorMath::Multiply(acceleration, deltaTime));

	angularAcceleration += torque / GetInertia();
	angularVelocity += angularAcceleration * deltaTime;

	force = Vector();
	torque = 0.0f;
	acceleration = Vector();
	angularAcceleration = 0.0f;
}

void RigidBody::UpdatePositionFromVelocity(float deltaTime) {
	SetPosition(VectorMath::Add(GetPosition(), VectorMath::Multiply(velocity, deltaTime)));
	SetRotation(GetRotation() + angularVelocity * deltaTime);
}
