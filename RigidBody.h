#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include <vector>
#include "Vector.h"
#include "Body.h"

class PHYSICS_API RigidBody : public Body {
private:
	float mass;
	float inertia = -1.0f;

	Vector velocity;
	Vector acceleration;
	Vector force;

	float angularVelocity;
	float angularAcceleration;
	float torque;

	float coefficientOfRestitution;

	float airDragForce; // coefficient to a function passed as a parameter ? honestly still don't know yet
	float waterDragForce; // coefficient to a function passed as a parameter ? honestly still don't know yet
	float damping; // air drag force or water drag force

	std::vector<Vector> contactList;

	//const float ANGULAR_FALL_DEADZONE = 0.05f;
	//const float ANGULAR_DAMPING = 0.98f;

public:

	RigidBody(Vector position_, const std::vector<Vector>& points_, float rotation_, float mass_,
		float coefficientOfFriction_ = 0.4f, float coefficientOfRestitution_ = 0.6f,
		Vector velocity_ = Vector(), Vector acceleration_ = Vector(), Vector force_ = Vector(), 
		float angularVelocity_ = 0.0f, float angularAcceleration_ = 0.0f, float torque_ = 0.0f);

	RigidBody(Vector position_, float radius_x, float radius_y, float rotation_, float mass_,
		float coefficientOfFriction_ = 0.4f, float coefficientOfRestitution_ = 0.6f,
		Vector velocity_ = Vector(), Vector acceleration_ = Vector(), Vector force_ = Vector(),
		float angularVelocity_ = 0.0f, float angularAcceleration_ = 0.0f, float torque_ = 0.0f);

	~RigidBody() override;

	float GetMass();
	float GetInertia();

	void SetMass(const float mass_);

	const Vector& GetVelocity() const;

	void SetVelocity(const Vector velocity_);

	void ApplyVelocity(const Vector velocity_);

	void ApplyImpulse(const Vector impulse_);

	const Vector& GetAcceleration() const;
	
	void SetAcceleration(const Vector acceleration_);

	void ApplyAcceleration(const Vector acceleration_);

	const Vector& GetForce() const;

	void SetForce(const Vector force_);

	void ApplyForce(const Vector force_);

	float GetAngularVelocity();
	void SetAngularVelocity(float angularVelocity_);
	void ApplyAngularVelocity(float angularVelocity_);

	float GetAngularAcceleration();
	void SetAngularAcceleration(float angularAcceleration_);
	void ApplyAngularAcceleration(float angularAcceleration_);

	float GetTorque();
	void SetTorque(float torque_);
	void ApplyTorque(float torque_);

	void SetContactList(const std::vector<Vector>& contactList_);
	void ClearContactList();
	void AddContactPoint(const Vector& point);
	bool HasContactPoint(const Vector& point) const;
	const std::vector<Vector>* GetContactList() const;

	float getCoefficientOfRestitution() const;

	void ApplyGravity(const Vector& gravity, float deltaTime);

	void UpdateVelocity(float deltaTime);
	void UpdatePositionFromVelocity(float deltaTime);

	Vector GetPointVelocity(const Vector& point) const;

	void Update(float deltaTime, Vector gravity, bool colliding);
};
