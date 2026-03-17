#pragma once
#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include "Vector.h"
#include "Body.h"

class PHYSICS_API Connection {
private:
	Body* body1;
	Body* body2;

	Vector body1RelativePosition;
	Vector body2RelativePosition;

public:
	Connection(Body* body1, Vector body1RelativePosition, Body* body2, Vector body2RelativePosition);

	virtual ~Connection();

	const Body* GetBody1() const;
	const Body* GetBody2() const;

	const Vector GetBody1RelativePosition() const;
	const Vector GetBody2RelativePosition() const;

	virtual void Update(float deltaTime, Vector gravity);
};
