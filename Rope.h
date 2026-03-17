#pragma once
#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include "Vector.h"
#include "Body.h"
#include "StaticBody.h"
#include "Connection.h"

class PHYSICS_API Rope : public Connection {
public:
	struct RopeNode {
		Vector position;
		Vector previousPosition;
		float inverseMass;
	};

private:
	float restLength;
	float segmentLength;
	int numberOfSegments;
	float stiffness;

	std::vector<RopeNode> nodes;

	const int solverIterations = 8;
	const float dampening = 0.995f;

public:
	Rope(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_, int numberOfSegments_ = 25, float stiffness_ = 0.8f);

	virtual ~Rope();

	float GetRestLength() const;

	const std::vector<RopeNode>& GetNodes() const;

	void Update(float deltaTime, Vector gravity) override;
};
