#include "pch.h"
#include "Rope.h"

#include "Body.h"
#include "VectorMath.h"
#include "RigidBody.h"

Rope::Rope(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_, int numberOfSegments_, float stiffness_)
	: Connection(body1_, body1RelativePosition_, body2_, body2RelativePosition_), numberOfSegments(numberOfSegments_), stiffness(stiffness_) {
	Vector anchor1 = VectorMath::Add(GetBody1()->GetPosition(), GetBody1RelativePosition());
	Vector anchor2 = VectorMath::Add(GetBody2()->GetPosition(), GetBody2RelativePosition());

    restLength = VectorMath::Distance(anchor1, anchor2);
    segmentLength = restLength / numberOfSegments;

	nodes.resize(numberOfSegments + 1);

	for (int i = 0; i <= numberOfSegments; i++) {
		float t = (float) i / numberOfSegments;
		nodes[i].position = Vector(
			anchor1.GetX() + t * (anchor2.GetX() - anchor1.GetX()),
			anchor1.GetY() + t * (anchor2.GetY() - anchor1.GetY())
		);
		nodes[i].previousPosition = nodes[i].position;
		nodes[i].inverseMass = 1.0f;
	}

	nodes[0].inverseMass = 0.0f;
	nodes[numberOfSegments].inverseMass = 0.0f;
}

Rope::~Rope() = default;

float Rope::GetRestLength() const {
    return restLength;
}

const std::vector<Rope::RopeNode>& Rope::GetNodes() const {
    return nodes;
}

void Rope::Update(float deltaTime, Vector gravity) {
    Body* body1 = const_cast<Body*>(GetBody1());
    Body* body2 = const_cast<Body*>(GetBody2());
    Vector body1RelativePosition = GetBody1RelativePosition();
    Vector body2RelativePosition = GetBody2RelativePosition();

    float rot1 = body1->GetRotation();
    Vector r1 = body1RelativePosition;
    Vector worldR1(
        r1.GetX() * cosf(rot1) - r1.GetY() * sinf(rot1),
        r1.GetX() * sinf(rot1) + r1.GetY() * cosf(rot1)
    );

    nodes[0].position = VectorMath::Add(body1->GetPosition(), worldR1);

    float rot2 = body2->GetRotation();
    Vector r2 = body2RelativePosition;
    Vector worldR2(
        r2.GetX() * cosf(rot2) - r2.GetY() * sinf(rot2),
        r2.GetX() * sinf(rot2) + r2.GetY() * cosf(rot2)
    );
    nodes[numberOfSegments].position = VectorMath::Add(body2->GetPosition(), worldR2);

    for (int i = 1; i < numberOfSegments; i++) {
        RopeNode& node = nodes[i];
        if (node.inverseMass == 0.0f) continue;

        Vector velocity = VectorMath::Subtract(node.position, node.previousPosition);
        node.previousPosition = node.position;

        velocity = VectorMath::Multiply(velocity, dampening);

        Vector gravityStep = VectorMath::Multiply(gravity, deltaTime * deltaTime);
        node.position = VectorMath::Add(VectorMath::Add(node.position, velocity), gravityStep);
    }

    for (int iter = 0; iter < solverIterations; iter++) {
        for (int i = 0; i < numberOfSegments; i++) {
            RopeNode& a = nodes[i];
            RopeNode& b = nodes[i + 1];

            Vector delta = VectorMath::Subtract(b.position, a.position);
            float dist = VectorMath::Norm(delta);
            if (dist == 0.0f) continue;

            float error = (dist - segmentLength) / dist;
            Vector correction = VectorMath::Multiply(delta, error * stiffness);

            float invMassSum = a.inverseMass + b.inverseMass;
            if (invMassSum == 0.0f) continue;

            if (a.inverseMass > 0.0f)
                a.position = VectorMath::Add(a.position,
                    VectorMath::Multiply(correction, a.inverseMass / invMassSum));
            if (b.inverseMass > 0.0f)
                b.position = VectorMath::Subtract(b.position,
                    VectorMath::Multiply(correction, b.inverseMass / invMassSum));
        }
    }

    nodes[0].position = VectorMath::Add(body1->GetPosition(), worldR1);
    nodes[numberOfSegments].position = VectorMath::Add(body2->GetPosition(), worldR2);

    RigidBody* rb1 = (body1->GetBodyType() == Body::BodyTypes::RigidBody)
        ? static_cast<RigidBody*>(body1) : nullptr;
    if (rb1) {
        Vector tension = VectorMath::Subtract(nodes[1].position, nodes[0].position);
        float dist = VectorMath::Norm(tension);
        if (dist > segmentLength) {
            float error = dist - segmentLength;
            Vector force = VectorMath::Multiply(VectorMath::Normalize(tension), error * stiffness / (deltaTime * deltaTime));
            rb1->ApplyForce(force);

            Vector leverArm = VectorMath::Subtract(nodes[0].position, rb1->GetPosition());
            float torque = VectorMath::Cross(leverArm, force);
            rb1->ApplyTorque(torque);
        }
    }

    RigidBody* rb2 = (body2->GetBodyType() == Body::BodyTypes::RigidBody)
        ? static_cast<RigidBody*>(body2) : nullptr;
    if (rb2) {
        Vector tension = VectorMath::Subtract(nodes[numberOfSegments - 1].position, nodes[numberOfSegments].position);
        float dist = VectorMath::Norm(tension);
        if (dist > segmentLength) {
            float error = dist - segmentLength;
            Vector force = VectorMath::Multiply(VectorMath::Normalize(tension), error * stiffness / (deltaTime * deltaTime));
            rb2->ApplyForce(force);

            Vector leverArm = VectorMath::Subtract(nodes[numberOfSegments].position, rb2->GetPosition());
            float torque = VectorMath::Cross(leverArm, force);
            rb2->ApplyTorque(torque);
        }
    }
}
