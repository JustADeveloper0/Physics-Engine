#include "pch.h"
#include "Spring.h"

#include "VectorMath.h"
#include "RigidBody.h"

Spring::Spring(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_, float stiffness_, float damping_)
	: Connection(body1_, body1RelativePosition_, body2_, body2RelativePosition_), stiffness(stiffness_), damping(damping_) {
	Vector anchor1 = VectorMath::Add(GetBody1()->GetPosition(), GetBody1RelativePosition());
	Vector anchor2 = VectorMath::Add(GetBody2()->GetPosition(), GetBody2RelativePosition());

	restLength = VectorMath::Distance(anchor1, anchor2);
}

Spring::~Spring() = default;

float Spring::GetRestLength() const {
	return restLength;
}

float Spring::GetStiffness() const {
	return stiffness;
}

void Spring::Update(float deltaTime, Vector gravity) {
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

    float rot2 = body2->GetRotation();
    Vector r2 = body2RelativePosition;
    Vector worldR2(
        r2.GetX() * cosf(rot2) - r2.GetY() * sinf(rot2),
        r2.GetX() * sinf(rot2) + r2.GetY() * cosf(rot2)
    );

    Vector anchor1 = VectorMath::Add(body1->GetPosition(), worldR1);
    Vector anchor2 = VectorMath::Add(body2->GetPosition(), worldR2);

    Vector delta = VectorMath::Subtract(anchor2, anchor1);
    float dist = VectorMath::Norm(delta);
    if (dist == 0.0f) return;

    Vector direction = VectorMath::Normalize(delta);

    float extension = dist - restLength;
    float springForceMag = stiffness * extension;

    RigidBody* rb1 = (body1->GetBodyType() == Body::BodyTypes::RigidBody)
        ? static_cast<RigidBody*>(body1) : nullptr;
    RigidBody* rb2 = (body2->GetBodyType() == Body::BodyTypes::RigidBody)
        ? static_cast<RigidBody*>(body2) : nullptr;

    auto getAnchorVelocity = [](RigidBody* rb, Vector leverArm) -> Vector {
        if (!rb) return Vector(0, 0);
        Vector v = rb->GetVelocity();
        float w = rb->GetAngularVelocity();
        return Vector(
            v.GetX() - w * leverArm.GetY(),
            v.GetY() + w * leverArm.GetX()
        );
        };

    Vector leverArm1 = VectorMath::Subtract(anchor1, body1->GetPosition());
    Vector leverArm2 = VectorMath::Subtract(anchor2, body2->GetPosition());

    Vector vel1 = getAnchorVelocity(rb1, leverArm1);
    Vector vel2 = getAnchorVelocity(rb2, leverArm2);

    Vector relVel = VectorMath::Subtract(vel2, vel1);
    float dampingForceMag = damping * VectorMath::Dot(relVel, direction);

    float totalForceMag = springForceMag + dampingForceMag;
    Vector force = VectorMath::Multiply(direction, totalForceMag);

    if (rb1) {
        rb1->ApplyForce(force);
        float torque = VectorMath::Cross(leverArm1, force);
        rb1->ApplyTorque(torque);
    }

    if (rb2) {
        Vector forceNeg = VectorMath::Multiply(force, -1.0f);
        rb2->ApplyForce(forceNeg);
        float torque = VectorMath::Cross(leverArm2, forceNeg);
        rb2->ApplyTorque(torque);
    }
}
