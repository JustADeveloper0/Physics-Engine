#include "pch.h"
#include "Hinge.h"

#include "RigidBody.h"
#include "VectorMath.h"

Hinge::Hinge(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_, float compliance_)
    : Connection(body1_, body1RelativePosition_, body2_, body2RelativePosition_),
    compliance(compliance_) {
}

Hinge::~Hinge() = default;

void ApplyHingeTorque(RigidBody* rb, const Vector& localPivot, float dt, Vector gravity) {
    Vector pivotWorld = VectorMath::Add(rb->GetPosition(), localPivot);
    Vector lever = VectorMath::Subtract(rb->GetPosition(), pivotWorld);

    float torque = lever.GetX() * gravity.GetY() - lever.GetY() * gravity.GetX();
    rb->ApplyTorque(torque * dt);
}

void Hinge::Update(float deltaTime, Vector gravity) {
    Body* body1 = const_cast<Body*>(GetBody1());
    Body* body2 = const_cast<Body*>(GetBody2());

    RigidBody* rb1 = (body1->GetBodyType() == Body::BodyTypes::RigidBody)
        ? static_cast<RigidBody*>(body1) : nullptr;
    RigidBody* rb2 = (body2->GetBodyType() == Body::BodyTypes::RigidBody)
        ? static_cast<RigidBody*>(body2) : nullptr;

    if (!rb1 && !rb2) return;

    Vector worldR1 = VectorMath::Rotate(GetBody1RelativePosition(), body1->GetRotation());
    Vector worldR2 = VectorMath::Rotate(GetBody2RelativePosition(), body2->GetRotation());

    Vector p1 = VectorMath::Add(body1->GetPosition(), worldR1);
    Vector p2 = VectorMath::Add(body2->GetPosition(), worldR2);

    Vector delta = VectorMath::Subtract(p2, p1);

    float invMass1 = rb1 ? 1.0f / rb1->GetMass() : 0.0f;
    float invMass2 = rb2 ? 1.0f / rb2->GetMass() : 0.0f;

    if (!rb1 && rb2) {
        rb2->SetPosition(VectorMath::Subtract(rb2->GetPosition(), delta));

        Vector pivotVel = rb2->GetPointVelocity(p2);
        rb2->ApplyVelocity(VectorMath::Multiply(pivotVel, -1.0f));

        ApplyHingeTorque(rb2, worldR2, deltaTime, gravity);
        rb2->SetAngularVelocity(rb2->GetAngularVelocity() * 0.98f);
    }
    else if (!rb2 && rb1) {
        rb1->SetPosition(VectorMath::Add(rb1->GetPosition(), delta));

        Vector pivotVel = rb1->GetPointVelocity(p1);
        rb1->ApplyVelocity(VectorMath::Multiply(pivotVel, -1.0f));

        ApplyHingeTorque(rb1, worldR1, deltaTime, gravity);
        rb1->SetAngularVelocity(rb1->GetAngularVelocity() * 0.98f);
    }
    else if (rb1 && rb2) {
        float totalInvMass = invMass1 + invMass2;
        if (totalInvMass > 0.0f) {
            rb1->SetPosition(VectorMath::Add(rb1->GetPosition(), VectorMath::Multiply(delta, invMass1 / totalInvMass)));
            rb2->SetPosition(VectorMath::Subtract(rb2->GetPosition(), VectorMath::Multiply(delta, invMass2 / totalInvMass)));

            Vector v1 = rb1->GetPointVelocity(p1);
            Vector v2 = rb2->GetPointVelocity(p2);
            Vector relativeVel = VectorMath::Subtract(v2, v1);

            rb1->ApplyVelocity(VectorMath::Multiply(relativeVel, invMass1 / totalInvMass));
            rb2->ApplyVelocity(VectorMath::Multiply(relativeVel, -invMass2 / totalInvMass));
        }

        ApplyHingeTorque(rb1, worldR1, deltaTime, gravity);
        ApplyHingeTorque(rb2, worldR2, deltaTime, gravity);
        rb1->SetAngularVelocity(rb1->GetAngularVelocity() * 0.98f);
        rb2->SetAngularVelocity(rb2->GetAngularVelocity() * 0.98f);
    }
}
