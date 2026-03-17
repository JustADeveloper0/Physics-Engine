#include "pch.h"
#include <iostream>

#include "Collisions.h"
#include "VectorMath.h"
#include "RigidBody.h"
#include "Constants.h"

#define M_PI 3.14159265358979323846

bool Collisions::PointOnSegment(const Vector& point, const Vector& a, const Vector& b) {
    Vector ab = VectorMath::Subtract(b, a);
    Vector ap = VectorMath::Subtract(point, a);

    float cross = VectorMath::Cross(ab, ap);
    if (fabs(cross) > CONTACT_EPSILON) return false;

    float dot = VectorMath::Dot(ap, ab);
    if (dot < -CONTACT_EPSILON) return false;

    float abLenSq = VectorMath::Dot(ab, ab);
    if (dot - abLenSq > CONTACT_EPSILON) return false;

    return true;
}

bool Collisions::PointInPolygon(const Vector& point, const std::vector<Vector>& polygon) {
    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if (PointOnSegment(point, polygon[j], polygon[i]))
            return true;
    }

    bool inside = false;

    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const Vector& pi = polygon[i];
        const Vector& pj = polygon[j];

        if (((pi.GetY() > point.GetY()) != (pj.GetY() > point.GetY()))
            && (point.GetX() < (pj.GetX() - pi.GetX()) * (point.GetY() - pi.GetY()) / (pj.GetY() - pi.GetY()) + pi.GetX()))
            inside = !inside;
    }

    return inside;
}

Vector Collisions::RigidBodyCollision(Body* body1, Body* body2, std::vector<Vector>& contactPoints) {
    if (body1->GetShapeType() == Body::ShapeTypes::Circle && body2->GetShapeType() == Body::ShapeTypes::Circle
        && body1->GetRadiusX() == body1->GetRadiusY() && body2->GetRadiusX() == body2->GetRadiusY()) {

        float dx = body2->GetPosition().GetX() - body1->GetPosition().GetX();
        float dy = body2->GetPosition().GetY() - body1->GetPosition().GetY();
        float radiiSum = body1->GetRadiusX() + body2->GetRadiusX();

        float distSq = dx * dx + dy * dy;
        if (distSq >= radiiSum * radiiSum) return Vector(0, 0);

        float dist = sqrtf(distSq);
        float depth = radiiSum - dist;
        Vector normal = dist > 0 ? Vector(dx / dist, dy / dist) : Vector(1, 0);

        contactPoints.push_back(Vector(
            body1->GetPosition().GetX() + normal.GetX() * body1->GetRadiusX(),
            body1->GetPosition().GetY() + normal.GetY() * body1->GetRadiusX()
        ));

        return VectorMath::Multiply(normal, depth);
    }

    Vector position1 = body1->GetPosition();
    Vector position2 = body2->GetPosition();

    Vector distance = VectorMath::Subtract(position2, position1);

    const std::vector<Vector>& points1 = (body1->GetShapeType() == Body::ShapeTypes::Circle) ? body1->GetCirclePoints() : body1->GetGlobalPoints();
    const std::vector<Vector>& points2 = (body2->GetShapeType() == Body::ShapeTypes::Circle) ? body2->GetCirclePoints() : body2->GetGlobalPoints();

    Vector minimumNormal(0, 0);
    float minimumOverlap = 1e09;

    for (int i = 0; i < points1.size(); i++) {
        const Vector& point1 = points1[i];
        const Vector& point2 = (i + 1 == points1.size()) ? points1[0] : points1[i + 1]; // i + 1 is just safe

        Vector normal = GetEdgeNormal(point2, point1);

        if (!CheckAxis(points1, points2, normal, minimumNormal, minimumOverlap, distance))
            return Vector(0, 0);
    }

    for (int i = 0; i < points2.size(); i++) {
        const Vector& point1 = points2[i];
        const Vector& point2 = (i + 1 == points2.size()) ? points2[0] : points2[i + 1]; // i + 1 is just safe

        Vector normal = GetEdgeNormal(point2, point1);

        if (!CheckAxis(points1, points2, normal, minimumNormal, minimumOverlap, distance))
            return Vector(0, 0);
    }

    RigidBody* rigidBody1 = (body1->GetBodyType() == Body::BodyTypes::RigidBody) ? static_cast<RigidBody*>(body1) : nullptr;
    RigidBody* rigidBody2 = (body2->GetBodyType() == Body::BodyTypes::RigidBody) ? static_cast<RigidBody*>(body2) : nullptr;

    for (const Vector& point : points1) {
        if (PointInPolygon(point, points2)) {
            contactPoints.push_back(point);

            if (rigidBody1) rigidBody1->AddContactPoint(point);
            if (rigidBody2) rigidBody2->AddContactPoint(point);
        }
    }

    for (const Vector& point : points2) {
        if (PointInPolygon(point, points1)) {
            contactPoints.push_back(point);

            if (rigidBody1) rigidBody1->AddContactPoint(point);
            if (rigidBody2) rigidBody2->AddContactPoint(point);
        }
    }

    return VectorMath::Multiply(minimumNormal, minimumOverlap);
}

Vector Collisions::GetEdgeNormal(const Vector& point1, const Vector& point2) {
    Vector edge = VectorMath::Subtract(point2, point1);
    Vector normal(-edge.GetY(), edge.GetX()); // its like a perpendicular slope (m = 2/3, new m = -3/2)
    return VectorMath::Normalize(normal);
}

bool Collisions::CheckAxis(const std::vector<Vector>& points1, const std::vector<Vector>& points2, const Vector& axis,
    Vector& minimumNormal, float& minimumOverlap, const Vector& distance) {
    Projection projection1 = ProjectPoints(points1, axis);
    Projection projection2 = ProjectPoints(points2, axis);

    if (projection1.max < projection2.min || projection2.max < projection1.min) return false;

    float maxVal = (projection1.max < projection2.max) ? projection1.max : projection2.max;
    float minVal = (projection1.min > projection2.min) ? projection1.min : projection2.min;
    float overlap = maxVal - minVal;


    if (overlap < minimumOverlap) {
        minimumOverlap = overlap;
        minimumNormal = axis;

        if (VectorMath::Dot(distance, minimumNormal) < 0)
            minimumNormal = VectorMath::Multiply(minimumNormal, -1);
    }

    return true;
}

Collisions::Projection Collisions::ProjectPoints(const std::vector<Vector>& points, const Vector& axis) {
    float min = 1e9; // later setup as maybe first point or smth
    float max = -1e9;

    for (const Vector& point : points) {
        float projected = VectorMath::Dot(point, axis);
        if (projected < min) min = projected;
        if (projected > max) max = projected;
    }

    return { min, max };
}

std::vector<Vector> Collisions::GetCirclePoints(Body* circle) {
    std::vector<Vector> points;
    points.reserve(CircleDetectionAccuracy); // no reallocation = better preformance

    float radiusX = circle->GetRadiusX();
    float radiusY = circle->GetRadiusY();
    float rotation = circle->GetRotation();
    Vector position = circle->GetPosition();

    for (int i = 0; i < CircleDetectionAccuracy; i++) {
        float theta = (2.0f * M_PI * i) / CircleDetectionAccuracy;

        float x = radiusX * cosf(theta);
        float y = radiusY * sinf(theta);

        float rotatedX = x * cosf(rotation) - y * sinf(rotation);
        float rotatedY = x * sinf(rotation) + y * cosf(rotation);

        points.emplace_back(rotatedX + position.GetX(), rotatedY + position.GetY()); // same as push back just doesnt make a copy
    }

    return points;
}

void Collisions::ResolveCollisionWithRotation(
    Body* body, Body* otherBody,
    RigidBody* rbA, RigidBody* rbB,
    Vector penetrationVector,
    const std::vector<Vector>& points1,
    const std::vector<Vector>& points2)
{
    if (!rbA && !rbB) return;

    Vector normal = VectorMath::Normalize(penetrationVector);
    float depth = VectorMath::Norm(penetrationVector);

    const float invMassA = (rbA && rbA->GetMass() > 0.0f) ? 1.0f / rbA->GetMass() : 0.0f;
    const float invMassB = (rbB && rbB->GetMass() > 0.0f) ? 1.0f / rbB->GetMass() : 0.0f;

    std::vector<Vector> contactPoints;

    if (rbA) {
        const std::vector<Vector>* listA = rbA->GetContactList();
        if (listA) {
            for (const Vector& p : *listA) {
                if (Collisions::PointInPolygon(p, points2))
                    contactPoints.push_back(p);
            }
        }
    }
    if (rbB) {
        const std::vector<Vector>* listB = rbB->GetContactList();
        for (const Vector& p : *listB) {
            if (Collisions::PointInPolygon(p, points1))
                contactPoints.push_back(p);
        }
    }

    if (contactPoints.empty() && depth > 0.0f) {
        Vector fallback = VectorMath::Add(
            rbA ? rbA->GetPosition() : Vector(0, 0),
            VectorMath::Multiply(normal, rbA && rbB ? 0.5f * depth : depth)
        );
        contactPoints.push_back(fallback);
    }

    const float contactCount = static_cast<float>(contactPoints.size());
    const float POSITION_PERCENT = 0.8f;
    const float POSITION_SLOP = 0.01f;

    float correctionMag = (depth > POSITION_SLOP) ? (depth - POSITION_SLOP) * POSITION_PERCENT : 0.0f;
    float invMassSum = invMassA + invMassB;
    if (invMassSum > 0.0f && correctionMag > 0.0f) {
        Vector correction = VectorMath::Multiply(normal, correctionMag / invMassSum);
        if (rbA) rbA->SetPosition(VectorMath::Subtract(rbA->GetPosition(), VectorMath::Multiply(correction, invMassA)));
        if (rbB) rbB->SetPosition(VectorMath::Add(rbB->GetPosition(), VectorMath::Multiply(correction, invMassB)));
    }

    float restitution = 0.0f;
    if (body->GetBodyType() == Body::BodyTypes::StaticBody) restitution = rbB ? rbB->getCoefficientOfRestitution() : 0.0f;
    else if (otherBody->GetBodyType() == Body::BodyTypes::StaticBody) restitution = rbA ? rbA->getCoefficientOfRestitution() : 0.0f;
    else if (rbA && rbB) restitution = std::fminf(rbA->getCoefficientOfRestitution(), rbB->getCoefficientOfRestitution());

    float mu = (body->getCoefficientOfFriction() + otherBody->getCoefficientOfFriction()) * 0.5f;
    const float EPS_IMPULSE = 1e-4f;

    std::vector<Vector> impulses;
    std::vector<Vector> rA_list, rB_list;

    for (const Vector& contactPoint : contactPoints) {
        Vector rA = rbA ? VectorMath::Subtract(contactPoint, rbA->GetPosition()) : Vector(0, 0);
        Vector rB = rbB ? VectorMath::Subtract(contactPoint, rbB->GetPosition()) : Vector(0, 0);

        Vector vA = rbA ? rbA->GetPointVelocity(contactPoint) : Vector(0, 0);
        Vector vB = rbB ? rbB->GetPointVelocity(contactPoint) : Vector(0, 0);

        Vector relative = VectorMath::Subtract(vB, vA);
        float contactVel = VectorMath::Dot(relative, normal);
        if (contactVel > 0.0f) continue;

        float rAperpDotN = VectorMath::Dot(Vector(-rA.GetY(), rA.GetX()), normal);
        float rBperpDotN = VectorMath::Dot(Vector(-rB.GetY(), rB.GetX()), normal);

        float invInertiaA = rbA && rbA->GetInertia() > 0.0f ? 1.0f / rbA->GetInertia() : 0.0f;
        float invInertiaB = rbB && rbB->GetInertia() > 0.0f ? 1.0f / rbB->GetInertia() : 0.0f;

        float denom = invMassA + invMassB + rAperpDotN * rAperpDotN * invInertiaA + rBperpDotN * rBperpDotN * invInertiaB;
        if (denom == 0.0f) continue;

        float j = -(1.0f + restitution) * contactVel / denom / contactCount;
        if (fabs(j) < EPS_IMPULSE) continue;

        impulses.push_back(VectorMath::Multiply(normal, j));
        rA_list.push_back(rA);
        rB_list.push_back(rB);
    }

    for (size_t k = 0; k < impulses.size(); ++k) {
        Vector J = impulses[k];
        Vector rA = rA_list[k];
        Vector rB = rB_list[k];

        if (rbA) {
            rbA->ApplyVelocity(VectorMath::Multiply(J, -invMassA));
            if (rbA->GetInertia() > 0.0f)
                rbA->ApplyAngularVelocity(VectorMath::Cross(rA, VectorMath::Multiply(J, -1.0f)) / rbA->GetInertia());
        }
        if (rbB) {
            rbB->ApplyVelocity(VectorMath::Multiply(J, invMassB));
            if (rbB->GetInertia() > 0.0f)
                rbB->ApplyAngularVelocity(VectorMath::Cross(rB, J) / rbB->GetInertia());
        }
    }

    for (size_t k = 0; k < impulses.size(); ++k) {
        Vector rA = rA_list[k], rB = rB_list[k];

        Vector vA = rbA ? rbA->GetPointVelocity(VectorMath::Add(rbA->GetPosition(), rA)) : Vector(0, 0);
        Vector vB = rbB ? rbB->GetPointVelocity(VectorMath::Add(rbB->GetPosition(), rB)) : Vector(0, 0);

        Vector relative = VectorMath::Subtract(vB, vA);
        Vector tangent = VectorMath::Subtract(relative, VectorMath::Multiply(normal, VectorMath::Dot(relative, normal)));
        float len = VectorMath::Norm(tangent);
        if (len < 1e-6f) continue;
        tangent = VectorMath::Multiply(tangent, 1.0f / len);

        float rAperpDotT = VectorMath::Dot(Vector(-rA.GetY(), rA.GetX()), tangent);
        float rBperpDotT = VectorMath::Dot(Vector(-rB.GetY(), rB.GetX()), tangent);

        float invInertiaA = rbA && rbA->GetInertia() > 0.0f ? 1.0f / rbA->GetInertia() : 0.0f;
        float invInertiaB = rbB && rbB->GetInertia() > 0.0f ? 1.0f / rbB->GetInertia() : 0.0f;

        float denom = invMassA + invMassB + rAperpDotT * rAperpDotT * invInertiaA + rBperpDotT * rBperpDotT * invInertiaB;
        if (denom == 0.0f) continue;

        float jt = -VectorMath::Dot(relative, tangent) / denom / contactCount;
        float maxFriction = mu * VectorMath::Norm(impulses[k]);
        jt = VectorMath::Max(VectorMath::Min(jt, maxFriction), -maxFriction);
        if (fabs(jt) < EPS_IMPULSE) continue;

        Vector Jf = VectorMath::Multiply(tangent, jt);
        if (rbA) {
            rbA->ApplyVelocity(VectorMath::Multiply(Jf, -invMassA));
            if (rbA->GetInertia() > 0.0f)
                rbA->ApplyAngularVelocity(VectorMath::Cross(rA, VectorMath::Multiply(Jf, -1.0f)) / rbA->GetInertia());
        }
        if (rbB) {
            rbB->ApplyVelocity(VectorMath::Multiply(Jf, invMassB));
            if (rbB->GetInertia() > 0.0f)
                rbB->ApplyAngularVelocity(VectorMath::Cross(rB, Jf) / rbB->GetInertia());
        }
    }

    const float VELOCITY_EPS = 1e-3f;
    if (rbA) {
        if (VectorMath::Norm(rbA->GetVelocity()) < VELOCITY_EPS) rbA->SetVelocity(Vector(0, 0));
        if (fabs(rbA->GetAngularVelocity()) < VELOCITY_EPS) rbA->SetAngularVelocity(0.0f);
    }
    if (rbB) {
        if (VectorMath::Norm(rbB->GetVelocity()) < VELOCITY_EPS) rbB->SetVelocity(Vector(0, 0));
        if (fabs(rbB->GetAngularVelocity()) < VELOCITY_EPS) rbB->SetAngularVelocity(0.0f);
    }
}
