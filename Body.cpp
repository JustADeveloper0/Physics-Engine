#include "pch.h"
#include <iostream>

#include "Body.h"
#include "VectorMath.h"
#include <algorithm>
#include "Constants.h"
#include "Collisions.h"

Body::Body(Vector& position_, const std::vector<Vector>& points_, float rotation_, int bodyType_, float coefficientOfFriction_) : rotation(rotation_), bodyType(bodyType_), coefficientOfFriction(coefficientOfFriction_) {
	shapeType = (int) ShapeTypes::Polygon;

	position = Vector(position_.GetX() * factor, position_.GetY() * factor);

	points.clear();
	points.reserve(points_.size());
	for (const Vector& p : points_) {
		points.emplace_back(p.GetX() * factor, p.GetY() * factor);
	}
}

Body::Body(Vector& position_, float radiusX_, float radiusY_, float rotation_, int bodyType_, float coefficientOfFriction_) : rotation(rotation_), bodyType(bodyType_), coefficientOfFriction(coefficientOfFriction_) {
	shapeType = (int) ShapeTypes::Circle;

	position = Vector(position_.GetX() * factor, position_.GetY() * factor);
	radiusX = radiusX_ * factor;
	radiusY = radiusY_ * factor;
}

Body::~Body() = default;

const Vector& Body::GetPosition() const {
	return position;
}

void Body::SetPosition(const Vector position_) {
	position = position_;

	globalPointsDirty = true;
	circlePointsDirty = true;
}

const std::vector<Vector>& Body::GetPoints() const { // make it compatible with many languages (!!!!!!!!!)
	return points;
}

const std::vector<Vector>& Body::GetGlobalPoints() const {
	if (!globalPointsDirty) return globalPoints;

	globalPoints.clear(); // clears but keeps memory
	globalPoints.reserve(points.size()); // avoids allocation (basically doubles have to double capacity when size == capacity)

	for (const Vector& point : points) {
		float x = point.GetX() * cosf(rotation) - point.GetY() * sinf(rotation);
		float y = point.GetX() * sinf(rotation) + point.GetY() * cosf(rotation);

		globalPoints.push_back(VectorMath::Add(Vector(x, y), position));
	}

	globalPointsDirty = false;

	return globalPoints;
}

const std::vector<Vector>& Body::GetCirclePoints() const {
	if (!circlePointsDirty) return cachedCirclePoints;

	cachedCirclePoints.clear();
	cachedCirclePoints.reserve(CircleDetectionAccuracy);

	for (int i = 0; i < CircleDetectionAccuracy; i++) {
		float theta = (2.0f * M_PI * i) / CircleDetectionAccuracy;

		float x = radiusX * cosf(theta);
		float y = radiusY * sinf(theta);

		float rotatedX = x * cosf(rotation) - y * sinf(rotation);
		float rotatedY = x * sinf(rotation) + y * cosf(rotation);

		cachedCirclePoints.emplace_back(rotatedX + position.GetX(), rotatedY + position.GetY());
	}

	circlePointsDirty = false;
	return cachedCirclePoints;
}

float Body::GetRadiusX() {
	return radiusX;
}

float Body::GetRadiusY() {
	return radiusY;
}

void Body::SetRadiusX(float radiusX_) {
	radiusX = radiusX_;

	circlePointsDirty = true;
}

void Body::SetRadiusY(float radiusY_) {
	radiusY = radiusY_;

	circlePointsDirty = true;
}

float Body::GetArea() {
	if (area < 0) {
		if (shapeType == Body::ShapeTypes::Circle)
			area = M_PI * radiusX * radiusY;

		if (shapeType == Body::ShapeTypes::Polygon) {
			float sum = 0.0f;

			for (int i = 0; i < points.size(); i++) {
				const Vector& p1 = points[i];
				const Vector& p2 = points[(i + 1) % points.size()];

				sum += VectorMath::Cross(p1, p2);
			}

			area = 0.5f * std::abs(sum);

		}

	}

	return area;
}

float Body::GetRotation() {
	return rotation;
}

void Body::SetRotation(float rotation_) {
	rotation = fmod(rotation_, (M_PI * 2.0));
	if (rotation < 0) rotation += (M_PI * 2.0);

	globalPointsDirty = true;
	circlePointsDirty = true;
}

int Body::GetShapeType() {
	return shapeType;
}

int Body::GetBodyType() {
	return bodyType;
}

float Body::getCoefficientOfFriction() const {
	return coefficientOfFriction;
}
