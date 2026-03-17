#include "pch.h"
#include "StaticBody.h"
#include <iostream>

StaticBody::StaticBody(Vector position_, const std::vector<Vector>& points_, float rotation_, float coefficientOfFriction_) :
	Body(position_, points_, rotation_, Body::BodyTypes::StaticBody, coefficientOfFriction_) {}

StaticBody::StaticBody(Vector position_, float radius_x, float radius_y, float rotation_, float coefficientOfFriction_) :
	Body(position_, radius_x, radius_y, rotation_, Body::BodyTypes::StaticBody, coefficientOfFriction_) {}

StaticBody::~StaticBody() = default;
