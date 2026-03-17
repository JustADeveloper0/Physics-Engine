#pragma once

#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include <vector>
#include "Vector.h"
#include "Body.h"

class PHYSICS_API StaticBody : public Body {
public:
	StaticBody(Vector position_, const std::vector<Vector>& points_, float rotation_, float coefficientOfFriction_ = 0.4f); // pointer to not copy and save memory, plus const it so no changes are made

	StaticBody(Vector position_, float radius_x, float radius_y, float rotation_, float coefficientOfFriction_ = 0.4f);

	~StaticBody() override;
};
