#pragma once
#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif
#include "Connection.h"

class PHYSICS_API Hinge : public Connection {
private:
    float compliance;

    const float rotationalDamping = 0.995f;

public:
    Hinge(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_, float compliance_ = 0.0f);
    virtual ~Hinge();

    void Update(float deltaTime, Vector gravity) override;
};
