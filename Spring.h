#pragma once
#ifdef EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif
#include "Connection.h"

class PHYSICS_API Spring : public Connection {
private:
    float restLength;
    float stiffness; // k shi
    float damping;

public:
    Spring(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_, float stiffness_ = 200.0f, float damping_ = 5.0f);

    virtual ~Spring();

    float GetRestLength() const;

    float GetStiffness() const;

    void Update(float deltaTime, Vector gravity) override;
};
