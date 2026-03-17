#include "pch.h"
#include "Connection.h"

Connection::Connection(Body* body1_, Vector body1RelativePosition_, Body* body2_, Vector body2RelativePosition_)
	: body1(body1_), body2(body2_), body1RelativePosition(body1RelativePosition_), body2RelativePosition(body2RelativePosition_) {
}

Connection::~Connection() = default;

const Body* Connection::GetBody1() const {
	return body1;
}

const Body* Connection::GetBody2() const {
	return body2;
}

const Vector Connection::GetBody1RelativePosition() const {
	return body1RelativePosition;
}

const Vector Connection::GetBody2RelativePosition() const {
	return body2RelativePosition;
}

void Connection::Update(float deltaTime, Vector gravity) {}
