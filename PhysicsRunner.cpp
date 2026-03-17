#include <iostream>
#include <SDL.h>

#include <string>

#include "Graphics.h"

#include "VectorMath.h"
#include "Vector.h"
#include "StaticBody.h"
#include "RigidBody.h"
#include "World.h"
#include "Spring.h"
#include "Rope.h"

int main(int argc, char* argv[]) {
    enum Bodies {
        CircleBody,
        RectangleBody
    };

    enum Connections {
        RopeConnection,
        SpringConnection,
        HingeConnection
    };

	srand(static_cast<unsigned int>(time(nullptr))); // seed random number generator

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow(
        "Physics Tester",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WindowSize::WindowWidth,
        WindowSize::WindowHeight,
        SDL_WINDOW_SHOWN
    );

    if (!window) {
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer) {
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bool running = true;
    SDL_Event event;

    Vector gravity(0, 98);
    World* world = new World(gravity, WindowSize::WindowWidth, WindowSize::WindowHeight);

    /*
    Vector p1(-50, -50);
    Vector p2(-50, 50);
    Vector p3(50, 50);
    Vector p4(50, -50);

    std::vector<Vector> points1 = { p1, p2, p3, p4 };

    RigidBody* body1 = new RigidBody(Vector(550, 300), points1, 1.0f, 1.0f);

    RigidBody* body2 = new RigidBody(Vector(300, 200), 50, 50, 0.0f, 1.0f);

    RigidBody* body3 = new RigidBody(Vector(350, 200), 50, 50, 0.0f, 1.0f);

    std::vector<Vector> points2 = { p1, p2, p3, p4 };

    RigidBody* body4 = new RigidBody(Vector(625, 100), points2, 0.0f, 1.0f); // always only checks y axis not x axis despite it being the best
    */

    std::vector<Vector> points3 = { Vector(-200, -25), Vector(-200, 25), Vector(200, 25), Vector(200, -25)};

    StaticBody* body5 = new StaticBody(Vector(400, 600), points3, 0.0f);

    std::vector<Vector> points4 = { Vector(75, 100), Vector(-200, 100), Vector(-75, -100),};

    StaticBody* body6 = new StaticBody(Vector(200, 200), points4, 0.0f);

	//std::vector<Vector> points5 = { Vector(-10, -75), Vector(-10, 75), Vector(10, 75), Vector(10, -75) };

	//RigidBody* body7 = new RigidBody(Vector(300, 0), points5, 0.0f, 1.0f); /////////////////////// UNCOMMENT FOR HINGES ///////////////////////

	//Hinge* hinge = new Hinge(body6, Vector(0, 100), body7, Vector(0, -75)); /////////////////////// UNCOMMENT FOR HINGES ///////////////////////

    /*
    world->AddBody(body1);
    world->AddBody(body2);
    world->AddBody(body3);
    world->AddBody(body4);
    */
    world->AddBody(body5);
    world->AddBody(body6);

	//world->AddBody(body7); /////////////////////// UNCOMMENT FOR HINGES ///////////////////////
	//world->AddConnection(hinge); /////////////////////// UNCOMMENT FOR HINGES ///////////////////////

    // WHEN BODIES FALL PAST Y = MAX HEIGHT ; DELETE (!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
    // DONT LEAVE POINTERS UNDELETED

    Uint32 lastTime = SDL_GetTicks();

    bool showInstructions = false;

    bool leftMouseDown = false;
	bool rightMouseDown = false;

	Body* connectionBody1 = nullptr;
	Body* connectionBody2 = nullptr;

	Vector connectionBody1RelativePosition(0, 0);
    Vector connectionBody2RelativePosition(0, 0);
    
    float frameTimeAccumulator = 0.0f;
    int frameCount = 0;
    float fps = 0.0f;
    float avgFrameTimeMilliseconds = 0.0f;

    bool middleMouseHeld = false;
    Body* dragBody = nullptr;

    float dragStrength = 2500.0f;
    float dragDamping = 0.85f;

	Bodies selectedBodyType = RectangleBody;
	Connections selectedConnectionType = RopeConnection;

    bool bHeld = false;
    bool cHeld = false;
    bool sHeld = false;

    while (running) {
        int mouseX, mouseY;
		SDL_GetMouseState(&mouseX, &mouseY);
        
        if (leftMouseDown) {
            int mouseX, mouseY;
            SDL_GetMouseState(&mouseX, &mouseY);

            bool hit = false;

            for (int i = 0; i < (int)world->GetBodies().size(); i++) {
                Body* body = world->GetBodies()[i];

                if (body->GetShapeType() == (int)Body::ShapeTypes::Polygon)
                    hit = PointInPolygon(*body, mouseX, mouseY);
                else if (body->GetShapeType() == (int)Body::ShapeTypes::Circle)
                    hit = PointInCircle(*body, mouseX, mouseY);

                if (hit) {
                    world->RemoveBodyAndConnections(body);
                    break;
                }
            }

            if (!hit) {
                Vector mousePos(mouseX, mouseY);
                RigidBody* newBody = nullptr;

                if (selectedBodyType == Bodies::CircleBody) {
                    int size = rand() % 26 + 25;
                    newBody = new RigidBody(mousePos, size, size, 0.0f, 1.0f);
                }
                else if (selectedBodyType == Bodies::RectangleBody) {
                    float halfWidth = (rand() % 26 + 25) / 2.0f;
                    float halfHeight = (rand() % 26 + 25) / 2.0f;

                    std::vector<Vector> points = {
                        Vector(-halfWidth, -halfHeight),
                        Vector(-halfWidth,  halfHeight),
                        Vector(halfWidth,  halfHeight),
                        Vector(halfWidth, -halfHeight)
                    };

                    newBody = new RigidBody(mousePos, points, 0.0f, 1.0f);
                }

                if (newBody) world->AddBody(newBody);
            }

            leftMouseDown = false;
        }

        Uint32 time = SDL_GetTicks();

        float deltaTime = (time - lastTime) / 1000.0f;
        lastTime = time;

        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                running = false;
            }

            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_b) bHeld = true;
                if (event.key.keysym.sym == SDLK_c) cHeld = true;
                if (event.key.keysym.sym == SDLK_s) sHeld = true;
                if (event.key.keysym.sym == SDLK_i) showInstructions = !showInstructions;
                if (event.key.keysym.sym == SDLK_l) {
					connectionBody1 = nullptr;
                    connectionBody1 = nullptr;

                    connectionBody1RelativePosition = Vector(0, 0);
                    connectionBody2RelativePosition = Vector(0, 0);
				}
            }

            if (event.type == SDL_KEYUP) {
                if (event.key.keysym.sym == SDLK_b) bHeld = false;
                if (event.key.keysym.sym == SDLK_c) cHeld = false;
                if (event.key.keysym.sym == SDLK_s) sHeld = false;
            }

            if (event.type == SDL_MOUSEWHEEL) {
                int direction = event.wheel.y > 0 ? 1 : -1;

                if (bHeld) {
                    int current = (int)selectedBodyType;
                    int count = 2;
                    current = (current + direction + count) % count;
                    selectedBodyType = (Bodies)current;
                }

                if (cHeld) {
                    int current = (int)selectedConnectionType;
                    int count = 3;
                    current = (current + direction + count) % count;
                    selectedConnectionType = (Connections)current;
                }

                if (sHeld) {
                    dragStrength += direction * 500.0f;

                    if (dragStrength < 0.0f) dragStrength = 0.0f;
                }
            }

            if (event.type == SDL_MOUSEBUTTONDOWN) {
				Vector mousePos(event.button.x, event.button.y);

                if (event.button.button == SDL_BUTTON_LEFT) {
					leftMouseDown = true;
                }

                if (event.button.button == SDL_BUTTON_RIGHT) {
					rightMouseDown = true;
                }

                if (event.button.button == SDL_BUTTON_MIDDLE) {
                    middleMouseHeld = true;
                }
            }

            if (event.type == SDL_MOUSEBUTTONUP) {
                if (event.button.button == SDL_BUTTON_MIDDLE) {
                    middleMouseHeld = false;
                    dragBody = nullptr;
                }
            }
        }

        if (rightMouseDown) {
            bool removedHinge = false;
            for (int i = 0; i < (int)world->GetConnections().size(); i++) {
                Hinge* hinge = dynamic_cast<Hinge*>(world->GetConnections()[i]);
                if (hinge && PointNearHinge(*hinge, mouseX, mouseY)) {
                    world->RemoveConnection(hinge);
                    removedHinge = true;
                    break;
                }
            }

            if (!removedHinge) {
                bool overBody = false;
                for (Body* body : world->GetBodies()) {
                    if (body->GetShapeType() == (int)Body::ShapeTypes::Polygon && PointInPolygon(*body, mouseX, mouseY)) {
                        overBody = true; break;
                    }
                    if (body->GetShapeType() == (int)Body::ShapeTypes::Circle && PointInCircle(*body, mouseX, mouseY)) {
                        overBody = true; break;
                    }
                }

                if (!overBody) {
                    for (int i = 0; i < (int)world->GetConnections().size(); i++) {
                        Connection* c = world->GetConnections()[i];

                        Rope* rope = dynamic_cast<Rope*>(c);
                        if (rope && PointNearRope(*rope, mouseX, mouseY)) {
                            world->RemoveConnection(rope); break;
                        }

                        Spring* spring = dynamic_cast<Spring*>(c);
                        if (spring && PointNearSpring(*spring, mouseX, mouseY)) {
                            world->RemoveConnection(spring); break;
                        }
                    }
                }
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        const std::vector<Vector>& contactPoints = world->GetContactPoints();

        //std::cout << contactPoints.size() << std::endl;

        for (int i = 0; i < world->GetBodies().size(); i++) {
            Body* body = world->GetBodies().at(i);

            if (!body) continue;

            switch (body->GetShapeType()) {
                case (int) Body::ShapeTypes::Polygon:
                    if (body->GetBodyType() == Body::BodyTypes::StaticBody)
                        DrawPolygon(renderer, *body, Colors::RED);
                    else
                        DrawPolygon(renderer, *body, Colors::WHITE);

                    break;
                case (int) Body::ShapeTypes::Circle:
                    if (body->GetBodyType() == Body::BodyTypes::StaticBody)
                        DrawCircle(renderer, *body, Colors::RED);
                    else {
                        DrawCircle(renderer, *body, Colors::WHITE);

                        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                        SDL_RenderDrawLine(renderer,
                            static_cast<int>(body->GetPosition().GetX()),
                            static_cast<int>(body->GetPosition().GetY()),
                            static_cast<int>(body->GetPosition().GetX() + body->GetRadiusX() * cosf(body->GetRotation())),
                            static_cast<int>(body->GetPosition().GetY() + body->GetRadiusY() * sinf(body->GetRotation()))
					    );
                    }

                    break;
            }

            if (middleMouseHeld && dragBody == nullptr && body->GetBodyType() == Body::BodyTypes::RigidBody) {
                bool hovered = false;

                if (body->GetShapeType() == (int)Body::ShapeTypes::Polygon)
                    hovered = PointInPolygon(*body, mouseX, mouseY);
                else if (body->GetShapeType() == (int)Body::ShapeTypes::Circle)
                    hovered = PointInCircle(*body, mouseX, mouseY);

                if (hovered) {
                    dragBody = body;
                }
            }

            /*
            for (const Vector& point : contactPoints) {
                SDL_Rect rect;

                int size = 4;

                rect.x = static_cast<int>(point.GetX()) - size / 2;
                rect.y = static_cast<int>(point.GetY()) - size / 2;
                rect.w = size;
                rect.h = size;

                SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
                SDL_RenderFillRect(renderer, &rect);
            }
            */

            if (body->GetBodyType() == Body::BodyTypes::RigidBody) {
                RigidBody* rigidBody = static_cast<RigidBody*>(body);

                if (!rigidBody) continue;

				std::vector<Vector>* contactList = const_cast<std::vector<Vector>*>(rigidBody->GetContactList());
                
				//std::cout << contactList->size() << std::endl;

                for (Vector& point : *contactList) {
                    SDL_Rect rect;
                    int size = 6;
                    rect.x = static_cast<int>(point.GetX()) - size / 2;
                    rect.y = static_cast<int>(point.GetY()) - size / 2;
                    rect.w = size;
                    rect.h = size;
                    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
                    SDL_RenderFillRect(renderer, &rect);
                    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                    SDL_RenderDrawRect(renderer, &rect);
				}
            }

            //staticBody->SetPosition(VectorMath::Add(staticBody->GetPosition(), Vector(0.01, 0)));

            switch (body->GetShapeType()) {
            case (int)Body::ShapeTypes::Polygon:
                if (PointInPolygon(*body, mouseX, mouseY)) {
                    renderTelemetry(renderer, *body, mouseX, mouseY);

                    if (rightMouseDown) {
                        if (connectionBody1 == nullptr) {
                            connectionBody1 = body;
							connectionBody1RelativePosition = VectorMath::Subtract(Vector(mouseX, mouseY), connectionBody1->GetPosition());
                        } 
                        else if (connectionBody2 == nullptr && body != connectionBody1) {
                            connectionBody2 = body;
                            connectionBody2RelativePosition = VectorMath::Subtract(Vector(mouseX, mouseY), connectionBody2->GetPosition());
                        }
                        else {
                            connectionBody1 = body;
                            connectionBody2 = nullptr;

                            connectionBody1RelativePosition = VectorMath::Subtract(Vector(mouseX, mouseY), connectionBody1->GetPosition());
                            connectionBody2RelativePosition = Vector(0, 0);
                        }
                    }
                }
                break;
            case (int) Body::ShapeTypes::Circle:
                if (PointInCircle(*body, mouseX, mouseY)) {
                    if (rightMouseDown) {
                        if (connectionBody1 == nullptr) {
                            connectionBody1 = body;
                            connectionBody1RelativePosition = VectorMath::Subtract(Vector(mouseX, mouseY), connectionBody1->GetPosition());
                        }
                        else if (connectionBody2 == nullptr && body != connectionBody1) {
                            connectionBody2 = body;
                            connectionBody2RelativePosition = VectorMath::Subtract(Vector(mouseX, mouseY), connectionBody2->GetPosition());
                        }
                        else {
                            connectionBody1 = body;
                            connectionBody2 = nullptr;
                            connectionBody1RelativePosition = VectorMath::Subtract(Vector(mouseX, mouseY), connectionBody1->GetPosition());
                            connectionBody2RelativePosition = Vector(0, 0);
                        }
                    }
                }
                break;
            }
        }

        if (connectionBody1 != nullptr && connectionBody2 != nullptr) {
            switch (selectedConnectionType) {
                case Connections::RopeConnection: {
                    Rope* rope = new Rope(connectionBody1, connectionBody1RelativePosition, connectionBody2, connectionBody2RelativePosition);
                    world->AddConnection(rope);

					break;
                }
                case Connections::SpringConnection: {
                    Spring* spring = new Spring(connectionBody1, connectionBody1RelativePosition, connectionBody2, connectionBody2RelativePosition);
                    world->AddConnection(spring);

                    break;
                }
                case Connections::HingeConnection: {
                    Hinge* hinge = new Hinge(connectionBody1, connectionBody1RelativePosition, connectionBody2, connectionBody2RelativePosition);
                    world->AddConnection(hinge);

                    break;
                }
            }

            connectionBody1 = nullptr;
            connectionBody2 = nullptr;

            connectionBody1RelativePosition = Vector(0, 0);
            connectionBody2RelativePosition = Vector(0, 0);
		}

        if (dragBody != nullptr) {
            bool stillExists = false;

            for (Body* body : world->GetBodies()) {
                if (body == dragBody) {
                    stillExists = true;
                    break;
                }
            }

            if (!stillExists) dragBody = nullptr;
        }

        if (middleMouseHeld && dragBody != nullptr && dragBody->GetBodyType() == Body::BodyTypes::RigidBody) {
            RigidBody* rigidBody = static_cast<RigidBody*>(dragBody);

            float deltaX = mouseX - dragBody->GetPosition().GetX();
            float deltaY = mouseY - dragBody->GetPosition().GetY();
            float dist = sqrtf(deltaX * deltaX + deltaY * deltaY);

            if (dist > 0.0f) {
                Vector springForce(deltaX * dragStrength, deltaY * dragStrength);
                rigidBody->ApplyForce(springForce);

                rigidBody->SetVelocity(Vector(
                    rigidBody->GetVelocity().GetX() * dragDamping,
                    rigidBody->GetVelocity().GetY() * dragDamping
                ));
            }

            SDL_SetRenderDrawColor(renderer, 0, 255, 100, 255);
            SDL_RenderDrawLine(renderer,
                (int)dragBody->GetPosition().GetX(),
                (int)dragBody->GetPosition().GetY(),
                mouseX, mouseY
            );
        }

        for (Connection* connection : world->GetConnections()) {
            Rope* rope = dynamic_cast<Rope*>(connection);
            if (rope) { DrawRope(renderer, *rope, Colors::YELLOW); continue; }

            Spring* spring = dynamic_cast<Spring*>(connection);
            if (spring) { DrawSpring(renderer, *spring, Colors::CYAN); continue; }

            Hinge* hinge = dynamic_cast<Hinge*>(connection);
            if (hinge) DrawHinge(renderer, *hinge, Colors::MAGENTA);
        }

        DrawText(renderer, "I: Toggle Instructions", 12, 12, Colors::WHITE, 2);
        
        std::string currentBodyText = "Current Shape: ";

        if (selectedBodyType == Bodies::CircleBody) currentBodyText += "Circle";
        else if (selectedBodyType == Bodies::RectangleBody) currentBodyText += "Rectangle";
		else currentBodyText += "None";

        DrawText(renderer, currentBodyText.c_str(), 12, 36, Colors::WHITE, 2);

        std::string currentConnectionText = "Current Connection: ";

        if (selectedConnectionType == Connections::RopeConnection) currentConnectionText += "Rope";
        else if (selectedConnectionType == Connections::SpringConnection) currentConnectionText += "Spring";
		else if (selectedConnectionType == Connections::HingeConnection) currentConnectionText += "Hinge";
        else currentConnectionText += "None";

        DrawText(renderer, currentConnectionText.c_str(), 12, 60, Colors::WHITE, 2);

        Uint32 totalFrameTicks = SDL_GetTicks() - time;

        frameTimeAccumulator += deltaTime;
        frameCount++;

        if (frameTimeAccumulator >= 0.5f) {
            fps = frameCount / frameTimeAccumulator;
            avgFrameTimeMilliseconds = (frameTimeAccumulator / frameCount) * 1000.0f;
            frameTimeAccumulator = 0.0f;
            frameCount = 0;
		}

        std::string specsText = "FPS: " + std::to_string((int)fps) + " | Avg: " + std::to_string(avgFrameTimeMilliseconds).substr(0, 4) + "ms";
        DrawText(renderer, specsText.c_str(), 12, 84, Colors::WHITE, 2);
        
        std::string bodiesText = "Body Count: " + std::to_string((int) world->GetBodies().size()) + " Bodies";
        DrawText(renderer, bodiesText.c_str(), 12, 108, Colors::WHITE, 2);

        std::string dragStrengthText = "Drag Strength: " + std::to_string((int) dragStrength) + " N/m";
        DrawText(renderer, dragStrengthText.c_str(), 12, 132, Colors::WHITE, 2);

        if (showInstructions) {
			RenderInstructions(renderer);
        }

        world->Update(deltaTime, 5);

        SDL_RenderPresent(renderer);

        Uint32 frameTime = SDL_GetTicks() - time;

        if (frameTime < FPS::TargetFrameTime) {
            SDL_Delay((Uint32)(FPS::TargetFrameTime - frameTime));
        }

        if (rightMouseDown) rightMouseDown = false;
    }
    
    for (Body* body : world->GetBodies())
        delete body;

    delete world;

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
