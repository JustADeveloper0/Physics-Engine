#include <SDL.h>
#include <math.h>
#include <vector>
#include <algorithm>

#include "Graphics.h"
#include "StaticBody.h"
#include "Vector.h"
#include <string>
#include "VectorMath.h"

#include "../DynamicPhysicsEngine/Constants.h"

const unsigned char font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x5F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x08, 0x2A, 0x1C, 0x2A, 0x08},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x00, 0x08, 0x14, 0x22, 0x41},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x41, 0x22, 0x14, 0x08, 0x00},
    {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x41, 0x51, 0x32},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x04, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x03, 0x04, 0x78, 0x04, 0x03},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20}, 
    {0x00, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00},
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x48, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x08, 0x54, 0x54, 0x54, 0x3C},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
    {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x08, 0x08, 0x2A, 0x1C, 0x08},
};

Color Colors::WHITE = Color{ 255, 255, 255 };
Color Colors::BLACK = Color{ 0, 0, 0 };
Color Colors::RED = Color{ 255, 0, 0 };
Color Colors::GREEN = Color{ 0, 255, 0 };
Color Colors::BLUE = Color{ 0, 0, 255 };
Color Colors::YELLOW = Color{ 255, 255, 0 };
Color Colors::CYAN = Color{ 0, 255, 255 };
Color Colors::MAGENTA = Color{ 255, 0, 255 };
Color Colors::ORANGE = Color{ 255, 165, 0 };
Color Colors::PURPLE = Color{ 128, 0, 128 };
Color Colors::PINK = Color{ 255, 192, 203 };
Color Colors::GRAY = Color{ 128, 128, 128 };
Color Colors::LIGHTGRAY = Color{ 200,200,200 };
Color Colors::DARKGRAY = Color{ 50,50,50 };
Color Colors::BROWN = Color{ 165,42,42 };

static inline int wx(float worldCoord) { return static_cast<int>(worldCoord / factor); }
static inline float wxf(float worldCoord) { return (worldCoord / factor); }

void DrawPolygon(SDL_Renderer* renderer, Body& body, Color& color) {
    std::vector<Vector> points = body.GetPoints();

    Vector position = body.GetPosition();
    float rotation = body.GetRotation();

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    for (int i = 0; i < points.size(); i++) {
        Vector point = points[i];
        Vector future_point;

        if (i == points.size() - 1)
            future_point = points[0];
        else
            future_point = points[i + 1];

        float x1 = position.GetX() + point.GetX() * cosf(rotation) - point.GetY() * sinf(rotation);
        float y1 = position.GetY() + point.GetX() * sinf(rotation) + point.GetY() * cosf(rotation);

        float x2 = position.GetX() + future_point.GetX() * cosf(rotation) - future_point.GetY() * sinf(rotation);
        float y2 = position.GetY() + future_point.GetX() * sinf(rotation) + future_point.GetY() * cosf(rotation);

        SDL_RenderDrawLine(renderer, wx(x1), wx(y1), wx(x2), wx(y2));
    }
}

void DrawCircle(SDL_Renderer* renderer, Body& body, Color& color) {
    float radius_x = body.GetRadiusX();
    float radius_y = body.GetRadiusY();

    Vector position = body.GetPosition();
    float rotation = body.GetRotation();

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    int screenRadiusX = static_cast<int>(radius_x / factor);
    int screenRadiusY = static_cast<int>(radius_y / factor);
    int screenCenterX = wx(position.GetX());
    int screenCenterY = wx(position.GetY());

    for (int i = 0; i < Resolutions::CircleResolution; ++i) {
        float theta1 = 2.0f * M_PI * i / Resolutions::CircleResolution;
        float theta2 = 2.0f * M_PI * (i + 1) / Resolutions::CircleResolution;

        float x1 = radius_x * cosf(theta1);
        float y1 = radius_y * sinf(theta1);
        float x2 = radius_x * cosf(theta2);
        float y2 = radius_y * sinf(theta2);

        float rotatedX1 = x1 * cos(rotation) - y1 * sin(rotation) + position.GetX();
        float rotatedY1 = x1 * sin(rotation) + y1 * cos(rotation) + position.GetY();
        float rotatedX2 = x2 * cos(rotation) - y2 * sin(rotation) + position.GetX();
        float rotatedY2 = x2 * sin(rotation) + y2 * cos(rotation) + position.GetY();

        SDL_RenderDrawLine(renderer, wx(rotatedX1), wx(rotatedY1), wx(rotatedX2), wx(rotatedY2));
    }
}

void DrawFilledPolygon(SDL_Renderer* renderer, Body& body, Color& color) {
    std::vector<Vector> points = body.GetPoints();

    if (points.size() < 3) return;

    Vector position = body.GetPosition();
    float rotation = body.GetRotation();

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    std::vector<SDL_Point> rotatedPoints;
    rotatedPoints.reserve(points.size());

    for (const Vector& point : points) {
        float x = point.GetX();
        float y = point.GetY();
        float rotatedX = x * cos(rotation) - y * sin(rotation) + position.GetX();
        float rotatedY = x * sin(rotation) + y * cos(rotation) + position.GetY();
        rotatedPoints.push_back({ wx(rotatedX), wx(rotatedY) });
    }

    int yMin = rotatedPoints[0].y;
    int yMax = rotatedPoints[0].y;
    for (const auto& p : rotatedPoints) {
        if (p.y < yMin) yMin = p.y;
        if (p.y > yMax) yMax = p.y;
    }

    for (int y = yMin; y <= yMax; y += std::max(1, static_cast<int>(Resolutions::FilledPolygonResolution))) {
        std::vector<int> intersections;
        int prevIndex = rotatedPoints.size() - 1;
        for (int i = 0; i < (int)rotatedPoints.size(); ++i) {
            int currentY = rotatedPoints[i].y;
            int previousY = rotatedPoints[prevIndex].y;
            int currentX = rotatedPoints[i].x;
            int previousX = rotatedPoints[prevIndex].x;

            if ((currentY < y && previousY >= y) || (previousY < y && currentY >= y)) {
                float t = float(y - currentY) / float(previousY - currentY);
                int interX = static_cast<int>(currentX + t * (previousX - currentX));
                intersections.push_back(interX);
            }
            prevIndex = i;
        }

        std::sort(intersections.begin(), intersections.end());
        for (int i = 0; i + 1 < (int)intersections.size(); i += 2) {
            int startX = intersections[i];
            int endX = intersections[i + 1];
            SDL_RenderDrawLine(renderer, startX, y, endX, y);
        }
    }
}

void DrawFilledCircle(SDL_Renderer* renderer, Body& body, Color& color) {
    float radiusX = body.GetRadiusX();
    float radiusY = body.GetRadiusY();

    Vector position = body.GetPosition();
    float rotation = body.GetRotation();

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    std::vector<SDL_Point> ellipsePoints;
    ellipsePoints.reserve(Resolutions::CircleResolution);

    for (int i = 0; i < Resolutions::CircleResolution; ++i) {
        float t = 2.0f * M_PI * i / Resolutions::CircleResolution;
        float x = radiusX * cosf(t);
        float y = radiusY * sinf(t);

        float rotatedX = cos(rotation) * x - sin(rotation) * y + position.GetX();
        float rotatedY = sin(rotation) * x + cos(rotation) * y + position.GetY();

        ellipsePoints.push_back({ wx(rotatedX), wx(rotatedY) });
    }

    int yMin = ellipsePoints[0].y;
    int yMax = ellipsePoints[0].y;
    for (const auto& p : ellipsePoints) {
        if (p.y < yMin) yMin = p.y;
        if (p.y > yMax) yMax = p.y;
    }

    for (int y = yMin; y <= yMax; ++y) {
        std::vector<int> intersections;
        int prevIndex = ellipsePoints.size() - 1;
        for (int i = 0; i < (int)ellipsePoints.size(); ++i) {
            int x1 = ellipsePoints[prevIndex].x;
            int y1 = ellipsePoints[prevIndex].y;
            int x2 = ellipsePoints[i].x;
            int y2 = ellipsePoints[i].y;

            if ((y1 <= y && y2 > y) || (y2 <= y && y1 > y)) {
                float t = float(y - y1) / float(y2 - y1);
                int interX = static_cast<int>(x1 + t * (x2 - x1));
                intersections.push_back(interX);
            }
            prevIndex = i;
        }
        std::sort(intersections.begin(), intersections.end());
        for (int i = 0; i + 1 < (int)intersections.size(); i += 2) {
            SDL_RenderDrawLine(renderer, intersections[i], y, intersections[i + 1], y);
        }
    }
}

void DrawText(SDL_Renderer* renderer, const char* text, int x, int y, Color& color, int scale, bool centered) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    int textLength = 0;
    for (int i = 0; text[i] != '\0'; i++) {
        unsigned char c = (unsigned char)text[i];
        if (c >= 32 && c <= 126)
            textLength++;
    }
    int textWidth = textLength * 6 * scale;
    int textHeight = 7 * scale;

    int startX = x;
    int startY = y;
    if (centered) {
        startX -= textWidth / 2;
        startY -= textHeight / 2;
    }

    int drawX = startX;
    for (int i = 0; text[i] != '\0'; i++) {
        unsigned char c = (unsigned char)text[i];
        if (c < 32 || c > 126) {
            drawX += 6 * scale;
            continue;
        }

        for (int col = 0; col < 5; col++) {
            unsigned char byte = font5x7[c - 32][col];
            for (int row = 0; row < 7; row++) {
                if (byte & (1 << row)) {
                    SDL_Rect pixel = { drawX + col * scale, startY + row * scale, scale, scale };
                    SDL_RenderFillRect(renderer, &pixel);
                }
            }
        }
        drawX += 6 * scale;
    }
}

void renderTelemetry(SDL_Renderer* renderer, Body& body, int mouseX, int mouseY) {
    const int lineHeight = 10;
    const int padding = 5;
    const int lines = 9;
    const int boxWidth = 200;
    const int boxHeight = lines * lineHeight + padding * 2;

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_RenderDrawLine(renderer, mouseX, mouseY, mouseX, mouseY - boxHeight);
    SDL_RenderDrawLine(renderer, mouseX, mouseY - boxHeight, mouseX + boxWidth, mouseY - boxHeight);
    SDL_RenderDrawLine(renderer, mouseX + boxWidth, mouseY - boxHeight, mouseX + boxWidth, mouseY);
    SDL_RenderDrawLine(renderer, mouseX + boxWidth, mouseY, mouseX, mouseY);

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

    char buf[128];
    int tx = mouseX + padding;
    int ty = mouseY - boxHeight + padding;

    DrawText(renderer, "Body Telemetry:", tx, ty, Colors::RED, 1);
    ty += lineHeight;

    snprintf(buf, sizeof(buf), "Pos: (%.3f, %.3f)", body.GetPosition().GetX() / factor, body.GetPosition().GetY() / factor);
    DrawText(renderer, buf, tx, ty, Colors::RED, 1); ty += lineHeight;

    snprintf(buf, sizeof(buf), "Rot: %.3f rad  %.3f deg", body.GetRotation(), body.GetRotation() * (180.0f / M_PI));
    DrawText(renderer, buf, tx, ty, Colors::RED, 1); ty += lineHeight;
    
    snprintf(buf, sizeof(buf), "Area: %.3f", body.GetArea() / (factor * factor));
    DrawText(renderer, buf, tx, ty, Colors::RED, 1); ty += lineHeight;

    const char* shape = body.GetShapeType() == Body::ShapeTypes::Polygon ? "Polygon" : "Circle";
    snprintf(buf, sizeof(buf), "Shape: %s", shape);
    DrawText(renderer, buf, tx, ty, Colors::RED, 1); ty += lineHeight;

    const char* type = body.GetBodyType() == Body::BodyTypes::StaticBody ? "Static" : "Rigid";
    snprintf(buf, sizeof(buf), "Type: %s", type);
    DrawText(renderer, buf, tx, ty, Colors::RED, 1); ty += lineHeight;

    if (body.GetShapeType() == Body::ShapeTypes::Circle) {
        snprintf(buf, sizeof(buf), "RadiusX: %.3f  RadiusY: %.3f", body.GetRadiusX() / factor, body.GetRadiusY() / factor);
        DrawText(renderer, buf, tx, ty, Colors::RED, 1); ty += lineHeight;
    }
    else {
        snprintf(buf, sizeof(buf), "Vertices: %d", (int)body.GetPoints().size());
        DrawText(renderer, buf, tx, ty,Colors::RED, 1); ty += lineHeight;
    }
}

void RenderInstructions(SDL_Renderer* renderer) {
    int size = 2;
    int textHeight = 12 * size;

    const char* instructions[] = {
        "Instructions:",
        "Left Click: Create / Delete Body",
        "Right Click: Create / Delete Connection",
        "Middle Click: Move Body",
        "B + Scroll: Change Body Type",
        "C + Scroll: Change Connection Type",
		"S + Scroll: Change Drag Strength",
        "L: Remove Selected Rope Bodies"
        "I: Toggle Instructions",
    };

    size_t numInstructions = sizeof(instructions) / sizeof(instructions[0]);

    int x = (WindowSize::WindowWidth / 2);
    int y = (WindowSize::WindowHeight / 2) - ((numInstructions / 2) * textHeight);

    for (const char* line : instructions) {
        DrawText(renderer, line, x, y, Colors::WHITE, size, true);
        y += textHeight;
    }
}

bool PointInPolygon(Body& body, int mouseX, int mouseY) {
    float worldX = mouseX * factor;
    float worldY = mouseY * factor;

    float relativePosX = worldX - body.GetPosition().GetX();
    float relativePosY = worldY - body.GetPosition().GetY();

    float angle = -body.GetRotation();

    float localX = relativePosX * cosf(angle) - relativePosY * sinf(angle);
    float localY = relativePosX * sinf(angle) + relativePosY * cosf(angle);

    const std::vector<Vector>& localPoints = body.GetPoints();
    bool inside = false;
    int n = localPoints.size();

    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((localPoints[i].GetY() > localY) != (localPoints[j].GetY() > localY)) &&
            (localX < (localPoints[j].GetX() - localPoints[i].GetX()) * (localY - localPoints[i].GetY()) /
                (localPoints[j].GetY() - localPoints[i].GetY()) + localPoints[i].GetX())) {
            inside = !inside;
        }
    }
    return inside;
}

bool PointInCircle(Body& body, int mouseX, int mouseY) {
    float worldX = mouseX * factor;
    float worldY = mouseY * factor;

    Vector position = body.GetPosition();

    float angle = -body.GetRotation();
    float rx = body.GetRadiusX();
    float ry = body.GetRadiusY();

    float dx = worldX - position.GetX();
    float dy = worldY - position.GetY();

    float localX = dx * cosf(angle) - dy * sinf(angle);
    float localY = dx * sinf(angle) + dy * cosf(angle);

    float result = (localX * localX) / (rx * rx) + (localY * localY) / (ry * ry);

    return result <= 1.0f;
}

void DrawRope(SDL_Renderer* renderer, Rope& rope, Color& color) {
    const std::vector<Rope::RopeNode>& nodes = rope.GetNodes();
    if (nodes.size() < 2) return;

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    for (int i = 0; i < (int)nodes.size() - 1; i++) {
        SDL_RenderDrawLine(renderer,
            wx(nodes[i].position.GetX()),
            wx(nodes[i].position.GetY()),
            wx(nodes[i + 1].position.GetX()),
            wx(nodes[i + 1].position.GetY())
        );
    }

    for (int i = 0; i < (int) nodes.size(); i++) {
        if (nodes[i].inverseMass == 0.0f) {
            SDL_Rect rect;
            int size = 6;
            rect.x = wx(nodes[i].position.GetX()) - size / 2;
            rect.y = wx(nodes[i].position.GetY()) - size / 2;
            rect.w = size;
            rect.h = size;
            SDL_SetRenderDrawColor(renderer, 255, 100, 0, 255);
            SDL_RenderFillRect(renderer, &rect);
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        }
    }
}

void DrawSpring(SDL_Renderer* renderer, Spring& spring, Color& color) {
    Body* b1 = const_cast<Body*>(spring.GetBody1());
    Body* b2 = const_cast<Body*>(spring.GetBody2());

    float rot1 = b1->GetRotation();
    Vector r1 = spring.GetBody1RelativePosition();
    Vector worldR1(
        r1.GetX() * cosf(rot1) - r1.GetY() * sinf(rot1),
        r1.GetX() * sinf(rot1) + r1.GetY() * cosf(rot1)
    );

    float rot2 = b2->GetRotation();
    Vector r2 = spring.GetBody2RelativePosition();
    Vector worldR2(
        r2.GetX() * cosf(rot2) - r2.GetY() * sinf(rot2),
        r2.GetX() * sinf(rot2) + r2.GetY() * cosf(rot2)
    );

    Vector anchor1 = VectorMath::Add(b1->GetPosition(), worldR1);
    Vector anchor2 = VectorMath::Add(b2->GetPosition(), worldR2);

    const int segments = 12;
    const float zigzagWidth = 6.0f;

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    float dx = anchor2.GetX() - anchor1.GetX();
    float dy = anchor2.GetY() - anchor1.GetY();
    float len = sqrtf(dx * dx + dy * dy);
    if (len == 0.0f) return;

    float perpX = -dy / len;
    float perpY = dx / len;

    Vector prev = anchor1;

    for (int i = 1; i <= segments; i++) {
        float t = (float)i / segments;
        float baseX = anchor1.GetX() + t * dx;
        float baseY = anchor1.GetY() + t * dy;

        float offset = (i == 0 || i == segments) ? 0.0f :
            ((i % 2 == 0) ? zigzagWidth : -zigzagWidth);

        Vector next(baseX + perpX * offset, baseY + perpY * offset);

        SDL_RenderDrawLine(renderer,
            wx(prev.GetX()), wx(prev.GetY()),
            wx(next.GetX()), wx(next.GetY())
        );

        prev = next;
    }

    for (int k = 0; k < 2; k++) {
        Vector& anchor = (k == 0) ? anchor1 : anchor2;
        SDL_Rect rect;
        int size = 6;
        rect.x = wx(anchor.GetX()) - size / 2;
        rect.y = wx(anchor.GetY()) - size / 2;
        rect.w = size;
        rect.h = size;
        SDL_SetRenderDrawColor(renderer, 255, 100, 0, 255);
        SDL_RenderFillRect(renderer, &rect);
    }
}

void DrawHinge(SDL_Renderer* renderer, Hinge& hinge, Color& color) {
    Body* b1 = const_cast<Body*>(hinge.GetBody1());
    Body* b2 = const_cast<Body*>(hinge.GetBody2());

    float rot1 = b1->GetRotation();
    Vector r1 = hinge.GetBody1RelativePosition();
    Vector worldR1(
        r1.GetX() * cosf(rot1) - r1.GetY() * sinf(rot1),
        r1.GetX() * sinf(rot1) + r1.GetY() * cosf(rot1)
    );

    float rot2 = b2->GetRotation();
    Vector r2 = hinge.GetBody2RelativePosition();
    Vector worldR2(
        r2.GetX() * cosf(rot2) - r2.GetY() * sinf(rot2),
        r2.GetX() * sinf(rot2) + r2.GetY() * cosf(rot2)
    );

    Vector anchor1 = VectorMath::Add(b1->GetPosition(), worldR1);
    Vector anchor2 = VectorMath::Add(b2->GetPosition(), worldR2);

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLine(renderer,
        wx(b1->GetPosition().GetX()), wx(b1->GetPosition().GetY()),
        wx(anchor1.GetX()), wx(anchor1.GetY())
    );

    SDL_RenderDrawLine(renderer,
        wx(b2->GetPosition().GetX()), wx(b2->GetPosition().GetY()),
        wx(anchor2.GetX()), wx(anchor2.GetY())
    );

    Vector mid(
        (anchor1.GetX() + anchor2.GetX()) * 0.5f,
        (anchor1.GetY() + anchor2.GetY()) * 0.5f
    );

    int cx = wx(mid.GetX());
    int cy = wx(mid.GetY());
    int r = 5;

    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r * r) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    for (int i = 0; i < 32; i++) {
        float t1 = 2.0f * M_PI * i / 32;
        float t2 = 2.0f * M_PI * (i + 1) / 32;
        SDL_RenderDrawLine(renderer,
            cx + (int)(r * cosf(t1)), cy + (int)(r * sinf(t1)),
            cx + (int)(r * cosf(t2)), cy + (int)(r * sinf(t2))
        );
    }
}

bool PointNearRope(Rope& rope, int mouseX, int mouseY, float threshold) {
    const std::vector<Rope::RopeNode>& nodes = rope.GetNodes();

    for (int i = 0; i < (int)nodes.size() - 1; i++) {
        float ax = nodes[i].position.GetX();
        float ay = nodes[i].position.GetY();
        float bx = nodes[i + 1].position.GetX();
        float by = nodes[i + 1].position.GetY();

        float dx = bx - ax;
        float dy = by - ay;
        float lenSq = dx * dx + dy * dy;

        if (lenSq == 0.0f) continue;

        float t = ((mouseX - ax) * dx + (mouseY - ay) * dy) / lenSq;
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

        float closestX = ax + t * dx;
        float closestY = ay + t * dy;

        float distSq = (mouseX - closestX) * (mouseX - closestX) + (mouseY - closestY) * (mouseY - closestY);

        if (distSq <= threshold * threshold) return true;
    }

    return false;
}

bool PointNearSpring(Spring& spring, int mouseX, int mouseY, float threshold) {
    Body* b1 = const_cast<Body*>(spring.GetBody1());
    Body* b2 = const_cast<Body*>(spring.GetBody2());

    float rot1 = b1->GetRotation();
    Vector r1 = spring.GetBody1RelativePosition();
    Vector worldR1(r1.GetX() * cosf(rot1) - r1.GetY() * sinf(rot1), r1.GetX() * sinf(rot1) + r1.GetY() * cosf(rot1));

    float rot2 = b2->GetRotation();
    Vector r2 = spring.GetBody2RelativePosition();
    Vector worldR2(r2.GetX() * cosf(rot2) - r2.GetY() * sinf(rot2), r2.GetX() * sinf(rot2) + r2.GetY() * cosf(rot2));

    float ax = VectorMath::Add(b1->GetPosition(), worldR1).GetX();
    float ay = VectorMath::Add(b1->GetPosition(), worldR1).GetY();
    float bx = VectorMath::Add(b2->GetPosition(), worldR2).GetX();
    float by = VectorMath::Add(b2->GetPosition(), worldR2).GetY();

    float dx = bx - ax, dy = by - ay;
    float lenSq = dx * dx + dy * dy;
    if (lenSq == 0.0f) return false;

    float t = ((mouseX - ax) * dx + (mouseY - ay) * dy) / lenSq;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

    float cx = ax + t * dx, cy = ay + t * dy;
    float distSq = (mouseX - cx) * (mouseX - cx) + (mouseY - cy) * (mouseY - cy);
    return distSq <= threshold * threshold;
}

bool PointNearHinge(Hinge& hinge, int mouseX, int mouseY, float threshold) {
    Body* b1 = const_cast<Body*>(hinge.GetBody1());
    Body* b2 = const_cast<Body*>(hinge.GetBody2());

    float rot1 = b1->GetRotation();
    Vector r1 = hinge.GetBody1RelativePosition();
    Vector worldR1(
        r1.GetX() * cosf(rot1) - r1.GetY() * sinf(rot1),
        r1.GetX() * sinf(rot1) + r1.GetY() * cosf(rot1)
    );

    float rot2 = b2->GetRotation();
    Vector r2 = hinge.GetBody2RelativePosition();
    Vector worldR2(
        r2.GetX() * cosf(rot2) - r2.GetY() * sinf(rot2),
        r2.GetX() * sinf(rot2) + r2.GetY() * cosf(rot2)
    );

    Vector anchor1 = VectorMath::Add(b1->GetPosition(), worldR1);
    Vector anchor2 = VectorMath::Add(b2->GetPosition(), worldR2);

    Vector mid(
        (anchor1.GetX() + anchor2.GetX()) * 0.5f,
        (anchor1.GetY() + anchor2.GetY()) * 0.5f
    );

    float dx = mouseX - wx(mid.GetX());
    float dy = mouseY - wx(mid.GetY());
    return (dx * dx + dy * dy) <= threshold * threshold;
}
