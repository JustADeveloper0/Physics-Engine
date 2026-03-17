#pragma once

#include "Body.h"
#include "Spring.h"
#include "Rope.h"
#include "Hinge.h"

extern const unsigned char font5x7[95][5];

struct WindowSize {
    static constexpr int WindowWidth = 800;
    static constexpr int WindowHeight = 800;
};

struct FPS {
    static constexpr int TargetFPS = 60;
    static constexpr float TargetFrameTime = 1000.0f / TargetFPS;
};

struct Resolutions {
    static constexpr int CircleResolution = 100;
    static constexpr float FilledPolygonResolution = 1.0f;
    static constexpr float FilledCircleResolution = 1.0f;
};

struct Color {
    int r, g, b, a;

    Color() : r(255), g(255), b(255), a(255) {}
    Color(int red, int green, int blue, float alpha = 255) : r(red), g(green), b(blue), a(alpha) {}
};

struct Colors {
    static Color WHITE;
    static Color BLACK;
    static Color RED;
    static Color GREEN;
    static Color BLUE;
    static Color YELLOW;
    static Color CYAN;
    static Color MAGENTA;
    static Color ORANGE;
    static Color PURPLE;
    static Color PINK;
    static Color GRAY;
    static Color LIGHTGRAY;
    static Color DARKGRAY;
    static Color BROWN;
};

void DrawPolygon(SDL_Renderer* renderer, Body& body, Color& color);

void DrawCircle(SDL_Renderer* renderer, Body& body, Color& color);

void DrawFilledPolygon(SDL_Renderer* renderer, Body& body, Color& color);

void DrawFilledCircle(SDL_Renderer* renderer, Body& body, Color& color);

void DrawText(SDL_Renderer* renderer, const char* text, int x, int y, Color& color, int scale = 1, bool centered = false);

void renderTelemetry(SDL_Renderer* renderer, Body& body, int mouseX, int mouseY);

void RenderInstructions(SDL_Renderer* renderer);

bool PointInPolygon(Body& body, int mouseX, int mouseY);

bool PointInCircle(Body& body, int mouseX, int mouseY);

void DrawRope(SDL_Renderer* renderer, Rope& rope, Color& color);

void DrawSpring(SDL_Renderer* renderer, Spring& spring, Color& color);

void DrawHinge(SDL_Renderer* renderer, Hinge& hinge, Color& color);

bool PointNearRope(Rope& rope, int mouseX, int mouseY, float threshold = 10.0f);

bool PointNearSpring(Spring& spring, int mouseX, int mouseY, float threshold = 10.0f);

bool PointNearHinge(Hinge& hinge, int mouseX, int mouseY, float threshold = 15.0f);
