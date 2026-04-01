#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
using namespace std;

//Constants
float dt = 0.0167f;
float gravity = 9.81f;
float groundLevel = 800.0f;
float rBound = 800.0f;
float restitution = 0.8f;

//Structs
struct Node {
    float mass;
    sf::Vector2f vel;
    sf::Vector2f pos;
};

struct Spring {
    float k;
    float d;
    float damp;
    Node* n1;
    Node* n2;
};

void updateSprings(vector<Spring>& springs) {
    for (Spring& sp : springs) {
        Node& n1 = *sp.n1;
        Node& n2 = *sp.n2;

        //Determine delta vector
        sf::Vector2f delta = n2.pos - n1.pos;
        float dst = sqrtf(delta.x * delta.x + delta.y * delta.y);
        if (dst == 0) continue;

        sf::Vector2f normDir = { delta.x / dst, delta.y / dst };

        float dx = dst - sp.d;
        float f = dx * -sp.k;
        sf::Vector2f springForce = { normDir.x * f, normDir.y * f };
        sf::Vector2f relativeVel = n2.vel - n1.vel;
        float velAlongDir = relativeVel.x * normDir.x + relativeVel.y * normDir.y;
        float dampForceMag = -sp.damp * velAlongDir;
        sf::Vector2f dampForce = { normDir.x * dampForceMag, normDir.y * dampForceMag };

        sf::Vector2f totalForce = springForce + dampForce;

        //Add acceleration
        n1.vel.x += -totalForce.x / n1.mass * dt;
        n1.vel.y += -totalForce.y / n1.mass * dt;
        n2.vel.x += totalForce.x / n2.mass * dt;
        n2.vel.y += totalForce.y / n2.mass * dt;
        
        //Update position
        n1.pos.x += n1.vel.x * dt;
        n1.pos.y += n1.vel.y * dt;
        n2.pos.x += n2.vel.x * dt;
        n2.pos.y += n2.vel.y * dt;
    }
}

void updateNodes(vector<Node*>& nodes) {
    for (Node* n : nodes) {
        n->vel.y += gravity * 10 * dt;
        n->pos.y += n->vel.y * dt;

        if (n->pos.y < 0) {
            n->pos.y = 0;
            n->vel.y = -n->vel.y * restitution;
        }
        else if (n->pos.y > groundLevel) {
            n->pos.y = groundLevel;
            n->vel.y = -n->vel.y * restitution;
        }

        if (n->pos.x < 0) {
            n->pos.x = 0;
            n->vel.x = -n->vel.x * restitution;
        }
        else if (n->pos.y > rBound) {
            n->pos.x = rBound;
            n->vel.x = -n->vel.x * restitution;
        }
    }
}

void simulate(vector<Node*>& nodes, vector<Spring>& springs) {
    updateSprings(springs);
    updateNodes(nodes);
}

void createRect(vector<Node*>& nodes, vector<Spring>& springs, sf::Vector2f topLeft, float width, float height, float k, float damp) {
    Node* n1 = new Node{ 1.0f, {0.0f, 0.0f}, topLeft };
    Node* n2 = new Node{ 1.0f, {0.0f, 0.0f}, {topLeft.x + width, topLeft.y} };
    Node* n3 = new Node{ 1.0f, {0.0f, 0.0f}, {topLeft.x, topLeft.y + height} };
    Node* n4 = new Node{ 1.0f, {0.0f, 0.0f}, {topLeft.x + width, topLeft.y + height} };
    nodes.push_back(n1);
    nodes.push_back(n2);
    nodes.push_back(n3);
    nodes.push_back(n4);

    float dHorizontal = width;
    float dVertical = height;
    float dDiagonal = sqrtf(width * width + height * height);

    springs.push_back(Spring{ k, dHorizontal, damp, n1, n2 });
    springs.push_back(Spring{ k, dHorizontal, damp, n3, n4 });
    springs.push_back(Spring{ k, dVertical, damp, n1, n3 });
    springs.push_back(Spring{ k, dVertical, damp, n2, n4 });
    springs.push_back(Spring{ k, dDiagonal, damp, n1, n4 });
    springs.push_back(Spring{ k, dDiagonal, damp, n2, n3 });
}

void createCircle(vector<Node*>& nodes, vector<Spring>& springs, sf::Vector2f center, float radius, int numPoints, float k, float damp) {
    Node* cntr = new Node{ 1.0f, {0.0f, 0.0f}, center };
    nodes.push_back(cntr);
    float dRad = 6.28f / numPoints;
    float arcLen = radius * dRad;
    for (int i = 0; i < numPoints; i++) {
        float lX = radius * cosf(i * dRad);
        float lY = radius * sinf(i * dRad);
        Node* tN = new Node{ 1.0f, {0.0f, 0.0f}, {center.x + lX, center.y + lY} };
        nodes.push_back(tN);
        springs.push_back(Spring{ k, radius, damp, cntr, tN });
        if (i != 0) springs.push_back(Spring{ k, arcLen, damp, nodes.at(nodes.size() - 2), tN });
    }
    springs.push_back(Spring{ k, radius, damp, nodes.at(nodes.size() - numPoints), nodes.at(nodes.size() - 1) });
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(rBound, groundLevel), "Soft-Body");
    window.setFramerateLimit(60);
    vector<Spring> springs;
    vector<Node*> nodes;

    createRect(nodes, springs, { 650.0f, 100.0f }, 100.0f, 100.0f, 40.0f, 0.25f);
    createCircle(nodes, springs, { 200.0f, 700.0f }, 50.0f, 8, 40.0f, 0.25f);

    //Drawing
    sf::CircleShape ball;
    sf::VertexArray line(sf::Lines, 2);
    ball.setFillColor(sf::Color::Green);
    ball.setRadius(5);

    while (window.isOpen())
    {
        //Events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        simulate(nodes, springs);

        //Rendering
        window.clear();
        for (Spring& sp : springs) {
            line[0] = sp.n1->pos;
            line[1] = sp.n2->pos;
            window.draw(line);
            ball.setPosition(sf::Vector2f(sp.n1->pos.x - ball.getRadius(), sp.n1->pos.y - ball.getRadius()));
            window.draw(ball);
            ball.setPosition(sf::Vector2f(sp.n2->pos.x - ball.getRadius(), sp.n2->pos.y - ball.getRadius()));
            window.draw(ball);
        }
        window.display();
    }

    return 0;
}