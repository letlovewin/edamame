/**
 * @author Gray
 * @details It's just gravity.py but ported to C++ for more efficiency.
 */

#pragma once

#include <iostream>
#include <climits>
#include <cmath>
#include <vector>
#include <array>
#include <memory>

using namespace std;

float G = 6.67e-11;
float SOFTENING_CONSTANT = 10;
float THETA = 0.5;

class Vector2
{
public:
    float x;
    float y;
    bool isEmpty;
    Vector2();
    Vector2(float px, float py);
    Vector2 &operator=(const Vector2 &v);
    friend bool operator==(const Vector2 &v1, const Vector2 &v2);
    friend bool operator!=(const Vector2 &v1, const Vector2 &v2);
    bool empty();
};

Vector2::Vector2()
{
    x = INT_MIN;
    y = INT_MIN;
    isEmpty = true;
}

Vector2::Vector2(float px, float py)
{
    x = px;
    y = py;
    isEmpty = false;
}

Vector2 &Vector2::operator=(const Vector2 &v)
{
    x = v.x;
    y = v.y;
    return *this;
}

bool operator!=(const Vector2 &v1, const Vector2 &v2)
{
    return v1.x != v2.x || v1.y != v2.y;
}

bool operator==(const Vector2 &v1, const Vector2 &v2)
{
    return v1.x == v2.x && v1.y == v2.y;
}

bool Vector2::empty()
{
    return isEmpty;
}

bool in(Vector2 &p, float x_interval[], float y_interval[])
{
    return p.x >= x_interval[0] && p.x <= x_interval[1] && p.y >= y_interval[0] && p.y <= y_interval[1];
}

class Quadrant
{
private:
    Vector2 origin;
    float length;

public:
    Quadrant();
    Quadrant(Vector2 &v_origin, float v_length);
    Quadrant &operator=(Quadrant &q);
    float getLength();
    bool contains(Vector2 &p);
    Quadrant NorthEast();
    Quadrant NorthWest();
    Quadrant SouthWest();
    Quadrant SouthEast();
};

Quadrant::Quadrant()
{
    origin = Vector2();
    length = 0;
}

Quadrant::Quadrant(Vector2 &v_origin, float v_length)
{
    origin = v_origin;
    length = v_length;
}

float Quadrant::getLength()
{
    return length;
}

Quadrant &Quadrant::operator=(Quadrant &q)
{
    origin.x = q.origin.x;
    origin.y = q.origin.y;
    length = q.getLength();
    return *this;
}

bool Quadrant::contains(Vector2 &p)
{
    float x_interval[2] = {origin.x, origin.x + getLength()};
    float y_interval[2] = {origin.y, origin.y + getLength()};
    return in(p, x_interval, y_interval);
}

Quadrant Quadrant::NorthEast()
{
    Vector2 new_origin(origin.x + getLength() / 2, origin.y + getLength() / 2);
    return Quadrant(new_origin, getLength() / 2);
}

Quadrant Quadrant::NorthWest()
{
    Vector2 new_origin(origin.x, origin.y + getLength() / 2);
    return Quadrant(new_origin, getLength() / 2);
}

Quadrant Quadrant::SouthWest()
{
    Vector2 new_origin(origin.x, origin.y);
    return Quadrant(new_origin, getLength() / 2);
}

Quadrant Quadrant::SouthEast()
{
    Vector2 new_origin(origin.x + getLength() / 2, origin.y);
    return Quadrant(new_origin, getLength() / 2);
}

class Particle
{
private:
    Vector2 position, velocity;
    float mass;

public:
    Particle();
    Particle(Vector2 &p, Vector2 &v, float m);
    bool in(Quadrant &quadrant);
    bool isNull();
    float getMass();
    Vector2 getPosition();
    Vector2 getVelocity();
    void setPosition(Vector2 &p);
    void setVelocity(Vector2 &v);
};

Particle::Particle()
{
    mass = 0;
}

Particle::Particle(Vector2 &p, Vector2 &v, float m)
{
    position = p;
    velocity = v;
    mass = m;
}

bool Particle::in(Quadrant &quadrant)
{
    return quadrant.contains(position);
}

Vector2 Particle::getPosition()
{
    return position;
}

Vector2 Particle::getVelocity()
{
    return velocity;
}

void Particle::setPosition(Vector2 &p)
{
    position.x = p.x;
    position.y = p.y;
}

void Particle::setVelocity(Vector2 &v)
{
    velocity.x = v.x;
    velocity.y = v.y;
}

bool Particle::isNull()
{
    return position.empty() && velocity.empty();
}

float Particle::getMass()
{
    return mass;
}

class BarnesHutTree
{
private:
    Quadrant quadrant;
    shared_ptr<Particle> particle = nullptr;

    shared_ptr<BarnesHutTree> northWest = nullptr;
    shared_ptr<BarnesHutTree> northEast = nullptr;
    shared_ptr<BarnesHutTree> southWest = nullptr;
    shared_ptr<BarnesHutTree> southEast = nullptr;

public:
    BarnesHutTree();
    BarnesHutTree(Quadrant &q);
    bool empty();
    bool null();
    Quadrant getQuadrant();
    vector<shared_ptr<BarnesHutTree>> getChildren();
    float getTotalMass(float mass);
    float getYMoment(float yMoment);
    float getXMoment(float xMoment);
    Vector2 getCOM();
    void insert(Particle &p);
    Vector2 calculateForce(Particle &p, Vector2 &F_t);
};

BarnesHutTree::BarnesHutTree()
{
    particle = nullptr;
}

BarnesHutTree::BarnesHutTree(Quadrant &q)
{
    quadrant = q;
}

bool BarnesHutTree::null()
{
    return particle == nullptr;
}

bool BarnesHutTree::empty()
{
    return northWest == nullptr && northEast == nullptr && southWest == nullptr && southEast == nullptr;
}

Quadrant BarnesHutTree::getQuadrant()
{
    return quadrant;
}

vector<shared_ptr<BarnesHutTree>> BarnesHutTree::getChildren()
{
    vector<shared_ptr<BarnesHutTree>> children;
    if (northWest != nullptr)
        children.push_back(northWest);
    if (northEast != nullptr)
        children.push_back(northEast);
    if (southWest != nullptr)
        children.push_back(southWest);
    if (southEast != nullptr)
        children.push_back(southEast);
    return children;
}

float BarnesHutTree::getTotalMass(float mass = 0)
{
    if (empty() && !null())
        return particle->getMass();
    if (empty() && null())
        return 0;
    float temp_mass = 0;
    for (shared_ptr<BarnesHutTree> &subtree : getChildren())
    {
        temp_mass += subtree->getTotalMass(mass);
    }
    return mass + temp_mass;
}

float BarnesHutTree::getYMoment(float yMoment = 0)
{
    if (empty() && !null())
        return particle->getMass() * particle->getPosition().x;
    if (empty() && null())
        return 0;
    float temp_moment = 0;
    for (shared_ptr<BarnesHutTree> &subtree : getChildren())
    {
        temp_moment += subtree->getYMoment(yMoment);
    }
    return yMoment + temp_moment;
}

float BarnesHutTree::getXMoment(float xMoment = 0)
{
    if (empty() && null())
        return 0;
    if (empty() && !null())
        return particle->getMass() * particle->getPosition().y;
    float temp_moment = 0;
    for (shared_ptr<BarnesHutTree> &subtree : getChildren())
    {
        temp_moment += subtree->getXMoment(xMoment);
    }
    return xMoment + temp_moment;
}

void BarnesHutTree::insert(Particle &p)
{
    if (null() && empty())
    {
        particle = unique_ptr<Particle>(&p);
        return;
    }
    else if (!null())
    {
        Vector2 p_pos = p.getPosition();
        Vector2 pa_pos = particle->getPosition();
        if (empty())
        {
            Quadrant NorthWest = getQuadrant().NorthWest();
            Quadrant NorthEast = getQuadrant().NorthEast();
            Quadrant SouthWest = getQuadrant().SouthWest();
            Quadrant SouthEast = getQuadrant().SouthEast();

            shared_ptr<BarnesHutTree> newNorthWestBHT(new BarnesHutTree());
            shared_ptr<BarnesHutTree> newNorthEastBHT(new BarnesHutTree());
            shared_ptr<BarnesHutTree> newSouthWestBHT(new BarnesHutTree());
            shared_ptr<BarnesHutTree> newSouthEastBHT(new BarnesHutTree());
            northWest = newNorthWestBHT;
            northEast = newNorthEastBHT;
            southWest = newSouthWestBHT;
            southEast = newSouthEastBHT;

            for (shared_ptr<BarnesHutTree> &subtree : getChildren())
            {

                if (subtree->getQuadrant().contains(p_pos))
                {
                    subtree->insert(p);
                    break;
                }
            }

            for (shared_ptr<BarnesHutTree> &subtree : getChildren())
            {
                if (subtree->getQuadrant().contains(pa_pos))
                {
                    subtree->insert(*particle);
                    break;
                }
            }

            particle = nullptr;
            return;
        }
        for (shared_ptr<BarnesHutTree> &subtree : getChildren())
        {
            if (!subtree->null())
            {

                if (subtree->getQuadrant().contains(p_pos))
                {
                    subtree->insert(p);
                    return;
                }
            }
        }
        return;
    }
    return;
}

Vector2 BarnesHutTree::getCOM()
{
    float Mx = getXMoment();
    float My = getYMoment();
    float Tm = getTotalMass();

    Vector2 COM(0, 0);
    COM.x = My / Tm;
    COM.y = Mx / Tm;

    return COM;
}

Vector2 ZERO_VECTOR(0, 0);

Vector2 BarnesHutTree::calculateForce(Particle &p, Vector2 &F_t = ZERO_VECTOR)
{
    if (empty() && !null())
    {
        Vector2 p_pos = particle->getPosition();
        if (p_pos == p.getPosition())
            return Vector2(0, 0);

        float particle_x = particle->getPosition().x;
        float particle_y = particle->getPosition().y;
        float particle_mass = particle->getMass();

        float px = p.getPosition().x;
        float py = p.getPosition().y;
        float pmass = p.getMass();

        float dx = particle_x - px;
        float dy = particle_y - py;

        float r = sqrtf(pow(dx, 2) + pow(dy, 2));
        float F = G * pmass * particle_mass / (pow(r, 2) + pow(SOFTENING_CONSTANT, 2));

        return Vector2(F * dx / r, F * dy / r);
    }

    if (empty() || null())
        return Vector2(0, 0);

    Vector2 COM = getCOM();
    float s = getQuadrant().getLength();

    Vector2 pos = particle->getPosition();
    float particle_x = pos.x;
    float particle_y = pos.y;
    float particle_mass = particle->getMass();

    float dx = COM.x - particle_x;
    float dy = COM.y - particle_y;

    float d = sqrtf(pow(dx, 2) + pow(dy, 2));
    if (s / d < THETA)
    {
        float total_mass = getTotalMass();
        float F = G * total_mass * particle_mass / (pow(d, 2) + pow(SOFTENING_CONSTANT, 2));
        return Vector2(F * dx / d, F * dy / d);
    }
    else
    {
        Vector2 totalForce_temp(0, 0);

        for (shared_ptr<BarnesHutTree> &subtree : getChildren())
        {
            Vector2 subtree_force = subtree->calculateForce(p, F_t);
            totalForce_temp.x += subtree_force.x;
            totalForce_temp.y += subtree_force.y;
        }

        return totalForce_temp;
    }
}

class Universe
{
private:
    float start_time, end_time, step_size, radius;
    bool running;
    vector<Particle> children;

public:
    Universe();
    Universe(float a, float b, float dt, float r);
    vector<Particle> getChildren();
    void addChild(Particle &p);
    void start();
    void stop();
};

Universe::Universe()
{
}

Universe::Universe(float a, float b, float dt, float r)
{
    start_time = a;
    end_time = b;
    step_size = dt;
    radius = r;
    running = false;
}

vector<Particle> Universe::getChildren()
{
    return children;
}

void Universe::addChild(Particle &p)
{
    children.push_back(p);
}

void Universe::start()
{
    if (running)
    {
        cout << "Simulation is already running.\n";
        return;
    }
    running = true;
    while (start_time < end_time)
    {
        Vector2 origin_vector(0, 0);
        Quadrant q(origin_vector, radius);
        BarnesHutTree BHT(q);
        for (Particle &p : getChildren())
        {
            Vector2 position = p.getPosition();
            if (q.contains(position))
                BHT.insert(p);
        }

        for (Particle &p : getChildren())
        {
            float mass = p.getMass();

            Vector2 F = BHT.calculateForce(p);

            unique_ptr<Vector2> a(new Vector2(F.x / mass, F.y / mass));
            Vector2 v_plus(p.getVelocity().x + step_size * a->x, p.getVelocity().y + step_size * a->y);
            p.setVelocity(v_plus);
            Vector2 p_plus(p.getPosition().x + step_size * v_plus.x, p.getPosition().y + step_size * v_plus.y);
            p.setPosition(p_plus);
        }
        start_time++;
    }
    running = false;

    cout << "Output\n";

    for (Particle &p : getChildren())
    {
        cout << "Position at time " << end_time << ": (" << p.getPosition().x << ", " << p.getPosition().y << ")" << endl;
        cout << "Velocity at time " << end_time << ": (" << p.getVelocity().y << ", " << p.getVelocity().y << ")" << endl;
    }
}
