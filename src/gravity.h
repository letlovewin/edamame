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
    Particle &operator=(Particle &p);
    friend bool operator==(Particle &p1, Particle &p2);
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
}

Particle::Particle(Vector2 &p, Vector2 &v, float m)
{
    position = p;
    velocity = v;
    mass = m;
}

Particle &Particle::operator=(Particle &p)
{
    if (this != &p)
    {
        position = p.position;
        velocity = p.velocity;
        mass = p.mass;
    }

    return *this;
}

bool operator==(Particle &p1, Particle &p2)
{
    return p1.getVelocity() == p2.getVelocity() && p1.getPosition() == p2.getPosition();
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

Vector2 NULL_VECTOR = Vector2(INT_MIN, INT_MAX);

Particle PLACEHOLDER = Particle(NULL_VECTOR, NULL_VECTOR, INT_MIN);

class BarnesHutTree
{
private:
    Quadrant quadrant;
    unique_ptr<Particle> particle;

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
    void insert(unique_ptr<Particle> &p);
    Vector2 calculateForce(Vector2 p, Vector2 v, float m, Vector2 &F_t);
};

BarnesHutTree::BarnesHutTree()
{
}

BarnesHutTree::BarnesHutTree(Quadrant &q)
{
    quadrant = q;
    particle = nullptr;
    shared_ptr<BarnesHutTree> northWest = nullptr;
    shared_ptr<BarnesHutTree> northEast = nullptr;
    shared_ptr<BarnesHutTree> southWest = nullptr;
    shared_ptr<BarnesHutTree> southEast = nullptr;
}

bool BarnesHutTree::null()
{
    return particle == nullptr;
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

bool BarnesHutTree::empty()
{
    return getChildren().size() == 0;
}

float BarnesHutTree::getTotalMass(float mass)
{
    if (empty() && !null())
    {
        return particle->getMass();
    }

    if (empty() && null())
        return mass;
    float temp_mass = 0;
    for (shared_ptr<BarnesHutTree> &subtree : getChildren())
    {
        if (subtree != nullptr)
        {
            temp_mass += subtree->getTotalMass(mass);
        }
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

void BarnesHutTree::insert(unique_ptr<Particle> &p)
{
    // cout << "null: " << null() << endl;
    // cout << "empty: " << empty() << endl;
    if (null() && empty())
    {
        particle = move(p);
        return;
    }
    else if (null() && !empty())
    {
        Vector2 p_pos = p->getPosition();

        Quadrant NorthWest = getQuadrant().NorthWest();
        Quadrant NorthEast = getQuadrant().NorthEast();
        Quadrant SouthWest = getQuadrant().SouthWest();
        Quadrant SouthEast = getQuadrant().SouthEast();

        shared_ptr<BarnesHutTree> newNorthWestBHT(new BarnesHutTree(NorthWest));
        shared_ptr<BarnesHutTree> newNorthEastBHT(new BarnesHutTree(NorthEast));
        shared_ptr<BarnesHutTree> newSouthWestBHT(new BarnesHutTree(SouthWest));
        shared_ptr<BarnesHutTree> newSouthEastBHT(new BarnesHutTree(SouthEast));

        if (NorthWest.contains(p_pos))
        {
            if (northWest == nullptr)
                northWest = newNorthWestBHT;
            northWest->insert(p);
        }
        else if (NorthEast.contains(p_pos))
        {
            if (northEast == nullptr)
                northEast = newNorthEastBHT;
            northEast->insert(p);
        }
        else if (SouthWest.contains(p_pos))
        {
            if (southWest == nullptr)
                southWest = newSouthWestBHT;
            southWest->insert(p);
        }
        else if (SouthEast.contains(p_pos))
        {
            if (southEast == nullptr)
                southEast = newSouthEastBHT;
            southEast->insert(p);
        }
    }
    else if (!null())
    {
        Vector2 p_pos = p->getPosition();
        Vector2 pa_pos = particle->getPosition();
        Quadrant NorthWest = getQuadrant().NorthWest();
        Quadrant NorthEast = getQuadrant().NorthEast();
        Quadrant SouthWest = getQuadrant().SouthWest();
        Quadrant SouthEast = getQuadrant().SouthEast();

        shared_ptr<BarnesHutTree> newNorthWestBHT(new BarnesHutTree(NorthWest));
        shared_ptr<BarnesHutTree> newNorthEastBHT(new BarnesHutTree(NorthEast));
        shared_ptr<BarnesHutTree> newSouthWestBHT(new BarnesHutTree(SouthWest));
        shared_ptr<BarnesHutTree> newSouthEastBHT(new BarnesHutTree(SouthEast));

        if (NorthWest.contains(p_pos))
        {
            northWest = newNorthWestBHT;
            northWest->insert(p);
        }
        else if (NorthEast.contains(p_pos))
        {
            northEast = newNorthEastBHT;
            northEast->insert(p);
        }
        else if (SouthWest.contains(p_pos))
        {
            southWest = newSouthWestBHT;
            southWest->insert(p);
        }
        else if (SouthEast.contains(p_pos))
        {
            southEast = newSouthEastBHT;
            southEast->insert(p);
        }
        unique_ptr<Particle> particle_transfer = move(particle);
        particle = nullptr;
        if (NorthWest.contains(pa_pos))
        {
            if (northWest == nullptr)
                northWest = newNorthWestBHT;
            northWest->insert(particle_transfer);
        }
        else if (NorthEast.contains(pa_pos))
        {
            if (northEast == nullptr)
                northEast = newNorthEastBHT;
            northEast->insert(particle_transfer);
        }
        else if (SouthWest.contains(pa_pos))
        {
            if (southWest == nullptr)
                southWest = newSouthWestBHT;
            southWest->insert(particle_transfer);
        }
        else if (SouthEast.contains(pa_pos))
        {
            if (southEast == nullptr)
                southEast = newSouthEastBHT;
            southEast->insert(particle_transfer);
        }

        particle = nullptr;
        return;
    }
    return;
}

Vector2 BarnesHutTree::getCOM()
{
    float Mx = getXMoment();
    float My = getYMoment();
    float Tm = getTotalMass(0);

    Vector2 COM(0, 0);
    COM.x = My / Tm;
    COM.y = Mx / Tm;

    return COM;
}

Vector2 ZERO_VECTOR(0, 0);

Vector2 BarnesHutTree::calculateForce(Vector2 p, Vector2 v, float m, Vector2 &F_t = ZERO_VECTOR)
{
    if (empty() && !null())
    {
        Vector2 p_pos = particle->getPosition();
        if(p_pos == p && v == particle->getVelocity()) {
            return Vector2(0,0);
        }
        float particle_x = particle->getPosition().x;
        float particle_y = particle->getPosition().y;
        float particle_mass = particle->getMass();

        float px = p.x;
        float py = p.y;
        float pmass = m;

        float dx = particle_x - px;
        float dy = particle_y - py;

        float r = sqrtf(pow(dx, 2) + pow(dy, 2));
        float F = G * pmass * particle_mass / (pow(r, 2) + pow(SOFTENING_CONSTANT, 2));
        return Vector2(F * dx / r, F * dy / r);
    }
    else if (!empty() && null())
    {

        Vector2 COM = getCOM();

        float s = getQuadrant().getLength();

        float particle_x = p.x;
        float particle_y = p.y;
        float particle_mass = m;

        float dx = COM.x - particle_x;
        float dy = COM.y - particle_y;

        float d = sqrtf(pow(dx, 2) + pow(dy, 2));
        if (s / d < THETA)
        {

            float total_mass = getTotalMass(0);
            float F = G * total_mass * particle_mass / (pow(d, 2) + pow(SOFTENING_CONSTANT, 2));
            return Vector2(F * dx / d, F * dy / d);
        }
        else
        {
            Vector2 totalForce_temp(0, 0);

            for (shared_ptr<BarnesHutTree> &subtree : getChildren())
            {
                if (subtree != nullptr)
                {
                    Vector2 subtree_force = subtree->calculateForce(p, v, m, F_t);
                    totalForce_temp.x += subtree_force.x;
                    totalForce_temp.y += subtree_force.y;
                }
            }

            return Vector2(totalForce_temp.x + F_t.x, totalForce_temp.y + F_t.y);
        }
    }
    else
    {
        return Vector2(0, 0);
    }
}

class Universe
{
private:
    float start_time, end_time, step_size, radius;
    bool running;
    vector<unique_ptr<Particle>> children;

public:
    Universe();
    Universe(float a, float b, float dt, float r);
    vector<unique_ptr<Particle>> getChildren();
    void addChild(unique_ptr<Particle> &p);
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

vector<unique_ptr<Particle>> Universe::getChildren()
{
    return children;
}

void Universe::addChild(unique_ptr<Particle> &p)
{
    children.push_back(make_unique<Particle>(p));
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
        for (unique_ptr<Particle> &p : getChildren())
        {
            Vector2 position = p->getPosition();
            if (q.contains(position))
                BHT.insert(p);
        }

        for (unique_ptr<Particle> &p : getChildren())
        {
            float mass = p->getMass();
            Vector2 position = p->getPosition();
            Vector2 velocity = p->getVelocity();
            Vector2 F = BHT.calculateForce(position,velocity,mass);
            cout << "(" << F.x << ", " << F.y << ")" << endl;
            unique_ptr<Vector2> a(new Vector2(F.x / mass, F.y / mass));
            Vector2 v_plus(p->getVelocity().x + step_size * a->x, p->getVelocity().y + step_size * a->y);
            p->setVelocity(v_plus);
            Vector2 p_plus(p->getPosition().x + step_size * v_plus.x, p->getPosition().y + step_size * v_plus.y);
            p->setPosition(p_plus);
        }
        start_time += step_size;
    }
    running = false;

    cout << "Output\n";

    for (unique_ptr<Particle> &p : getChildren())
    {
        cout << "Position at time " << end_time << ": (" << p->getPosition().x << ", " << p->getPosition().y << ")" << endl;
        cout << "Velocity at time " << end_time << ": (" << p->getVelocity().x << ", " << p->getVelocity().y << ")" << endl;
    }
}