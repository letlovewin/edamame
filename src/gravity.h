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

class Quadrant
{
public:
    float length, x, y;
    Quadrant();
    Quadrant(float len, float ex, float ey);
    bool contains(float ex, float ey);
    Quadrant NW();
    Quadrant NE();
    Quadrant SW();
    Quadrant SE();
};

Quadrant::Quadrant()
{
}

Quadrant::Quadrant(float len, float ex, float ey)
{
    length = len;
    x = ex;
    y = ey;
}

bool Quadrant::contains(float ex, float ey)
{
    return ex >= x && ex <= x + length && ey >= y && ey <= y + length;
}

Quadrant Quadrant::NW()
{
    return Quadrant(length / 2, x, y + length / 2);
}

Quadrant Quadrant::NE()
{
    return Quadrant(length / 2, x + length / 2, y + length / 2);
}

Quadrant Quadrant::SW()
{
    return Quadrant(length / 2, x, y);
}

Quadrant Quadrant::SE()
{
    return Quadrant(length / 2, x + length / 2, y);
}

class Particle
{
public:
    float px, py, vx, vy, mass;
    Particle();
    Particle(float epx, float epy, float evx, float evy, float em);
    Particle &operator=(Particle &p);
};

Particle::Particle()
{
}

Particle::Particle(float epx, float epy, float evx, float evy, float em)
{
    px = epx;
    py = epy;
    vx = evx;
    vy = evy;
    mass = em;
};

Particle &Particle::operator=(Particle &p)
{
    px = p.px;
    py = p.py;
    vx = p.vx;
    vy = p.vy;
    mass = p.mass;
    return *this;
}

class BarnesHutTreeNode
{
public:
    shared_ptr<Particle> p;
    Quadrant q;
    shared_ptr<BarnesHutTreeNode> NE, NW, SW, SE;
    BarnesHutTreeNode();
    BarnesHutTreeNode(float x, float y, float length);
    BarnesHutTreeNode(Quadrant q);
    bool null();
    bool empty();
    void insert(Particle &ep);
    float getTotalMass(float tm);
    float getXMoment(float xm);
    float getYMoment(float ym);
    array<float, 2> getCOM();
    array<float, 2> netForce(float fx, float fy, Particle &pa);
};

BarnesHutTreeNode::BarnesHutTreeNode()
{
}

BarnesHutTreeNode::BarnesHutTreeNode(float x, float y, float length)
{
    q = Quadrant(length, x, y);
}

BarnesHutTreeNode::BarnesHutTreeNode(Quadrant quad)
{
    q = quad;
}

bool BarnesHutTreeNode::null()
{
    return p == nullptr;
}

bool BarnesHutTreeNode::empty()
{
    return NE == nullptr && NW == nullptr && SW == nullptr && SE == nullptr;
}

void BarnesHutTreeNode::insert(Particle &ep)
{
    if (null() && empty())
    {
        p = make_shared<Particle>(ep);
    }
    else if (!null() && empty())
    {
        Particle tp = *p; // Transferred the particle
        p = nullptr;      // Now the node is empty and null.

        Quadrant qNE = q.NE();
        Quadrant qNW = q.NW();
        Quadrant qSW = q.SW();
        Quadrant qSE = q.SE();

        if (qNE.contains(ep.px, ep.py))
        {
            NE = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qNE));
            NE->insert(ep);
        }
        else if (qNW.contains(ep.px, ep.py))
        {
            NW = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qNW));
            NW->insert(ep);
        }
        else if (qSW.contains(ep.px, ep.py))
        {
            SW = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qSW));
            SW->insert(ep);
        }
        else if (qSE.contains(ep.px, ep.py))
        {
            SE = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qSE));
            SE->insert(ep);
        }

        if (qNE.contains(tp.px, tp.py))
        {
            if (NE == nullptr)
                NE = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qNE));
            NE->insert(tp);
        }
        else if (qNW.contains(tp.px, tp.py))
        {
            if (NW == nullptr)
                NW = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qNW));
            NW->insert(tp);
        }
        else if (qSW.contains(tp.px, tp.py))
        {
            if (SW == nullptr)
                SW = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qSW));
            SW->insert(tp);
        }
        else if (qSE.contains(tp.px, tp.py))
        {
            if (SE == nullptr)
                SE = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qSE));
            SE->insert(tp);
        }

        return;
    }
    else if (null() && !empty())
    {
        Quadrant qNE = q.NE();
        Quadrant qNW = q.NW();
        Quadrant qSW = q.SW();
        Quadrant qSE = q.SE();

        if (qNE.contains(ep.px, ep.py))
        {
            if (NE == nullptr)
                NE = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qNE));
            NE->insert(ep);
        }
        else if (qNW.contains(ep.px, ep.py))
        {
            if (NW == nullptr)
                NW = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qNW));
            NW->insert(ep);
        }
        else if (qSW.contains(ep.px, ep.py))
        {
            if (SW == nullptr)
                SW = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qSW));
            SW->insert(ep);
        }
        else if (qSE.contains(ep.px, ep.py))
        {
            if (SE == nullptr)
                SE = shared_ptr<BarnesHutTreeNode>(new BarnesHutTreeNode(qSE));
            SE->insert(ep);
        }
        return;
    }
}

float BarnesHutTreeNode::getTotalMass(float tm)
{
    if (empty() && !null())
    {
        return p->mass;
    }
    else if (null() && !empty())
    {
        float temp = 0;
        if (NW != nullptr)
        {
            temp += NW->getTotalMass(tm);
        }
        if (NE != nullptr)
        {
            temp += NE->getTotalMass(tm);
        }
        if (SW != nullptr)
        {
            temp += SW->getTotalMass(tm);
        }
        if (SE != nullptr)
        {
            temp += SE->getTotalMass(tm);
        }
        return tm + temp;
    }
    else if (null() && empty())
    {
        return 0;
    }
}

float BarnesHutTreeNode::getXMoment(float xm)
{
    if (empty() && !null())
    {
        return p->mass * p->py;
    }
    else if (null() && !empty())
    {
        float temp = 0;
        if (NW != nullptr)
        {
            temp += NW->getXMoment(xm);
        }
        if (NE != nullptr)
        {
            temp += NE->getXMoment(xm);
        }
        if (SW != nullptr)
        {
            temp += SW->getXMoment(xm);
        }
        if (SE != nullptr)
        {
            temp += SE->getXMoment(xm);
        }
        return xm + temp;
    }
    else if (null() && empty())
    {
        return 0;
    }
}

float BarnesHutTreeNode::getYMoment(float ym)
{
    if (empty() && !null())
    {
        return p->mass * p->px;
    }
    else if (null() && !empty())
    {
        float temp = 0;
        if (NW != nullptr)
        {
            temp += NW->getYMoment(ym);
        }
        if (NE != nullptr)
        {
            temp += NE->getYMoment(ym);
        }
        if (SW != nullptr)
        {
            temp += SW->getYMoment(ym);
        }
        if (SE != nullptr)
        {
            temp += SE->getYMoment(ym);
        }
        return ym + temp;
    }
    else if (null() && empty())
    {
        return 0;
    }
}

array<float, 2> BarnesHutTreeNode::getCOM()
{
    float xm = getXMoment(0);
    float ym = getYMoment(0);
    float tm = getTotalMass(0);
    return array<float, 2>({ym / tm, xm / tm});
}

array<float, 2> BarnesHutTreeNode::netForce(float fx, float fy, Particle &pa)
{
    if (empty() && !null())
    {
        if (pa.px == p->px && pa.py == p->py && pa.vx == p->vx && pa.vy == p->vy)
            return array<float, 2>({0, 0});
        float dx = p->px - pa.px;
        float dy = p->py - pa.py;
        float r = sqrt(pow(dx, 2) + pow(dy, 2));
        float F = G * p->mass * pa.mass / pow(r, 2);
        return array<float, 2>({F * dx / r, F * dy / r});
    }
    else if (null() && !empty())
    {
        array<float, 2> COM = getCOM();
        float Tm = getTotalMass(0);
        float s = q.length;
        float dx = COM[0] - pa.px;
        float dy = COM[1] - pa.py;
        float d = sqrt(pow(dx, 2) + pow(dy, 2));
        if (s / d < THETA)
        {
            float F = G * Tm * pa.mass / pow(d, 2);
            float r = sqrt(pow(dx, 2) + pow(dy, 2));
            return array<float, 2>({F * dx / r, F * dy / r});
        }
        else
        {
            float tfx, tfy;
            if (NE != nullptr)
            {
                array<float, 2> NE_f = NE->netForce(fx, fy, pa);
                tfx += NE_f[0];
                tfy += NE_f[1];
            }
            if (NW != nullptr)
            {
                array<float, 2> T_f = NW->netForce(fx, fy, pa);
                tfx += T_f[0];
                tfy += T_f[1];
            }
            if (SW != nullptr)
            {
                array<float, 2> T_f = SW->netForce(fx, fy, pa);
                tfx += T_f[0];
                tfy += T_f[1];
            }
            if (SE != nullptr)
            {
                array<float, 2> T_f = SE->netForce(fx, fy, pa);
                tfx += T_f[0];
                tfy += T_f[1];
            }
            return array<float, 2>({fx + tfx, fy + tfy});
        }
    }
    else if (null() && empty())
    {
        return array<float, 2>({0, 0});
    }
}