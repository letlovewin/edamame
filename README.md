# Edamame 

An N-body simulation library written in C++ with Barnes-Hut trees.

## Usage

Everything in this library relies on the `Vector2` class. To initialize a new `Vector2`, write:

```cpp
Vector2 *vec = new Vector2(x1,x2);
```

where `x1` and `x2` are `float`.

To establish new particles in your simulation, write:

```cpp
Particle *p = new Particle(position,velocity,mass);
```

where `position` and `velocity` are `Vector2` and `mass` is a `float`.

Next, to set up your simulation, you create a `Universe` object:

```cpp
Universe universe(begin_time,end_time,step_size,radius);
```

Where `begin_time`, `end_time`, `step_size`, and `radius` are `float`. `radius` is the size of your universe. If any particles go past the bounds of this radius (specifically it is a square with side lengths equal to `radius`, and its bottom left corner is at (0,0)) they will be excluded from the computation of net forces of particles.

To add your particles to the `Universe` object, you write:

```cpp
universe.addChild(p);
```

and to start the simulation,

```cpp
universe.start();
```
