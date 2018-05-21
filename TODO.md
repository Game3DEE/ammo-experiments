# The Crew I notes

Since The Crew was a big inspritation for this project, and many hours have gone in playing it, I'm collection my analysis of different parts of the game, and how I applied it to my own.

## Terrain

I can see 3 main terrain types:

* Road (highway, city streets, etc)
* Dirt road (Farm roads, forest roads)
* Off road (Fields, deserts, salt flats, etc)

It defines increasing friction on the terrain types, with road having least friction, and off-road most.

Then it defines the car specs as having most/least grip, from Raid having most grip, to Circuit having least(?).
(The full list of specs is Raid, Dirt, Street, Performance, Circuit )

This way they can simply use the physics engine to take care of Circuit cars sliding across off-road terrain and the like.

## Destructables

All "destructables" in the crew are not really destructable, but just objects that "move" on enough impact. Wether this "enough" is defined by the amount of mass assigned or some "on impact" test I'm not sure. In any case, any "destructable" objects consist of 1 or more parts, and on impact they are seperated and flung around.

As increasing the amount of movable objects impacts performance, I want to see if there's a possibility of giving them mass 0 to start and on impact change that. Not sure that's going to work.

Also, destructables are only temporarily moved, after a predefined amount of seconds the object returns to its original state (I'm guessing this is because of the "always-online" multiplayer feature). Maybe this is even just done locally, and not seen on other clients (We saw similar behaviour with Trains in NFS: Payback).

# Bullet Physics

## Attributes

1. Objects:
    * Mass
    * Friction
    * Restitution
    * Dampening
2. Vehicle:
    * friction slip
    * suspension force
    * suspension travel(cm)
    * suspension compression
    * suspension damping
    * suspension stiffness
    * Per Wheel:
        * suspension stiffness
        * wheels damping relaxation
        * wheels damping compression
        * friction slip
        * roll Influence

# Interesting Links

This is just a list of interesting links, to code/libraries/papers that are relevant to this project in some way. Mostly here so I don't loose them ;)

- [Speedo Codepen](https://codepen.io/alvaromontoro/pen/XzaZra)
- [Bullet pull request](https://github.com/bulletphysics/bullet3/pull/448) that has a nice "hack" for center of gravity issues.

