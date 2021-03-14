# Unity-DOTS-RTS-Collision-System
A basic RTS collision system.
Unity 2020.2.3f1

A physics collision system to handle very large amounts of units, in the 30-50 thousand range.
Uses ECS, jobs and Burst.
It uses a fixed grid array or fixed grid multi hashmap.
The grid is state-dependant, meaning if you kill units you must rebuild it occasionally;
this shortcut allows my broadphase to be much faster.
You can get away with updating the grid every 5 minutes, so no big deal.

If the scene has some broken dependencies, all you have to do is add a mesh and material to GameManager in the editor 
for units, collision will work. For the map you need an object with the script MapGenerator(),
as well as an empty object with mesh filter and mesh renderer.

You can change the number of units and where they spawn in UnitSpawner(). You can toggle a boolean to
give them a radnom destination on start.

You can control the camera in Play-Mode with WASD, TG, QE and mouse
You can select units with left click, or drag select, and right click to send them to location
By default, all units have random target position
You can disable this in unit spawner
You can change number of units spanwed and where they spawn in unit spawner

Next updates include:
- hierarchical grid, for different unit sizes
- k-d or binary tree for static collision
- SIMD some stuff