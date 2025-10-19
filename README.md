# Rigidbody Playground

![alt text](https://cdn.discordapp.com/attachments/1406594805236236308/1429593902121226250/icon256.png?ex=68f6b492&is=68f56312&hm=529c9790bde0bf7d873181933421673a0039afb6e950a9c44dd961a074808704&)

Rigibody playground is a 2D Physics Engine with a clean UI.

It allows for some basic lua scripting (not many in-built functions but easily extendable from the source code)

Its made as a project for me to explore the ideas of making a physics engine and setting it as a challenge because I've always failed at making one in the past.

# Features

 The way I approached in making this phyiscs engine is building a matrix with all the collision equations and continuing by solving it with Projected Gauss-Seidel. It uses SAT for collision detections and has AABB for optimization purposes (although, after some tests, it seems to make the performance worse for some reason that I'll have to investigate further).

It also supports scripting through Lua with some basic functions built in, but it's definitely not ready for use. Also It allows for saving and loading scenes that you've built from the editor.
