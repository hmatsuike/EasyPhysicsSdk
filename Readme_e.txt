///////////////////////////////////////////////////////////////////////////////
// Easy Physics SDK Package Version 0.8

===============================================================================
== Overview

'Easy Physics SDK' is the C++ library which can be used to realize the real-time rigid body simulation. It provides not only basic functions to handle rigid bodies, but also advanced functions to support various joints and conitnuous collision detection.

'Physics simulation for game developers' published from Impress Japan Corporation gives detailed explanations of the 'Easy Physics' library. This library offers more improved fueatures than the one included in the book.

** Major improvements
 - Supports speculative contacts to prevent collision tunneling.
 - Various joints by a 6DOF joint.
 - Optimized broadphase by the 1 axis sort and sweep algorithm.
 - Optimized collision detection between convex meshes.

===============================================================================
== Features

Rigid bodies are handled as convex meshes.
Each rigid body has up to 5 convex meshes.
Following joints are supported
 - Ball/Hinge/Fixed/Swing twist/Slider

===============================================================================
== Install

 - The visual studio project file to build the library
EasyPhysicsSdk/easy_physics/project/easy_physics.vcxproj

 - The visual Studio solution file to build all.
 asyPhysicsSdk/sample/physics_sample_vs2010.sln

===============================================================================
== About Samples

In all samples, each scene is created in physicsCreateScene() in physics_func.cpp.
Simulation is executed in physicsSimulate().

 - EasyPhysicsSdk/sample/01_basic
This sample shows basic aspects of a rigid body simulation.

 - EasyPhysicsSdk/sample/02_compound
This sample shows advanced features such as combined objects or rigid bodies which put off a center of gravity.

 - EasyPhysicsSdk/sample/03_joint
This sample shows how to use joints which connect rigid bodies to create complex objects.

===============================================================================
== Integration to your application

 - Using Easy Physics SDK API
Please add the Easy Physics SDK project into your application solution. Then, include 'EasyPhysicsSdk/easy_physics/EpxInclude.h' in your source code. All of the other necessary headers are referred relatively in 'EpxInclude.h'.

===============================================================================
== Buid Environment

OS : Windows XP or later
CPU : 1GHz
VGA : OpenGL supported video card
Compiler : Visual Studio 2010 (Express Edition)

===============================================================================
https://github.com/hmatsuike/EasyPhysicsSdk
Hiroshi Matsuike(hmatsuike@gmail.com)
