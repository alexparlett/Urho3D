---
layout: page
title: About
---
{% include JB/setup %}

**Urho3D** is a lightweight, cross-platform rendering and game engine implemented in C++ and released under the MIT license. Greatly inspired by [OGRE](http://www.ogre3d.org/) and [Horde3D](http://www.horde3d.org/).

## Features
- Direct3D9 or OpenGL rendering (Shader Model 2, OpenGL 2.0 or OpenGL ES 2.0 required as minimum)
- HLSL or GLSL shaders + optional offline compilation of HLSL code
- Configurable rendering pipeline. Default implementations for forward, light pre-pass and deferred rendering
- Component based scene model
- Skeletal (with hardware skinning), vertex morph and node animation
- Automatic instancing on SM3 capable hardware
- Point, spot and directional lights
- Shadow mapping for all light types; cascaded shadow maps for directional lights
- Particle rendering
- Geomipmapped terrain
- Static and skinned decals
- Auxiliary view rendering (reflections etc.)
- Geometry, material & animation LOD
- Software rasterized occlusion culling
- Post-processing
- Task-based multithreading
- Hierarchical performance profiler
- Scene and object load/save in binary and XML format
- Keyboard, mouse, joystick and touch input (if available)
- Cross-platform support using [SDL 2.0](http://www.libsdl.org/) (currently runs on Windows, Linux, Mac OS X, Android, iOS, and [Raspberry Pi](http://www.raspberrypi.org/)<font color="olive"><sup>v1.3</sup></font>)
- Physics using [Bullet](http://www.bulletphysics.org/)
- Scripting using [AngelScript](http://www.angelcode.com/angelscript/)
- Alternative script interface using [Lua](http://www.lua.org/)<font color="olive"><sup>v1.3</sup></font> or [LuaJIT](http://luajit.org/)<font color="green"><sup>new</sup></font> (on Windows, Linux, Mac OS X, Android, and Raspberry Pi)
- Networking using [kNet](https://github.com/juj/kNet/) + possibility to make HTTP requests<font color="olive"><sup>v1.3</sup></font>
- Pathfinding using [Recast/Detour](https://code.google.com/p/recastnavigation/)<font color="olive"><sup>v1.23</sup></font>
- Image loading using [stb_image](http://nothings.org/) + DDS / KTX / PVR compressed texture support
- 2D and "3D" audio playback, Ogg Vorbis support using [stb_vorbis](http://nothings.org/) + WAV format support
- TrueType font rendering using [FreeType](http://www.freetype.org/), [AngelCode bitmap fonts](http://www.angelcode.com/products/bmfont/) are also supported
- Unicode string support
- Inbuilt UI system
- Scene editor and UI-layout editor implemented in script with undo & redo capabilities
- Model/scene/animation/material import from formats supported by [Open Asset Import Library](http://assimp.sourceforge.net)
- Alternative model/animation import from OGRE mesh.xml and skeleton.xml files
- Supported build tools and IDEs: Visual Studio, Xcode, Eclipse, GCC, LLVM/Clang, MinGW
- Supports both 32-bit and 64-bit<font color="olive"><sup>v1.3</sup></font> build
- Build as single external library<font color="olive"><sup>v1.3</sup></font> (can be linked against statically or dynamically)

## Discussions, support and contributing
- There is a Google Groups [discussion group / mailing list](http://groups.google.com/group/urho3d/) for questions and discussion.
- You can also post Urho3D-related questions to the [GameDev.net APIs and Tools forum](http://www.gamedev.net/forum/41-apis-and-tools/).<br>
- For bug reports and contributing, please use the GitHub [issue tracker](https://github.com/urho3d/Urho3D/issues/) and [pull requests](https://github.com/urho3d/Urho3D/pulls) pages.
- When you contribute, you have to agree that your work will be released under the license as seen at the top of [License.txt](https://github.com/urho3d/Urho3D/blob/master/License.txt).

## Limitations
Though Urho3D already contains several useful features and implements a framework for 3D games and applications, it is not yet a complete out-of-the-box game creation toolkit. Some things you can expect having to work on, or to require skills for: (contributions to the project are naturally welcome)
- C++ for performance-critical code and improving existing subsystems such as networking, physics and animation, depending on your needs
- HLSL and GLSL shaders: the provided shaders give only basic examples of what is possible
- Additional optimization for mobile devices
- Editors and additional asset conversion functionality
- Integration of additional 3rd party libraries, for example a more complete UI library, HTML rendering, or additional network protocols

## Engine development, contributions and bugfixes by
- Lasse Öörni (loorni@gmail.com, AgentC on GameDev.net, @cadaver)
- Wei Tjong Yao (@weitjong)
- Aster Jian (@aster2013)
- Colin Barrett
- Erik Beran
- Carlo Carollo
- Sebastian Delatorre (primitivewaste)
- Chris Friesen
- Alex Fuller
- Mika Heinonen
- Jason Kinzer
- Gunnar Kriik
- Ali Kämäräinen
- Pete Leigh
- Paul Noome
- Vladimir Pobedinsky
- Miika Santala
- Firegorilla
- Magic.Lixin
- amadeus_osa
- reattiva
- skaiware

## Origin of the name
**Urho** (Finnish for "hero" or "brave fighter") was a legendary, tyrannical king among the fish. He always hunted without mercy. Sadly, he passed away on 6th November 2009. This engine was originally called Bofh3D (hinting at the covert goal of reimplementing the game BOFH: Servers under Siege in 3D, which still stands) but was renamed to Urho3D in his honor. For more of Urho's legend, see also the music project http://www.mikseri.net/Urho/.
