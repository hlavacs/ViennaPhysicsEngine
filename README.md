# The Vienna Physics Engine (VPE)
The Vienna Physics Engine (VPE) is a simple, single file physics engine for educational purposes.
Rendering is done by the Vienna Vulkan Engine.

The VPE is developed as basis for game based courses at the Faculty of Computer Science of the University of Vienna, held by Prof. Helmut Hlavacs:

- https://ufind.univie.ac.at/de/course.html?lv=052214&semester=2019W
- https://ufind.univie.ac.at/de/course.html?lv=052212&semester=2019W
- https://ufind.univie.ac.at/de/course.html?lv=052211&semester=2019S

VPE's main contributor is Prof. Helmut Hlavacs (http://entertain.univie.ac.at/~hlavacs/). However, VPE will be heavily involved in the aforementioned courses, and other courses as well.

VPE features are:
- C++20
- Polytopes only
- Sequential impulse based
- Implements two solvers to choose from
- Friction
- Contact set reduction
- Warm starting, stable stacking

# Set up for Windows 10

The whole engine is contained in VPE.hpp. Clone the project, open the .sln file, compile, run.
For Windows 11, all non-Vulkan dependencies are in the external directory. The Vulkan SDK is supposed to be pointed at by the VULKAN_SDK environment variable.

The project will be updated regularly, so it makes sense to download the source files once in a while, or clone the whole project. Make sure to keep your main.cpp or other files that you created.

# Using VPE

You can create bodies, erase bodies, attach forces to bodies. When creating bodies you can also
specify two callbacks. One is called when the body moves, so the render engine can update its position
and orientation. The other is called if the body is erased by calling eraseBody() or clear().

The simulation is advanced by calling tick(). See how the debug panel works for more options.

## Links
-	https://github.com/hlavacs/ViennaVulkanEngine
