# VVE V3 migration

The physics example consumes the already-built ViennaVulkanEngine V3 release
tree. It does not configure or rebuild VVE.

`examples/physicsexample/CMakeLists.txt` first looks for VVE in the documented
`../ViennaVulkanEngine` sibling location. Controller worktrees use
`../../ViennaVulkanEngine`, which is supported as a fallback.

The executable links the prebuilt VVE shared library and ImGui archive, and
reuses VVE's compiled Clang C++23 facade and standard-library module files.
The VVE vcpkg include/library tree also supplies the matching Vulkan loader.

The source now uses:

- `EngineBuilder` and `WindowSetup` for engine/window startup;
- `RenderSystem::addPlane` and `addTexturedCuboid` for scene authoring;
- render-object handles with `setObjectTransform` and `removeObject` for VPE
  body callbacks;
- directional and point lights through the V3 render facade;
- a V3 user-system update callback to call `VPEWorld::tick`;
- `GuiSystem::draw` and ImGui for the physics and constraint panels.

For automated smoke testing, the executable accepts `--frames N`. Adding
`--stack` creates the 120-cube stacking scenario at startup and reports whether
all body positions remain finite and bounded at exit.
