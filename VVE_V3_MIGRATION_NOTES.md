# VVE V3 migration

There are two ways the physics example consumes ViennaVulkanEngine V3, selected
by the CMake option `VPE_VVE_IN_TREE`:

- **In-tree (Windows default).** The root `CMakeLists.txt` adds
  `../ViennaVulkanEngine` with `add_subdirectory`, so VVE's modules, the
  `VEPhysicsEngine` module and `physicsexample` are compiled by one Ninja +
  MSVC build with `import std`. This is the only configuration that works with
  MSVC, whose `.ifc` module files require identical compiler flags on both
  sides. The example then links `ViennaVulkanEngine::ViennaVulkanEngine` and
  `imgui::imgui` exactly like VVE's own examples and calls
  `vve_set_output_dirs`, so `physicsexample.exe` lands in
  `ViennaVulkanEngine/bin/<config>/exe` next to the runtime DLLs. VPE uses
  VVE's vcpkg `glm` in this mode so both modules see the same glm
  declarations. `build_cmake.cmd` drives this mode. VVE's own examples and
  tests are not built unless `VPE_BUILD_VVE_EXAMPLES` / `VPE_BUILD_VVE_TESTS`
  are enabled. VVE had to be made subdirectory-safe for this: it now uses
  `CMAKE_CURRENT_SOURCE_DIR` for its vcpkg and `bin/` paths, exports the
  output-directory variables to the parent scope, and guards its example
  build behind `VVE_BUILD_EXAMPLES`.

- **Prebuilt (Linux/macOS default).** The example consumes the already-built
  ViennaVulkanEngine V3 release tree and does not configure or rebuild VVE.
  The rest of this document describes that mode.

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
