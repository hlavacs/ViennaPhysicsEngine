/**
* The Vienna Physics Engine — portable example
*
* Renders VPE scenes with raylib (https://www.raylib.com), so the physics can
* be demonstrated on macOS/Linux/Windows without the Vienna Vulkan Engine.
* The full-featured demo (debug UI, picking) remains examples/physicsexample.
*
* (c) bei Helmut Hlavacs, University of Vienna, 2026
*
* Controls:
*   1  stack of cubes         2  ball-socket pendulum
*   3  hinge                  4  cloth
*   SPACE  drop a random cube          R  reset scene
*   WASD + mouse drag: move camera     ESC  quit
*
* Note on coordinates: VPE is left handed, raylib right handed (both Y up).
* Rendering mirrors the z axis: M_ray = S * M_vpe * S with S = diag(1,1,-1,1).
*/

#include "VPE.hpp"

#include "raylib.h"

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

using namespace vpe;

namespace {

//---------------------------------------------------------------------------
// VPE -> raylib conversions (mirror z to go from left handed to right handed)
//---------------------------------------------------------------------------

Matrix toRaylib(const glmmat4& m) {
	glmmat4 s{ 1 };
	s[2][2] = -1;
	glmmat4 r = s * m * s;
	Matrix out;	//raylib Matrix is column major in memory, like glm
	out.m0 = (float)r[0][0]; out.m1 = (float)r[0][1]; out.m2  = (float)r[0][2]; out.m3  = (float)r[0][3];
	out.m4 = (float)r[1][0]; out.m5 = (float)r[1][1]; out.m6  = (float)r[1][2]; out.m7  = (float)r[1][3];
	out.m8 = (float)r[2][0]; out.m9 = (float)r[2][1]; out.m10 = (float)r[2][2]; out.m11 = (float)r[2][3];
	out.m12 = (float)r[3][0]; out.m13 = (float)r[3][1]; out.m14 = (float)r[3][2]; out.m15 = (float)r[3][3];
	return out;
}

Vector3 toRaylib(const glmvec3& v) { return Vector3{ (float)v.x, (float)v.y, (float)-v.z }; }

//---------------------------------------------------------------------------
// Scene handling
//---------------------------------------------------------------------------

/// Owners must be unique per body; the demo does not use callbacks, so plain
/// increasing ids serve as keys. (Phase 2 of REFACTORING.md replaces this.)
void* nextOwner() {
	static std::uintptr_t counter = 0;
	return reinterpret_cast<void*>(++counter);
}

std::shared_ptr<VPEWorld::Body> makeBody(VPEWorld& world, std::string name,
	glmvec3 position, real mass_inv, glmquat orientation = { 1, 0, 0, 0 },
	glmvec3 scale = { 1, 1, 1 }) {
	auto body = std::make_shared<VPEWorld::Body>(
		&world, name, nextOwner(), &world.g_cube, scale, position, orientation,
		glmvec3{ 0 }, glmvec3{ 0 }, mass_inv, world.m_restitution, world.m_friction);
	if (mass_inv > 0.0_real) {
		body->setForce(0, VPEWorld::Force{ { 0.0_real, world.c_gravity, 0.0_real } });
	}
	world.addBody(body);
	return body;
}

struct Scene {
	std::unique_ptr<VPEWorld>			world = std::make_unique<VPEWorld>();
	std::shared_ptr<VPEWorld::Cloth>	cloth;			//nullptr if no cloth in scene
	std::vector<uint32_t>				clothIndices;	//triangle indices for cloth rendering
	const char*							title = "empty — press 1..4";
};

void sceneStack(Scene& s) {
	s.title = "stack";
	for (int i = 0; i < 5; ++i) {
		makeBody(*s.world, "cube" + std::to_string(i),
			glmvec3{ 0.01_real * i, 0.5_real + 1.1_real * i, 0.005_real * i }, 1.0_real);
	}
}

void scenePendulum(Scene& s) {
	s.title = "ball-socket pendulum";
	auto anchor = makeBody(*s.world, "anchor", glmvec3{ 0, 6, 0 }, 0.0_real);
	auto bob    = makeBody(*s.world, "bob",    glmvec3{ 2, 6, 0 }, 1.0_real);
	s.world->addConstraint(std::make_shared<VPEWorld::BallSocketJoint>(
		anchor, bob, glmvec3{ 0, 6, 0 }));
}

void sceneHinge(Scene& s) {
	s.title = "hinge";
	auto post = makeBody(*s.world, "post", glmvec3{ 0, 6, 0 }, 0.0_real);
	auto door = makeBody(*s.world, "door", glmvec3{ 2, 6, 0 }, 1.0_real);
	s.world->addConstraint(std::make_shared<VPEWorld::HingeJoint>(
		post, door, glmvec3{ 1, 6, 0 }, glmvec3{ 0, 0, 1 }));
}

void sceneCloth(Scene& s) {
	s.title = "cloth";
	constexpr int N = 10;
	std::vector<glmvec3> vertices;
	for (int y = 0; y < N; ++y) {
		for (int x = 0; x < N; ++x) {
			vertices.emplace_back(0.3_real * x - 1.5_real, 3.0_real + 0.3_real * y, 0.0_real);
		}
	}
	for (int y = 0; y < N - 1; ++y) {
		for (int x = 0; x < N - 1; ++x) {
			uint32_t i0 = y * N + x, i1 = i0 + 1, i2 = i0 + N, i3 = i2 + 1;
			s.clothIndices.insert(s.clothIndices.end(), { i0, i1, i2, i1, i3, i2 });
		}
	}
	std::vector<glmvec3> fixed{ vertices[(N - 1) * N], vertices[N * N - 1] };	//top corners
	s.cloth = std::make_shared<VPEWorld::Cloth>(
		s.world.get(), "cloth", nextOwner(), nullptr, nullptr,
		vertices, s.clothIndices, fixed, 1.0_real, 4, 0.8_real);
	s.world->addCloth(s.cloth);
}

void dropCube(Scene& s, std::mt19937& rng) {
	std::uniform_real_distribution<float> pos(-2.0f, 2.0f), angle(0.0f, 3.14f);
	glmquat q = glm::angleAxis(angle(rng), glm::normalize(glmvec3{ pos(rng), 1, pos(rng) }));
	makeBody(*s.world, "drop", glmvec3{ pos(rng), 8.0_real, pos(rng) }, 1.0_real, q);
}

//---------------------------------------------------------------------------
// Rendering
//---------------------------------------------------------------------------

Color bodyColor(const VPEWorld::Body& body) {
	if (body.m_mass_inv == 0.0_real) { return GRAY; }
	const Color palette[]{ ORANGE, SKYBLUE, LIME, GOLD, PINK, VIOLET, BEIGE, MAROON };
	return palette[std::hash<std::string>{}(body.m_name) % std::size(palette)];
}

void drawBodies(const VPEWorld& world, Model& cube) {
	for (const auto& entry : world.m_bodies) {
		const auto& body = *entry.second;
		cube.transform = toRaylib(body.m_model);
		DrawModel(cube, Vector3{ 0, 0, 0 }, 1.0f, bodyColor(body));
		DrawModelWires(cube, Vector3{ 0, 0, 0 }, 1.0f, DARKGRAY);
	}
}

void drawCloth(Scene& s) {
	if (!s.cloth) { return; }
	std::vector<glmvec3> v = s.cloth->generateVertices();
	for (size_t i = 0; i + 2 < s.clothIndices.size(); i += 3) {
		Vector3 a = toRaylib(v[s.clothIndices[i]]);
		Vector3 b = toRaylib(v[s.clothIndices[i + 1]]);
		Vector3 c = toRaylib(v[s.clothIndices[i + 2]]);
		DrawTriangle3D(a, b, c, RED);		//draw both windings: cloth is visible
		DrawTriangle3D(c, b, a, MAROON);	//from both sides
	}
}

} // namespace

int main() {
	SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
	InitWindow(1280, 720, "Vienna Physics Engine — simple example (raylib)");
	SetTargetFPS(60);

	Camera3D camera{};
	camera.position = Vector3{ 8.0f, 6.0f, 8.0f };
	camera.target = Vector3{ 0.0f, 2.0f, 0.0f };
	camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	Model cube = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
	std::mt19937 rng{ 42 };

	Scene scene;
	sceneStack(scene);
	scene.title = "stack";

	while (!WindowShouldClose()) {
		if (IsKeyPressed(KEY_ONE))   { scene = Scene{}; sceneStack(scene); }
		if (IsKeyPressed(KEY_TWO))   { scene = Scene{}; scenePendulum(scene); }
		if (IsKeyPressed(KEY_THREE)) { scene = Scene{}; sceneHinge(scene); }
		if (IsKeyPressed(KEY_FOUR))  { scene = Scene{}; sceneCloth(scene); }
		if (IsKeyPressed(KEY_R))     { scene = Scene{}; }
		if (IsKeyPressed(KEY_SPACE)) { dropCube(scene, rng); }
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) { UpdateCamera(&camera, CAMERA_THIRD_PERSON); }

		scene.world->tick(GetFrameTime());

		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);
		DrawGrid(20, 1.0f);		//VPE's built-in ground plane is at y = 0
		drawBodies(*scene.world, cube);
		drawCloth(scene);
		EndMode3D();

		DrawText(TextFormat("scene: %s   bodies: %d   fps: %d",
			scene.title, (int)scene.world->m_bodies.size(), GetFPS()), 10, 10, 20, DARKGRAY);
		DrawText("1 stack | 2 pendulum | 3 hinge | 4 cloth | SPACE drop cube | R reset | drag LMB camera",
			10, 690, 20, GRAY);
		EndDrawing();
	}

	UnloadModel(cube);
	CloseWindow();
	return 0;
}
