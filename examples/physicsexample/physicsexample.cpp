/**
 * Vienna Physics Engine example rendered through the Vienna Vulkan Engine V3 facade.
 */

#include <imgui.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <expected>
#include <memory>
#include <optional>
#include <random>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "VPEConstraintDemos.hpp"

import VEEngine;
import VEPhysicsEngine;

namespace {

using namespace vpe;

constexpr vve::PixelExtent windowExtent{.width = 1280, .height = 720};
constexpr vve::Vec3 cubeMinimum{-0.5F, -0.5F, -0.5F};
constexpr vve::Vec3 cubeMaximum{0.5F, 0.5F, 0.5F};
constexpr double headlessPhysicsTimeStep = 1.0 / 60.0;
constexpr int headlessMinimumSettleFrames = 300;
constexpr real headlessSettleSpeedThreshold = 0.05_real;
constexpr real headlessSettleAngularSpeedThreshold = 0.05_real;
constexpr real headlessHorizontalDriftTolerance = 0.25_real;

struct BodyVisual {
	vve::RenderSystem render;
	vve::RenderObjectHandle object;
};

struct PhysicsTickSystem {
	VPEWorld *physics{};

	template <typename TWorld>
	std::expected<void, vve::Error> update(TWorld &, const vve::FrameContext &frame) {
		physics->tick(frame.delta_time.seconds);
		return {};
	}
};

[[nodiscard]] vve::Transform renderTransform(
	const glmvec3 &position, const glmquat &orientation, const glmvec3 &scale) {
	return vve::Transform{
		.translation = vve::Position{
			.value = vve::Vec3{static_cast<float>(position.x), static_cast<float>(position.y),
									static_cast<float>(position.z)}},
		.rotation = vve::Rotation{
			.value = vve::Quat{static_cast<float>(orientation.w), static_cast<float>(orientation.x),
								 static_cast<float>(orientation.y), static_cast<float>(orientation.z)}},
		.scale = vve::Scale{
			.value = vve::Vec3{static_cast<float>(scale.x), static_cast<float>(scale.y),
								 static_cast<float>(scale.z)}}};
}

[[nodiscard]] glmvec3 physicsVector(const vve::Vec3 &value) {
	return {static_cast<real>(value.x), static_cast<real>(value.y), static_cast<real>(value.z)};
}

[[nodiscard]] vve::Vec3 cameraForward(const vve::DefaultCameraController &camera) {
	return vve::math::normalize(vve::Vec3{
		std::cos(camera.pitch) * std::sin(camera.yaw), std::sin(camera.pitch),
		-std::cos(camera.pitch) * std::cos(camera.yaw)});
}

[[nodiscard]] std::optional<int> frameLimit(int argc, char **argv) {
	for (int index = 1; index + 1 < argc; ++index) {
		if (argv[index] == nullptr || argv[index + 1] == nullptr ||
			 std::string_view{argv[index]} != "--frames") {
			continue;
		}
		int value{};
		const std::string_view text{argv[index + 1]};
		const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
		if (result.ec == std::errc{} && value >= 0) {
			return value;
		}
	}
	return std::nullopt;
}

[[nodiscard]] bool hasArgument(int argc, char **argv, std::string_view argument) {
	for (int index = 1; index < argc; ++index) {
		if (argv[index] != nullptr && std::string_view{argv[index]} == argument) {
			return true;
		}
	}
	return false;
}

[[nodiscard]] std::filesystem::path crateTexture() {
#ifdef VVE_V3_ROOT_PATH
	return std::filesystem::path{VVE_V3_ROOT_PATH} / "assets/game/crate0/diffuse.png";
#else
	return std::filesystem::path{"../ViennaVulkanEngine/assets/game/crate0/diffuse.png"};
#endif
}

[[nodiscard]] VPEWorld::callback_move makeMoveCallback(vve::RenderSystem render) {
	return [render](double dt, std::shared_ptr<VPEWorld::Body> body) mutable {
		auto *visual = static_cast<BodyVisual *>(body->m_owner);
		if (visual == nullptr) {
			return;
		}
		glmvec3 position = body->m_positionW;
		glmquat orientation = body->m_orientationLW;
		body->stepPosition(dt, position, orientation, false);
		if (const auto result = render.setObjectTransform(
				visual->object, renderTransform(position, orientation, body->m_scale));
			 !result) {
			std::cerr << "[physicsexample] transform update failed: error="
						 << vve::errorName(result.error()) << '\n';
		}
	};
}

[[nodiscard]] VPEWorld::callback_erase makeEraseCallback(vve::RenderSystem render) {
	return [render](std::shared_ptr<VPEWorld::Body> body) mutable {
		auto *visual = static_cast<BodyVisual *>(body->m_owner);
		if (visual == nullptr) {
			return;
		}
		if (const auto result = render.removeObject(visual->object); !result) {
			std::cerr << "[physicsexample] render-object removal failed: error="
						 << vve::errorName(result.error()) << '\n';
		}
		delete visual;
	};
}

[[nodiscard]] ve::ConstraintDemos::create_visual_callback makeVisualFactory(
	vve::RenderSystem render, std::filesystem::path texture) {
	return [render, texture = std::move(texture)](
				  glmvec3 scale, glmvec3 position, glmquat orientation) mutable -> void * {
		const auto object = render.addTexturedCuboid(
			cubeMinimum, cubeMaximum, texture, renderTransform(position, orientation, scale));
		if (!object) {
			std::cerr << "[physicsexample] cube creation failed: error="
						 << vve::errorName(object.error()) << '\n';
			throw std::runtime_error{"VVE V3 could not create a physics cube"};
		}
		return new BodyVisual{render, *object};
	};
}

[[nodiscard]] std::shared_ptr<VPEWorld::Body> addCube(
	VPEWorld &physics, const ve::ConstraintDemos::create_visual_callback &create_visual,
	const VPEWorld::callback_move &on_move, const VPEWorld::callback_erase &on_erase,
	glmvec3 scale, glmvec3 position, glmquat orientation = glmquat{1, 0, 0, 0},
	glmvec3 velocity = glmvec3{0.0_real}, glmvec3 angular_velocity = glmvec3{0.0_real},
	real inverse_mass = 1.0_real / 100.0_real, bool gravity = true,
	real friction = 1.0_real) {
	void *visual = create_visual(scale, position, orientation);
	if (visual == nullptr) {
		return {};
	}
	auto body = std::make_shared<VPEWorld::Body>(
		&physics, "Body" + std::to_string(physics.m_bodies.size()), visual, &physics.g_cube,
		scale, position, orientation, velocity, angular_velocity, inverse_mass,
		physics.m_restitution, friction);
	body->m_on_move = on_move;
	body->m_on_erase = on_erase;
	if (gravity) {
		body->setForce(0ul, VPEWorld::Force{{0, physics.c_gravity, 0}});
	}
	physics.addBody(body);
	on_move(0.0, body);
	return body;
}

void addStack(
	VPEWorld &physics, const ve::ConstraintDemos::create_visual_callback &create_visual,
	const VPEWorld::callback_move &on_move, const VPEWorld::callback_erase &on_erase) {
	for (int y = 0; y < 15; ++y) {
		for (int x = 0; x < 15 - y; ++x) {
			(void)addCube(physics, create_visual, on_move, on_erase, glmvec3{1.0_real},
						glmvec3{x + 0.4_real * y, 0.5_real + y, 0.0_real});
		}
	}
}

int runHeadlessStack(VPEWorld &physics, int frames) {
	struct InitialBodyState {
		std::shared_ptr<VPEWorld::Body> body;
		glmvec2 horizontal_position;
	};

	const ve::ConstraintDemos::create_visual_callback create_visual =
		[](glmvec3, glmvec3, glmquat) -> void * { return new int{}; };
	const VPEWorld::callback_move on_move = [](double, std::shared_ptr<VPEWorld::Body>) {};
	const VPEWorld::callback_erase on_erase = [](std::shared_ptr<VPEWorld::Body> body) {
		delete static_cast<int *>(body->m_owner);
	};
	addStack(physics, create_visual, on_move, on_erase);

	std::vector<InitialBodyState> initial_states;
	initial_states.reserve(physics.m_bodies.size());
	real initial_stack_height{};
	for (const auto &entry : physics.m_bodies) {
		const auto &body = entry.second;
		initial_states.push_back(
			{body, glmvec2{body->m_positionW.x, body->m_positionW.z}});
		initial_stack_height = std::max(
			initial_stack_height, body->m_positionW.y + 0.5_real * body->m_scale.y);
	}

	if (frames > 0) {
		// tick() first establishes VPE's fixed-slot clock. Each following fixed-delta call
		// advances one complete 1/60 s solver slot, independent of render timing.
		physics.tick(headlessPhysicsTimeStep);
		for (int frame = 0; frame < frames; ++frame) {
			physics.tick(headlessPhysicsTimeStep);
		}
	}

	const bool completed =
		frames >= headlessMinimumSettleFrames &&
		physics.m_loop >= static_cast<decltype(physics.m_loop)>(frames);
	real max_linear_speed{};
	real max_angular_speed{};
	real max_horizontal_drift{};
	real min_final_y = initial_stack_height;
	real max_final_y{};
	bool bodies_settled = initial_states.size() == physics.m_bodies.size();
	for (const auto &initial : initial_states) {
		const auto &position = initial.body->m_positionW;
		const real linear_speed = glm::length(initial.body->m_linear_velocityW);
		const real angular_speed = glm::length(initial.body->m_angular_velocityW);
		const real horizontal_drift =
			glm::length(glmvec2{position.x, position.z} - initial.horizontal_position);
		max_linear_speed = std::max(max_linear_speed, linear_speed);
		max_angular_speed = std::max(max_angular_speed, angular_speed);
		max_horizontal_drift = std::max(max_horizontal_drift, horizontal_drift);
		min_final_y = std::min(min_final_y, position.y);
		max_final_y = std::max(max_final_y, position.y);

		const bool finite_position =
			std::isfinite(position.x) && std::isfinite(position.y) &&
			std::isfinite(position.z);
		bodies_settled =
			bodies_settled && finite_position &&
			linear_speed <= headlessSettleSpeedThreshold &&
			angular_speed <= headlessSettleAngularSpeedThreshold &&
			position.y >= 0.0_real && position.y <= initial_stack_height &&
			horizontal_drift <= headlessHorizontalDriftTolerance;
	}
	const bool stable = completed && bodies_settled;
	std::cout << "[physicsexample] settle max_residual_speed=" << max_linear_speed
				 << " max_angular_speed=" << max_angular_speed
				 << " max_horizontal_drift=" << max_horizontal_drift
				 << " final_y=[" << min_final_y << ',' << max_final_y << "]\n";
	std::cout << "[physicsexample] stack=" << (stable ? "stable" : "unstable")
				 << " bodies=" << physics.m_bodies.size() << '\n';
	physics.clear();
	std::cout << "[physicsexample] frames=" << frames << '\n';
	return stable ? 0 : 4;
}

void addRandomBodies(
	VPEWorld &physics, const ve::ConstraintDemos::create_visual_callback &create_visual,
	const VPEWorld::callback_move &on_move, const VPEWorld::callback_erase &on_erase,
	std::default_random_engine &generator, std::uniform_real_distribution<real> &random,
	int count) {
	for (int index = 0; index < count; ++index) {
		const glmvec3 position{
			random(generator), 20.0_real * random(generator) + 10.0_real, random(generator)};
		const glmvec3 velocity{random(generator), random(generator), random(generator)};
		const real angle = random(generator) * 10.0_real * 3.0_real * pi / 180.0_real;
		const glmvec3 axis = glm::normalize(
			glmvec3{random(generator), random(generator), random(generator)});
		const glmvec3 angular_velocity{
			random(generator) * 5.0_real, random(generator) * 5.0_real,
			random(generator) * 5.0_real};
		(void)addCube(physics, create_visual, on_move, on_erase, glmvec3{1.0_real}, position,
					glm::rotate(glmquat{1, 0, 0, 0}, angle, axis), velocity, angular_velocity);
	}
}

void drawBooleanSetting(const char *label, bool &value) {
	ImGui::TextUnformatted(label);
	ImGui::SameLine();
	if (ImGui::RadioButton((std::string{"Yes##"} + label).c_str(), value)) {
		value = true;
	}
	ImGui::SameLine();
	if (ImGui::RadioButton((std::string{"No##"} + label).c_str(), !value)) {
		value = false;
	}
}

struct LightControls {
	bool directional_enabled{true};
	std::array<float, 3> directional_direction{-0.45F, -0.8F, 0.35F};
	std::array<float, 3> directional_color{0.95F, 0.98F, 1.0F};
	float directional_intensity{1.05F};
	std::array<float, 3> directional_ambient{0.04F, 0.04F, 0.04F};

	bool point_enabled{true};
	std::array<float, 3> point_position{2.0F, 8.0F, -4.0F};
	std::array<float, 3> point_color{1.0F, 0.96F, 0.82F};
	float point_intensity{3.0F};
	float point_range{30.0F};
	std::array<float, 3> point_ambient{0.08F, 0.08F, 0.08F};
};

[[nodiscard]] vve::Vec3 normalizedDirection(const std::array<float, 3> &direction) {
	const float length_squared =
		direction[0] * direction[0] + direction[1] * direction[1] +
		direction[2] * direction[2];
	if (length_squared <= 0.000001F) {
		return vve::Vec3{-0.45F, -0.8F, 0.35F};
	}
	const float inverse_length = 1.0F / std::sqrt(length_squared);
	return vve::Vec3{
		direction[0] * inverse_length, direction[1] * inverse_length,
		direction[2] * inverse_length};
}

void applyLightControls(vve::RenderSystem render, const LightControls &lights) {
	const auto directional_color = vve::Vec3{
		lights.directional_color[0], lights.directional_color[1],
		lights.directional_color[2]};
	const auto directional_ambient = lights.directional_enabled
		? vve::Vec3{lights.directional_ambient[0], lights.directional_ambient[1],
				 lights.directional_ambient[2]}
		: vve::Vec3{0.0F, 0.0F, 0.0F};
	render.setDirectionalLight(
		vve::Direction{.value = normalizedDirection(lights.directional_direction)},
		vve::LinearColor{.value = directional_color},
		vve::LightIntensity{
			.value = lights.directional_enabled ? lights.directional_intensity : 0.0F},
		vve::LinearColor{.value = directional_ambient});

	const auto point_color =
		vve::Vec3{lights.point_color[0], lights.point_color[1], lights.point_color[2]};
	const auto point_ambient = lights.point_enabled
		? vve::Vec3{
				lights.point_ambient[0], lights.point_ambient[1], lights.point_ambient[2]}
		: vve::Vec3{0.0F, 0.0F, 0.0F};
	render.setPointLight(
		vve::Position{.value = vve::Vec3{
			lights.point_position[0], lights.point_position[1], lights.point_position[2]}},
		vve::LinearColor{.value = point_color},
		vve::LightIntensity{.value = lights.point_enabled ? lights.point_intensity : 0.0F},
		vve::LightRange{.value = lights.point_range},
		vve::LinearColor{.value = point_ambient});
}

[[nodiscard]] bool drawLightControls(LightControls &lights) {
	bool changed{};

	if (ImGui::CollapsingHeader("Directional light", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed |= ImGui::Checkbox("Enabled##Directional", &lights.directional_enabled);
		changed |= ImGui::DragFloat3(
			"Direction##Directional", lights.directional_direction.data(), 0.01F, -1.0F, 1.0F,
			"%.2f");
		changed |= ImGui::ColorEdit3(
			"Color##Directional", lights.directional_color.data());
		changed |= ImGui::SliderFloat(
			"Intensity##Directional", &lights.directional_intensity, 0.0F, 5.0F, "%.2f");
		changed |= ImGui::ColorEdit3(
			"Ambient##Directional", lights.directional_ambient.data());
	}

	if (ImGui::CollapsingHeader("Point light", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed |= ImGui::Checkbox("Enabled##Point", &lights.point_enabled);
		changed |= ImGui::DragFloat3(
			"Position##Point", lights.point_position.data(), 0.1F, -50.0F, 50.0F, "%.1f");
		changed |= ImGui::ColorEdit3("Color##Point", lights.point_color.data());
		changed |= ImGui::SliderFloat(
			"Intensity##Point", &lights.point_intensity, 0.0F, 20.0F, "%.2f");
		changed |=
			ImGui::SliderFloat("Range##Point", &lights.point_range, 0.5F, 100.0F, "%.1f");
		changed |= ImGui::ColorEdit3("Ambient##Point", lights.point_ambient.data());
	}

	if (ImGui::Button("Reset lighting")) {
		lights = LightControls{};
		changed = true;
	}
	return changed;
}

} // namespace

int main(int argc, char **argv) {
	std::cout << std::unitbuf;
	std::cerr << std::unitbuf;
	std::cout << "[physicsexample] engine=" << vve::engineImplementationNamespaceName << '\n';

	VPEWorld physics;
	const bool startup_stack = hasArgument(argc, argv, "--stack");
	const auto frame_limit = frameLimit(argc, argv);
	if (startup_stack && frame_limit) {
		return runHeadlessStack(physics, *frame_limit);
	}

	auto engine = vve::EngineBuilder<PhysicsTickSystem>{}
						 .applicationName("physicsexample")
						 .addWindow(vve::WindowSetup{}
										 .id("main")
										 .title("Vienna Physics Engine")
										 .extent(windowExtent)
										 .renderer(vve::RendererId{.value = "forward"})
										 .resizable(true))
						 .userSystems(vve::makeUserSystems(PhysicsTickSystem{.physics = &physics}))
						 .build();

	if (const auto result = engine.init(); !result) {
		std::cerr << "[physicsexample] engine init failed: error="
					 << vve::errorName(result.error()) << '\n';
		return 1;
	}
	auto render = engine.world().get<vve::RenderSystem>();
	render.clearScene();
	if (const auto plane = render.addPlane(
			vve::Vec2{500.0F, 500.0F},
			vve::LinearColor{.value = vve::Vec3{0.16F, 0.42F, 0.18F}});
		 !plane) {
		std::cerr << "[physicsexample] ground-plane creation failed: error="
					 << vve::errorName(plane.error()) << '\n';
		return 2;
	}
	LightControls lights;
	applyLightControls(render, lights);

	vve::DefaultCameraController camera;
	camera.eye = vve::Position{.value = vve::Vec3{0.0F, 6.0F, -14.0F}};
	const auto startup_forward =
		vve::math::normalize(vve::math::subtract(vve::Vec3{0.0F, 3.0F, 2.0F}, camera.eye.value));
	camera.yaw = std::atan2(startup_forward.x, -startup_forward.z);
	camera.pitch = std::asin(startup_forward.y);

	const auto on_move = makeMoveCallback(render);
	const auto on_erase = makeEraseCallback(render);
	const auto create_visual = makeVisualFactory(render, crateTexture());
	auto camera_position = [&camera] { return physicsVector(camera.eye.value); };
	auto camera_direction = [&camera] { return physicsVector(cameraForward(camera)); };
	ve::ConstraintDemos constraints{
		&physics, on_move, on_erase, create_visual, camera_position, camera_direction};

	std::default_random_engine random_generator{12345};
	std::uniform_real_distribution<real> random{0.0_real, 1.0_real};
	VPEWorld::callback_collide on_collide =
		[](std::shared_ptr<VPEWorld::Body> first, std::shared_ptr<VPEWorld::Body> second) {
			std::cout << "Collision " << first->m_name << ' ' << second->m_name << '\n';
		};
	real smoothed_fps{};
	if (startup_stack) {
		addStack(physics, create_visual, on_move, on_erase);
	}

	engine.world().get<vve::GuiSystem>().draw([&] {
		ImGui::SetNextWindowSize(ImVec2{470.0F, 680.0F}, ImGuiCond_FirstUseEver);
		if (ImGui::Begin("Physics Panel")) {
			if (ImGui::RadioButton("Solver A", physics.m_solver == 0)) physics.m_solver = 0;
			ImGui::SameLine();
			if (ImGui::RadioButton("Solver B", physics.m_solver == 1)) physics.m_solver = 1;

			ImGui::Text("Simulation frequency: %.1f", static_cast<double>(physics.m_sim_frequency));
			if (ImGui::Button("-10 Hz")) {
				physics.m_sim_frequency =
					std::max(10.0, physics.m_sim_frequency - 10.0);
				physics.m_sim_delta_time = 1.0_real / physics.m_sim_frequency;
			}
			ImGui::SameLine();
			if (ImGui::Button("+10 Hz")) {
				physics.m_sim_frequency += 10.0_real;
				physics.m_sim_delta_time = 1.0_real / physics.m_sim_frequency;
			}
			ImGui::SameLine();
			if (ImGui::Button("Next time slot")) {
				physics.m_current_time += physics.m_sim_delta_time;
			}

			if (ImGui::RadioButton(
					"Realtime",
					physics.m_mode == VPEWorld::simulation_mode_t::SIMULATION_MODE_REALTIME)) {
				physics.m_mode = VPEWorld::simulation_mode_t::SIMULATION_MODE_REALTIME;
			}
			ImGui::SameLine();
			if (ImGui::RadioButton(
					"Debug",
					physics.m_mode == VPEWorld::simulation_mode_t::SIMULATION_MODE_DEBUG)) {
				physics.m_mode = VPEWorld::simulation_mode_t::SIMULATION_MODE_DEBUG;
			}

			smoothed_fps = 0.05_real * physics.m_fps + 0.95_real * smoothed_fps;
			ImGui::Text("Time: %.5f s", physics.m_current_time);
			ImGui::Text("Physics FPS: %.1f", static_cast<double>(smoothed_fps));
			ImGui::Text("Bodies: %zu   Contacts: %zu   Active: %.1f",
							physics.m_bodies.size(), physics.m_contacts.size(),
							static_cast<double>(physics.m_num_active));

			ImGui::Text("Solver loops: %d", physics.m_loops);
			if (ImGui::Button("-5 loops")) physics.m_loops = std::max(5, physics.m_loops - 5);
			ImGui::SameLine();
			if (ImGui::Button("+5 loops")) physics.m_loops += 5;

			ImGui::Text("Resting factor: %.2f", static_cast<double>(physics.m_resting_factor));
			if (ImGui::Button("-0.2 resting")) {
				physics.m_resting_factor =
					std::max(0.2_real, physics.m_resting_factor - 0.2_real);
			}
			ImGui::SameLine();
			if (ImGui::Button("+0.2 resting")) physics.m_resting_factor += 0.2_real;

			ImGui::Text("Damping increment: %.2f", static_cast<double>(physics.m_damping_incr));
			if (ImGui::Button("-5 damping")) {
				physics.m_damping_incr =
					std::max(0.0_real, physics.m_damping_incr - 5.0_real);
			}
			ImGui::SameLine();
			if (ImGui::Button("+5 damping")) physics.m_damping_incr += 5.0_real;

			ImGui::Text("Position bias: %.2f", static_cast<double>(physics.m_pbias_factor));
			if (ImGui::Button("-0.1 bias")) {
				physics.m_pbias_factor =
					glm::clamp(physics.m_pbias_factor - 0.1_real, 0.0_real, 1.0_real);
			}
			ImGui::SameLine();
			if (ImGui::Button("+0.1 bias")) {
				physics.m_pbias_factor =
					glm::clamp(physics.m_pbias_factor + 0.1_real, 0.0_real, 1.0_real);
			}

			bool align_position_bias = physics.m_align_position_bias == 1;
			drawBooleanSetting("Align position bias", align_position_bias);
			physics.m_align_position_bias = align_position_bias ? 1 : 0;
			bool use_velocity_bias = physics.m_use_vbias == 1;
			drawBooleanSetting("Use velocity bias", use_velocity_bias);
			physics.m_use_vbias = use_velocity_bias ? 1 : 0;
			bool warmstart = physics.m_use_warmstart == 1;
			drawBooleanSetting("Warmstart all", warmstart);
			physics.m_use_warmstart = warmstart ? 1 : 0;
			bool warmstart_single = physics.m_use_warmstart_single == 1;
			drawBooleanSetting("Warmstart single", warmstart_single);
			physics.m_use_warmstart_single = warmstart_single ? 1 : 0;
			drawBooleanSetting("Deactivate resting bodies", physics.m_deactivate);
			bool clamp_position = physics.m_clamp_position == 1;
			drawBooleanSetting("Clamp position", clamp_position);
			physics.m_clamp_position = clamp_position ? 1 : 0;

			if (ImGui::Button("Create 20 bodies")) {
				addRandomBodies(physics, create_visual, on_move, on_erase, random_generator, random, 20);
			}
			ImGui::SameLine();
			if (ImGui::Button("Create stack")) {
				addStack(physics, create_visual, on_move, on_erase);
			}
			ImGui::SameLine();
			if (ImGui::Button("Clear bodies")) physics.clear();

			if (ImGui::Button("Pick body") && physics.m_bodies.size() != 0) {
				physics.m_body = physics.pickBody(camera_position(), camera_direction());
			}
			ImGui::SameLine();
			if (ImGui::Button("Delete body") && physics.m_bodies.size() != 0) {
				if (auto body = physics.pickBody(camera_position(), camera_direction())) {
					physics.eraseBody(body);
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Add collider") && physics.m_body) {
				physics.addCollider(physics.m_body, on_collide);
			}
			if (ImGui::Button("Remove colliders")) physics.clearCollider();

			if (physics.m_body) {
				ImGui::Text("Current body: %s", physics.m_body->m_name.c_str());
				const real step = 5.0_real * static_cast<real>(physics.m_sim_delta_time);
				glmvec3 translation{0.0_real};
				glmvec3 rotation{0.0_real};
				if (ImGui::Button("+X")) translation.x = step;
				ImGui::SameLine();
				if (ImGui::Button("-X")) translation.x = -step;
				ImGui::SameLine();
				if (ImGui::Button("+Y")) translation.y = step;
				ImGui::SameLine();
				if (ImGui::Button("-Y")) translation.y = -step;
				ImGui::SameLine();
				if (ImGui::Button("+Z")) translation.z = step;
				ImGui::SameLine();
				if (ImGui::Button("-Z")) translation.z = -step;
				if (ImGui::Button("+RX")) rotation.x = step;
				ImGui::SameLine();
				if (ImGui::Button("-RX")) rotation.x = -step;
				ImGui::SameLine();
				if (ImGui::Button("+RY")) rotation.y = step;
				ImGui::SameLine();
				if (ImGui::Button("-RY")) rotation.y = -step;
				ImGui::SameLine();
				if (ImGui::Button("+RZ")) rotation.z = step;
				ImGui::SameLine();
				if (ImGui::Button("-RZ")) rotation.z = -step;

				physics.m_body->m_positionW += translation;
				physics.m_body->m_orientationLW =
					glm::rotate(glmquat{1, 0, 0, 0}, rotation.x, glmvec3{1, 0, 0}) *
					glm::rotate(glmquat{1, 0, 0, 0}, rotation.y, glmvec3{0, 1, 0}) *
					glm::rotate(glmquat{1, 0, 0, 0}, rotation.z, glmvec3{0, 0, 1}) *
					physics.m_body->m_orientationLW;
				physics.m_body->updateMatrices();
				on_move(0.0, physics.m_body);
			}
		}
		ImGui::End();

		ImGui::SetNextWindowSize(ImVec2{430.0F, 390.0F}, ImGuiCond_FirstUseEver);
		if (ImGui::Begin("Light Controls")) {
			if (drawLightControls(lights)) {
				applyLightControls(render, lights);
			}
		}
		ImGui::End();

		ImGui::SetNextWindowSize(ImVec2{420.0F, 150.0F}, ImGuiCond_FirstUseEver);
		if (ImGui::Begin("Constraint Demos")) {
			if (ImGui::Button("Ball-Socket")) constraints.ballSocketJoint();
			ImGui::SameLine();
			if (ImGui::Button("Hinge")) constraints.hingeJoint();
			ImGui::SameLine();
			if (ImGui::Button("Slider")) constraints.sliderJoint();
			ImGui::SameLine();
			if (ImGui::Button("Fixed")) constraints.fixedJoint();
			if (ImGui::Button("Bridge")) constraints.bridge();
			ImGui::SameLine();
			if (ImGui::Button("Ragdoll")) constraints.ragdoll();
			ImGui::SameLine();
			if (ImGui::Button("Cannon")) constraints.sliderCannon();
			ImGui::SameLine();
			if (ImGui::Button("Wheel")) constraints.wheel();
			ImGui::SameLine();
			if (ImGui::Button("Chain")) constraints.hingeChain();
		}
		ImGui::End();
	});

	const int max_frames = frame_limit.value_or(0);
	int frame{};
	bool running = true;
	while (running && (max_frames == 0 || frame < max_frames)) {
		const auto frame_input = engine.world().get<vve::WindowSystem>().input();
		render.setCamera(camera.update(frame_input), windowExtent);

		const auto status = engine.step();
		if (!status) {
			std::cerr << "[physicsexample] frame failed: error="
						 << vve::errorName(status.error()) << '\n';
			physics.clear();
			return 3;
		}
		++frame;
		if (*status == vve::FrameStatus::stopped) break;

		const auto input = engine.world().get<vve::WindowSystem>().input();
		if (input.wasKeyPressed(vve::Key::escape)) running = false;
		if (input.wasKeyPressed(static_cast<std::int32_t>('b'))) {
			const auto direction = camera_direction();
			const glmvec3 velocity =
				(30.0_real + 5.0_real * random(random_generator)) * direction /
				glm::length(direction);
			const real angle = random(random_generator) * 10.0_real * 3.0_real * pi / 180.0_real;
			const glmvec3 axis = glm::normalize(glmvec3{
				random(random_generator), random(random_generator), random(random_generator)});
			(void)addCube(physics, create_visual, on_move, on_erase, glmvec3{1.0_real},
						camera_position() + 2.0_real * direction,
						glm::rotate(glmquat{1, 0, 0, 0}, angle, axis), velocity,
						glmvec3{random(random_generator) * 5.0_real,
								 random(random_generator) * 5.0_real,
								 random(random_generator) * 5.0_real});
		}
		if (input.wasKeyPressed(static_cast<std::int32_t>('y'))) {
			addStack(physics, create_visual, on_move, on_erase);
		}
		if (input.wasKeyPressed(static_cast<std::int32_t>('z'))) {
			(void)addCube(physics, create_visual, on_move, on_erase, glmvec3{1.0_real},
						camera_position() + 2.0_real * camera_direction());
		}
		if (input.wasKeyPressed(32)) {
			static real height = 0.5_real;
			const glmvec3 position = camera_position();
			(void)addCube(physics, create_visual, on_move, on_erase, glmvec3{1.0_real},
						glmvec3{position.x, height++, position.z + 4.0_real});
		}
	}

	if (startup_stack) {
		const bool stable = std::ranges::all_of(physics.m_bodies, [](const auto &entry) {
			const auto &position = entry.second->m_positionW;
			return std::isfinite(position.x) && std::isfinite(position.y) &&
					 std::isfinite(position.z) && glm::length(position) < 1000.0_real;
		});
		std::cout << "[physicsexample] stack=" << (stable ? "stable" : "unstable")
					 << " bodies=" << physics.m_bodies.size() << '\n';
		if (!stable) {
			physics.clear();
			return 4;
		}
	}
	physics.clear();
	std::cout << "[physicsexample] frames=" << frame << '\n';
	return 0;
}
