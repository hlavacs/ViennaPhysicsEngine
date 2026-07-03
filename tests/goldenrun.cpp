/// goldenrun.cpp — deterministic golden-run regression tests (REFACTORING.md, Phase 0).
///
/// Runs fixed scenes for a fixed number of simulation steps in DEBUG mode
/// (no wall-clock dependence) and serializes all body/cloth states.
///
///   goldenrun --generate <dir>          write golden files to <dir>
///   goldenrun --compare  <dir> [eps]    compare current run against golden
///                                       files; eps = max abs/rel deviation
///                                       per component (default 0 = exact)
///
/// Purpose: every refactoring phase must leave these runs unchanged (with the
/// same compiler/flags). Regenerate goldens ONLY when a behavior change is
/// intended and understood.

#include "VPE.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr int c_stack_steps = 600;   // 10 s at 60 Hz
constexpr int c_joint_steps = 600;
constexpr int c_cloth_steps = 300;

std::shared_ptr<vpe::VPEWorld::Body> makeBody(vpe::VPEWorld& world, void* owner,
	const std::string& name, glmvec3 position, real mass_inv,
	glmquat orientation = { 1, 0, 0, 0 }) {
	auto body = std::make_shared<vpe::VPEWorld::Body>(
		&world, name, owner, &world.g_cube, glmvec3{ 1.0_real }, position, orientation,
		glmvec3{ 0.0_real }, glmvec3{ 0.0_real }, mass_inv,
		world.m_restitution, world.m_friction);
	if (mass_inv > 0.0_real) {	//gravity acts on dynamic bodies only
		body->setForce(0, vpe::VPEWorld::Force{ { 0.0_real, world.c_gravity, 0.0_real } });
	}
	world.addBody(body);
	return body;
}

/// Advance the world exactly one simulation slot, wall-clock independent.
void step(vpe::VPEWorld& world, int i) {
	world.m_current_time = (i + 1.5) * world.m_sim_delta_time; //strictly past slot i+1, before slot i+2
	world.tick(0.0);
}

void dumpBodies(std::ostream& os, const vpe::VPEWorld& world) {
	os << std::setprecision(std::numeric_limits<real>::max_digits10);
	for (const auto& entry : world.m_bodies) {	//MapWrapper preserves insertion order -> deterministic
		const auto& b = *entry.second;
		os << "body " << b.m_name
			<< " " << b.m_positionW.x << " " << b.m_positionW.y << " " << b.m_positionW.z
			<< " " << b.m_orientationLW.w << " " << b.m_orientationLW.x
			<< " " << b.m_orientationLW.y << " " << b.m_orientationLW.z
			<< " " << b.m_linear_velocityW.x << " " << b.m_linear_velocityW.y << " " << b.m_linear_velocityW.z
			<< " " << b.m_angular_velocityW.x << " " << b.m_angular_velocityW.y << " " << b.m_angular_velocityW.z
			<< "\n";
	}
}

//---------------------------------------------------------------------------
// Scenes. Each returns the serialized end state.
//---------------------------------------------------------------------------

/// Five dynamic cubes stacked on the built-in ground, small lateral offsets.
/// Exercises: broadphase, SAT face contacts, warm starting, friction, resting.
std::string sceneStack() {
	vpe::VPEWorld world;
	world.m_mode = vpe::VPEWorld::SIMULATION_MODE_DEBUG;

	std::vector<int> owners(5);
	for (int i = 0; i < 5; ++i) {
		makeBody(world, &owners[i], "cube" + std::to_string(i),
			glmvec3{ 0.01_real * i, 0.5_real + 1.1_real * i, 0.005_real * i }, 1.0_real);
	}
	for (int i = 0; i < c_stack_steps; ++i) { step(world, i); }

	std::ostringstream os;
	dumpBodies(os, world);
	return os.str();
}

/// A dynamic cube swinging from a static cube via a ball-socket joint.
/// Exercises: BallSocketJoint setUp/solveVelocity, infinite-mass handling.
std::string scenePendulum() {
	vpe::VPEWorld world;
	world.m_mode = vpe::VPEWorld::SIMULATION_MODE_DEBUG;

	int o1 = 0, o2 = 0;
	auto anchor = makeBody(world, &o1, "anchor", glmvec3{ 0.0_real, 6.0_real, 0.0_real }, 0.0_real);
	auto bob    = makeBody(world, &o2, "bob",    glmvec3{ 2.0_real, 6.0_real, 0.0_real }, 1.0_real);
	world.addConstraint(std::make_shared<vpe::VPEWorld::BallSocketJoint>(
		anchor, bob, glmvec3{ 0.0_real, 6.0_real, 0.0_real }));

	for (int i = 0; i < c_joint_steps; ++i) { step(world, i); }

	std::ostringstream os;
	dumpBodies(os, world);
	return os.str();
}

/// A dynamic cube on a hinge attached to a static cube, swinging under gravity.
/// Exercises: HingeJoint rotational constraints.
std::string sceneHinge() {
	vpe::VPEWorld world;
	world.m_mode = vpe::VPEWorld::SIMULATION_MODE_DEBUG;

	int o1 = 0, o2 = 0;
	auto post = makeBody(world, &o1, "post", glmvec3{ 0.0_real, 6.0_real, 0.0_real }, 0.0_real);
	auto door = makeBody(world, &o2, "door", glmvec3{ 2.0_real, 6.0_real, 0.0_real }, 1.0_real);
	world.addConstraint(std::make_shared<vpe::VPEWorld::HingeJoint>(
		post, door, glmvec3{ 1.0_real, 6.0_real, 0.0_real }, glmvec3{ 0.0_real, 0.0_real, 1.0_real }));

	for (int i = 0; i < c_joint_steps; ++i) { step(world, i); }

	std::ostringstream os;
	dumpBodies(os, world);
	return os.str();
}

/// A 5x5 cloth sheet fixed at its two top corners, swinging under gravity.
/// Exercises: XPBD cloth constraints, ground handling.
std::string sceneCloth() {
	vpe::VPEWorld world;
	world.m_mode = vpe::VPEWorld::SIMULATION_MODE_DEBUG;

	constexpr int N = 5;	//N x N vertices
	std::vector<glmvec3> vertices;
	for (int y = 0; y < N; ++y) {
		for (int x = 0; x < N; ++x) {
			vertices.emplace_back(0.5_real * x, 3.0_real + 0.5_real * y, 0.0_real);
		}
	}
	std::vector<uint32_t> indices;
	for (int y = 0; y < N - 1; ++y) {
		for (int x = 0; x < N - 1; ++x) {
			uint32_t i0 = y * N + x, i1 = i0 + 1, i2 = i0 + N, i3 = i2 + 1;
			indices.insert(indices.end(), { i0, i1, i2, i1, i3, i2 });
		}
	}
	std::vector<glmvec3> fixed{ vertices[(N - 1) * N], vertices[N * N - 1] }; //top corners

	int owner = 0;
	auto cloth = std::make_shared<vpe::VPEWorld::Cloth>(
		&world, "cloth", &owner, nullptr, nullptr,
		vertices, indices, fixed, 1.0_real, 4, 0.8_real);
	world.addCloth(cloth);

	for (int i = 0; i < c_cloth_steps; ++i) { step(world, i); }

	std::ostringstream os;
	os << std::setprecision(std::numeric_limits<real>::max_digits10);
	int idx = 0;
	for (const auto& v : cloth->generateVertices()) {
		os << "vertex " << idx++ << " " << v.x << " " << v.y << " " << v.z << "\n";
	}
	return os.str();
}

//---------------------------------------------------------------------------
// Generate / compare
//---------------------------------------------------------------------------

struct Scene { const char* name; std::string (*run)(); };
const Scene c_scenes[] = {
	{ "stack",    sceneStack },
	{ "pendulum", scenePendulum },
	{ "hinge",    sceneHinge },
	{ "cloth",    sceneCloth },
};

/// Token-wise compare: non-numeric tokens must match exactly, numeric tokens
/// within eps (absolute or relative). eps 0 = exact numeric equality.
bool compareStreams(const std::string& name, std::istream& golden, std::istream& current, double eps) {
	std::string gtok, ctok;
	size_t n = 0;
	while (true) {
		bool g = static_cast<bool>(golden >> gtok), c = static_cast<bool>(current >> ctok);
		if (g != c) {
			std::cerr << "[" << name << "] length mismatch after " << n << " tokens\n";
			return false;
		}
		if (!g) { return true; }
		++n;
		if (gtok == ctok) { continue; }
		char* gend = nullptr; char* cend = nullptr;
		double gv = std::strtod(gtok.c_str(), &gend), cv = std::strtod(ctok.c_str(), &cend);
		bool numeric = gend != gtok.c_str() && *gend == '\0' && cend != ctok.c_str() && *cend == '\0';
		double tol = eps * std::max({ 1.0, std::fabs(gv), std::fabs(cv) });
		if (!numeric || std::fabs(gv - cv) > tol) {
			std::cerr << "[" << name << "] token " << n << " differs: golden '" << gtok
				<< "' vs current '" << ctok << "'\n";
			return false;
		}
	}
}

} // namespace

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cerr << "usage: goldenrun --generate <dir> | --compare <dir> [eps]\n";
		return 2;
	}
	const std::string mode = argv[1], dir = argv[2];
	const double eps = argc > 3 ? std::atof(argv[3]) : 0.0;

	bool ok = true;
	for (const auto& scene : c_scenes) {
		const std::string path = dir + "/" + scene.name + ".golden.txt";
		const std::string state = scene.run();

		if (mode == "--generate") {
			std::ofstream out{ path };
			if (!out) { std::cerr << "cannot write " << path << "\n"; return 2; }
			out << state;
			std::cout << "wrote " << path << "\n";
		}
		else if (mode == "--compare") {
			std::ifstream golden{ path };
			if (!golden) { std::cerr << "missing golden file " << path << "\n"; ok = false; continue; }
			std::istringstream current{ state };
			if (compareStreams(scene.name, golden, current, eps)) {
				std::cout << "[" << scene.name << "] OK\n";
			}
			else { ok = false; }
		}
		else { std::cerr << "unknown mode " << mode << "\n"; return 2; }
	}
	return ok ? 0 : 1;
}
