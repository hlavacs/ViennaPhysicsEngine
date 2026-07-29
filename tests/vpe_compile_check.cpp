#include "VPE.hpp"

int main() {
	vpe::VPEWorld physics;
	physics.tick(1.0 / 60.0);
	return physics.m_bodies.size() == 0 ? 0 : 1;
}
