
#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <array>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>

#include "gjk_epa.h"
#include "collider.h"
#include "collision.h"
#include "contact.h"
#include "test.h"

using namespace vpe;


int main() {

	unit_test_normals();
	unit_test_collision();
	unit_test_contacts();



}


