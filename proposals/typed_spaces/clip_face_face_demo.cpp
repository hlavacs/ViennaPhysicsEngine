/// clip_face_face_demo.cpp — PROPOSAL: VPE.hpp's clipFaceFace() and
/// createFaceContact() rewritten with typed coordinate spaces (vpe_spaces.hpp)
/// instead of the ITORTP/RTTOWP/WTOTIP/... macros.
///
/// Standalone: uses small mock structs mirroring VPE's Face/Contact so the
/// prototype compiles and runs without touching VPE.hpp. Compare side by side
/// with VPE.hpp lines ~1636-1698.
///
/// Build:  g++ -std=c++20 -I extern/glm proposals/typed_spaces/clip_face_face_demo.cpp -o demo
///
/// Uncomment the block at the bottom of main() to see the compiler reject
/// wrong-space arithmetic — the whole point of the exercise.

#include "vpe_spaces.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <ranges>
#include <vector>

using namespace vpe::spaces;

// ---------------------------------------------------------------------------
// Mock geometry (mirrors VPE::Polytope::Face and VPE::Contact minimally)
// ---------------------------------------------------------------------------

struct MockFace {
    glmvec3              m_normalL;        // face normal, body-local space
    glmmat4              m_LtoT;           // body-local -> face tangent space
    glmmat4              m_TtoL;           // face tangent -> body-local space
    std::vector<glmvec3> m_face_vertexL;   // face vertices, body-local space
    std::vector<glm::vec2> m_face_vertex2D_T; // same, in tangent space (x,z)
};

struct MockContact {
    ContactFrame m_frame;                  // replaces BodyPtr::m_to_other(_it)
    std::vector<glmvec3> m_pointsW;        // collected contact points (world)
    std::vector<real>    m_dists;
};

static constexpr real collision_margin = real(0.01);

void addContactPoint(MockContact& contact, Point<Space::World> posW,
                     Normal<Space::World> /*normalW*/, real dist) {
    contact.m_pointsW.push_back(posW.v);
    contact.m_dists.push_back(dist);
}

// Sutherland-Hodgman clipping, unchanged from VPE (2D tangent space is a
// single space, so no tagging needed there).
void sutherlandHodgman(const std::vector<glm::vec2>& subject,
                       const std::vector<glm::vec2>& clip,
                       std::vector<glm::vec2>& out) {
    out = subject;
    auto n = clip.size();
    real winding = real(0);   // signed area sign -> orientation-agnostic clipping
    for (size_t i = 0; i < n; ++i) {
        glm::vec2 a = clip[i], b = clip[(i + 1) % n];
        winding += a.x * b.y - b.x * a.y;
    }
    real sign = winding >= 0 ? real(1) : real(-1);
    for (size_t i = 0; i < n; ++i) {
        glm::vec2 a = clip[i], b = clip[(i + 1) % n];
        glm::vec2 e = b - a;
        auto inside = [&](glm::vec2 p) { return sign * (e.x * (p.y - a.y) - e.y * (p.x - a.x)) >= 0; };
        std::vector<glm::vec2> input = std::move(out);
        out.clear();
        for (size_t j = 0; j < input.size(); ++j) {
            glm::vec2 cur = input[j], prev = input[(j + input.size() - 1) % input.size()];
            glm::vec2 d = cur - prev;
            real denom = e.x * d.y - e.y * d.x;
            if (inside(cur)) {
                if (!inside(prev) && std::fabs(denom) > real(1e-9)) {
                    real t = (e.x * (a.y - prev.y) - e.y * (a.x - prev.x)) / -denom;
                    out.push_back(prev + t * d);
                }
                out.push_back(cur);
            } else if (inside(prev) && std::fabs(denom) > real(1e-9)) {
                real t = (e.x * (a.y - prev.y) - e.y * (a.x - prev.x)) / -denom;
                out.push_back(prev + t * d);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// THE REWRITE — compare with VPE.hpp clipFaceFace() (lines ~1652-1698)
// ---------------------------------------------------------------------------

/// Original signature: real clipFaceFace(Contact&, Face* face_ref, Face* face_inc)
/// Every macro call site is annotated with the macro it replaces.
real clipFaceFace(MockContact& contact, const MockFace& face_ref, const MockFace& face_inc) {
    const ContactFrame& frame = contact.m_frame;

    // Face tangent transforms, built from the matrices the faces already store.
    auto refT = faceTangent<Space::Ref, Space::RefTangent>(face_ref.m_LtoT);
    auto incT = faceTangent<Space::Inc, Space::IncTangent>(face_inc.m_LtoT);
    auto refTinv = Transform<Space::RefTangent, Space::Ref>{face_ref.m_TtoL};
    auto incTinv = Transform<Space::IncTangent, Space::Inc>{face_inc.m_TtoL};

    // Composite transform: incident-local -> reference tangent space.
    // The macro version rebuilt this matrix product at every vertex; here it
    // is composed once outside the loop (a small win the macros hid).
    auto incToRefT = refT * frame.incToRef;                      // was: ITORTP

    std::vector<glm::vec2> points; // projected contact points, 2D ref tangent space
    for (const auto& vL : face_inc.m_face_vertexL) {
        Point<Space::RefTangent> pT = incToRefT(Point<Space::Inc>{vL}); // was: ITORTP(vertex->m_positionL)
        points.emplace_back(pT.v.x, pT.v.z);
    }

    std::vector<glm::vec2> newPolygon;
    sutherlandHodgman(points, face_ref.m_face_vertex2D_T, newPolygon);

    if (newPolygon.size() > 4) {   // reduce to the 4 points spanning max area (unchanged)
        auto support = [&](glm::vec2 dir) {
            return *std::ranges::max_element(newPolygon, [&](auto& a, auto& b) {
                return glm::dot(dir, a) < glm::dot(dir, b); });
        };
        std::vector<glm::vec2> dirs{{0,1},{1,0},{0,-1},{-1,0},{1,1},{-1,1},{-1,-1},{1,-1}};
        std::vector<glm::vec2> supp;
        for (auto d : dirs) supp.push_back(support(d));

        auto area2 = [](glm::vec2 a, glm::vec2 b, glm::vec2 c, glm::vec2 d) {
            return std::fabs(glm::determinant(glmmat3{{a.x,a.y,1},{b.x,b.y,1},{c.x,c.y,1}})
                           + glm::determinant(glmmat3{{a.x,a.y,1},{b.x,b.y,1},{d.x,d.y,1}}));
        };
        if (area2(supp[0],supp[1],supp[2],supp[3]) > area2(supp[4],supp[5],supp[6],supp[7]))
            newPolygon = {supp[0], supp[1], supp[2], supp[3]};
        else
            newPolygon = {supp[4], supp[5], supp[6], supp[7]};
    }

    // Reference face normal in world space, computed once outside the loop
    // (the macro version evaluated RTOWN twice per point).
    Normal<Space::World> nW = frame.refToWorld(Normal<Space::Ref>{face_ref.m_normalL}); // was: RTOWN

    real minSep = real(0);
    for (const auto& p2D : newPolygon) {
        // No "cannot put comma into macro" workaround needed anymore:
        Point<Space::RefTangent> pT{{p2D.x, real(0), p2D.y}};

        Point<Space::World> posRW = frame.refToWorld(refTinv(pT));       // was: RTTOWP(p)
        Point<Space::IncTangent> posIT = incT(frame.worldToInc(posRW)); // was: WTOTIP(posRW)
        posIT.v.y = real(0);                       // project onto incident face
        Point<Space::World> posIW = frame.incToWorld(incTinv(posIT));   // was: ITTOWP(posIT)

        real dist = dot(posIW - posRW, nW);        // spaces checked: both World
        if (dist < collision_margin) {
            minSep = std::min(minSep, dist);
            addContactPoint(contact, posRW, nW, dist);
        }
    }
    return minSep;
}

/// Original: void createFaceContact(Contact&, FaceQuery&)  (VPE.hpp ~1636)
/// Shown for the normal-transform path: -RTOIN(fq.m_face_ref->m_normalL).
const MockFace* bestIncidentFace(const MockContact& contact, const MockFace& face_ref,
                                 const std::vector<MockFace>& inc_faces) {
    Normal<Space::Inc> An = normalize(
        Normal<Space::Inc>{-contact.m_frame.refToInc(Normal<Space::Ref>{face_ref.m_normalL}).v}); // was: -RTOIN(...)

    const MockFace* best = nullptr;
    real bestDot = -std::numeric_limits<real>::max();
    for (const auto& f : inc_faces) {   // was: maxFaceAlignment
        real d = dot(Vec<Space::Inc>{f.m_normalL}, An);
        if (d > bestDot) { bestDot = d; best = &f; }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Smoke test: two unit cubes, incident cube 0.995 above the reference cube
// -> 4 contact points at y ~ 0.5, distance ~ -0.005 (touching within margin).
// ---------------------------------------------------------------------------

MockFace makeTopFace() {   // +y face of a unit cube, in body-local space
    MockFace f;
    f.m_normalL = {0, 1, 0};
    f.m_face_vertexL = {{-0.5f,0.5f,-0.5f},{-0.5f,0.5f,0.5f},{0.5f,0.5f,0.5f},{0.5f,0.5f,-0.5f}};
    glmvec3 tangent{0,0,1}, bitangent{1,0,0};
    f.m_TtoL = glm::translate(glmmat4(1), f.m_face_vertexL[0]) * glmmat4{glmmat3{bitangent, f.m_normalL, tangent}};
    f.m_LtoT = glm::inverse(f.m_TtoL);
    for (auto& v : f.m_face_vertexL) {
        glmvec4 pt = f.m_LtoT * glmvec4{v, 1.0f};
        f.m_face_vertex2D_T.emplace_back(pt.x, pt.z);
    }
    return f;
}

MockFace makeBottomFace() { // -y face
    MockFace f = makeTopFace();
    f.m_normalL = {0, -1, 0};
    for (auto& v : f.m_face_vertexL) v.y = -0.5f;
    glmvec3 tangent{0,0,1}, bitangent{-1,0,0};
    f.m_TtoL = glm::translate(glmmat4(1), f.m_face_vertexL[0]) * glmmat4{glmmat3{bitangent, f.m_normalL, tangent}};
    f.m_LtoT = glm::inverse(f.m_TtoL);
    f.m_face_vertex2D_T.clear();
    for (auto& v : f.m_face_vertexL) {
        glmvec4 pt = f.m_LtoT * glmvec4{v, 1.0f};
        f.m_face_vertex2D_T.emplace_back(pt.x, pt.z);
    }
    return f;
}

int main() {
    glmmat4 modelRef = glmmat4(1);                                        // ref cube at origin
    glmmat4 modelInc = glm::translate(glmmat4(1), glmvec3{0, 0.995f, 0}); // inc cube just touching

    MockContact contact{ContactFrame{modelRef, modelInc}, {}, {}};
    MockFace refTop = makeTopFace();
    std::vector<MockFace> incFaces{makeBottomFace()};

    const MockFace* inc = bestIncidentFace(contact, refTop, incFaces);
    real sep = clipFaceFace(contact, refTop, *inc);

    std::cout << "contact points: " << contact.m_pointsW.size()
              << "  min separation: " << sep << "\n";
    for (size_t i = 0; i < contact.m_pointsW.size(); ++i) {
        auto& p = contact.m_pointsW[i];
        std::cout << "  (" << p.x << ", " << p.y << ", " << p.z << ")  dist " << contact.m_dists[i] << "\n";
    }

    // ---- What the type system now catches (uncomment to see the errors) ----
    // ContactFrame& frame = contact.m_frame;
    // Point<Space::Inc> pI{{0,0,0}};
    // frame.refToWorld(pI);                          // error: wrong source space
    // Point<Space::World> pW{{0,0,0}};
    // frame.refToWorld(Normal<Space::Ref>{{0,1,0}}); // ok — but returns Normal, so:
    // Point<Space::World> q = frame.refToWorld(Normal<Space::Ref>{{0,1,0}}); // error: normal is not a point
    // dot(pW - pW, Normal<Space::Ref>{{0,1,0}});     // error: mixing World and Ref

    return contact.m_pointsW.size() == 4 && sep < 0 ? 0 : 1;
}
