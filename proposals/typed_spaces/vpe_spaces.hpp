#pragma once

/// vpe_spaces.hpp — PROPOSAL: strongly typed coordinate spaces for VPE.
///
/// Replaces the 17 transform macros in VPE.hpp (ITORP, RTOWN, WTOTIP, ...).
/// The macro naming convention (R=reference, I=incident, T=tangent, W=world;
/// P=point, V=vector, N=normal) is enforced here by the type system instead
/// of by discipline: applying a transform to a value from the wrong space,
/// or treating a normal like a point, is a compile error.
///
/// Zero runtime overhead: Point/Vec/Normal are aggregates holding one glmvec3;
/// Transform holds the same matrices the Contact/Face structs already store.
///
/// Macro -> typed equivalent (frame is a ContactFrame, see below):
///   ITORP(X)  -> frame.incToRef(Point<Space::Inc>{X})
///   ITORV(X)  -> frame.incToRef(Vec<Space::Inc>{X})
///   ITORN(X)  -> frame.incToRef(Normal<Space::Inc>{X})
///   ITORTP(X) -> refTangent(frame.incToRef(Point<Space::Inc>{X}))
///   ITTOWP(X) -> frame.incToWorld(incTangent.inv()(Point<Space::IncTangent>{X}))
///   ITOWP(X)  -> frame.incToWorld(Point<Space::Inc>{X})
///   ITOWN(X)  -> frame.incToWorld(Normal<Space::Inc>{X})
///   RTOIP(X)  -> frame.refToInc(Point<Space::Ref>{X})
///   RTOIN(X)  -> frame.refToInc(Normal<Space::Ref>{X})
///   RTOWP(X)  -> frame.refToWorld(Point<Space::Ref>{X})
///   RTOWN(X)  -> frame.refToWorld(Normal<Space::Ref>{X})
///   RTORTP(X) -> refTangent(Point<Space::Ref>{X})
///   WTOTIP(X) -> incTangent(frame.worldToInc(Point<Space::World>{X}))
///   WTORN(X)  -> frame.worldToRef(Normal<Space::World>{X})
///   WTOIN(X)  -> frame.worldToInc(Normal<Space::World>{X})
///   RTTORP(X) -> refTangent.inv()(Point<Space::RefTangent>{X})
///   RTTOWP(X) -> frame.refToWorld(refTangent.inv()(Point<Space::RefTangent>{X}))

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>

namespace vpe::spaces {

// Use VPE's real/glm typedefs in the real integration; standalone here.
using real    = float;
using glmvec3 = glm::vec3;
using glmvec4 = glm::vec4;
using glmmat3 = glm::mat3;
using glmmat4 = glm::mat4;

/// The coordinate spaces that occur during narrowphase / contact generation.
enum class Space {
    World,      ///< world space
    Ref,        ///< local space of the reference body
    Inc,        ///< local space of the incident body
    RefTangent, ///< tangent space of the reference face (y = face normal)
    IncTangent  ///< tangent space of the incident face
};

/// A position. Transforms with the full 4x4 (rotation + translation).
template <Space S>
struct Point {
    glmvec3 v;
};

/// A direction / displacement. Transforms with the upper 3x3 only.
template <Space S>
struct Vec {
    glmvec3 v;
};

/// A surface normal. Transforms with the inverse transpose of the 3x3.
template <Space S>
struct Normal {
    glmvec3 v;
};

// --- operations that only make sense within ONE space -----------------------

template <Space S> constexpr Vec<S> operator-(Point<S> a, Point<S> b) { return {a.v - b.v}; }
template <Space S> constexpr Point<S> operator+(Point<S> p, Vec<S> d)  { return {p.v + d.v}; }
template <Space S> constexpr Vec<S> operator-(Vec<S> x)                { return {-x.v}; }

template <Space S> inline real dot(Vec<S> a, Vec<S> b)     { return glm::dot(a.v, b.v); }
template <Space S> inline real dot(Vec<S> a, Normal<S> n)  { return glm::dot(a.v, n.v); }
template <Space S> inline Vec<S> cross(Vec<S> a, Vec<S> b) { return {glm::cross(a.v, b.v)}; }
template <Space S> inline Normal<S> normalize(Normal<S> n) { return {glm::normalize(n.v)}; }

/// A transform between two named spaces. Composition and inversion keep the
/// space tags consistent; there is no way to build a Ref->World transform and
/// accidentally apply it to an incident-space point.
template <Space From, Space To>
class Transform {
public:
    /// From a 4x4 point transform; the normal transform (inverse transpose
    /// of the 3x3) is derived once, exactly as VPE does for m_to_other_it.
    explicit Transform(const glmmat4& m)
        : m_m{m}, m_it{glm::inverseTranspose(glmmat3{m})} {}

    Transform(const glmmat4& m, const glmmat3& it) : m_m{m}, m_it{it} {}

    Point<To>  operator()(Point<From> p) const { return {glmvec3{m_m * glmvec4{p.v, real(1)}}}; }
    Vec<To>    operator()(Vec<From> x)   const { return {glmmat3{m_m} * x.v}; }
    Normal<To> operator()(Normal<From> n) const { return {m_it * n.v}; }

    /// The reverse transform (named inv, not inverse, to avoid glm confusion).
    Transform<To, From> inv() const { return Transform<To, From>{glm::inverse(m_m)}; }

    const glmmat4& matrix() const { return m_m; }

private:
    glmmat4 m_m;  ///< point/vector transform
    glmmat3 m_it; ///< inverse transpose, for normals
};

/// Composition: (b_to_c * a_to_b) is an a_to_c. Spaces must chain.
template <Space A, Space B, Space C>
Transform<A, C> operator*(const Transform<B, C>& bc, const Transform<A, B>& ab) {
    return Transform<A, C>{bc.matrix() * ab.matrix()};
}

/// All transforms needed while processing one contact pair. Built once per
/// narrowphase test (replaces the m_to_other / m_to_other_it fields plus the
/// implicit m_model / m_model_inv chains inside the macros).
struct ContactFrame {
    Transform<Space::Ref, Space::World> refToWorld;
    Transform<Space::Inc, Space::World> incToWorld;
    Transform<Space::World, Space::Ref> worldToRef;
    Transform<Space::World, Space::Inc> worldToInc;
    Transform<Space::Inc, Space::Ref>   incToRef;
    Transform<Space::Ref, Space::Inc>   refToInc;

    /// modelRef/modelInc are the bodies' model matrices (m_model).
    ContactFrame(const glmmat4& modelRef, const glmmat4& modelInc)
        : refToWorld{modelRef},
          incToWorld{modelInc},
          worldToRef{refToWorld.inv()},
          worldToInc{incToWorld.inv()},
          incToRef{worldToRef * incToWorld},
          refToInc{worldToInc * refToWorld} {}

    /// Swapping reference and incident body = swapping the frame roles.
    ContactFrame swapped() const {
        return ContactFrame{incToWorld.matrix(), refToWorld.matrix()};
    }
};

/// Tangent-space transform of a face, from the face's m_LtoT matrix.
/// BodySpace is Space::Ref or Space::Inc, TangentSpace the matching tangent space.
template <Space BodySpace, Space TangentSpace>
Transform<BodySpace, TangentSpace> faceTangent(const glmmat4& LtoT) {
    return Transform<BodySpace, TangentSpace>{LtoT};
}

} // namespace vpe::spaces
