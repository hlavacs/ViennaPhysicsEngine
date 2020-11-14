#pragma once


template<typename T>
concept Hashable = requires(T a) {
	{ std::hash<T>{}(a) }->std::convertible_to<std::size_t>;
};

template <Hashable T>
inline auto hash_combine(std::size_t& seed, T v) {
	seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	return seed;
}

 template<>
 struct std::hash<vec3> {
    size_t operator()(const vec3& v) {
        std::size_t seed = std::hash<float>()(v.x);
        hash_combine(seed, v.y);
        return hash_combine(seed, v.z);
    }
 };


 template<typename T1, typename T2>
 struct std::hash<std::pair<T1,T2>> {
    size_t operator()(const std::pair<T1,T2>& p) {
        std::size_t seed = std::hash<T1>()(p.first);
        return hash_combine<T2>(seed, p.second);
    }
 };
