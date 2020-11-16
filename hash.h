#pragma once

//hashable concept
template<typename T>
concept Hashable = requires(T a) {
	{ std::hash<T>{}(a) }->std::convertible_to<std::size_t>;
};

//combine previous 0-N hashes with one more hash
template <Hashable T>
inline auto hash_combine(std::size_t& seed, T v) {
	seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	return seed;
}

//hash of pairs
 template<Hashable T1, Hashable T2>
 struct std::hash<std::pair<T1,T2>> {
    size_t operator()(const std::pair<T1,T2>& p) {
        std::size_t seed = std::hash<T1>()(p.first);
        return hash_combine<T2>(seed, p.second);
    }
 };

//hash of tuples
template<typename T, std::size_t... Is>
auto hash_impl(T const& t, std::index_sequence<Is...>) {
    size_t seed = 0;
	(hash_combine(seed, std::get<Is>(t)) + ... + 0);
	return seed;
}

//hash of a tuple of hashables
template<Hashable... Args>
struct std::hash<std::tuple<Args...>> {
    size_t operator()(const std::tuple<Args...>& v) {
        return hash_impl(v, std::make_index_sequence<sizeof...(Args)>());
    }
};

//hash of specific data types
 template<>
 struct std::hash<vec3> {
    size_t operator()(const vec3& v) {
        std::size_t seed = std::hash<float>()(v.x);
        hash_combine(seed, v.y);
        return hash_combine(seed, v.z);
    }
 };



