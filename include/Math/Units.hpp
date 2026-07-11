#pragma once

template<typename Tag, typename T = double>
class Quantity {
private:
    T val_;
public:
    constexpr Quantity() : val_(0) {}
    explicit constexpr Quantity(T val) : val_(val) {}

    constexpr T value() const { return val_; }

    constexpr Quantity operator+(const Quantity& other) const { return Quantity(val_ + other.val_); }
    constexpr Quantity operator-(const Quantity& other) const { return Quantity(val_ - other.val_); }
    constexpr Quantity operator*(T scalar) const { return Quantity(val_ * scalar); }
    constexpr Quantity operator/(T scalar) const { return Quantity(val_ / scalar); }
    
    constexpr Quantity& operator+=(const Quantity& other) { val_ += other.val_; return *this; }
    constexpr Quantity& operator-=(const Quantity& other) { val_ -= other.val_; return *this; }
    constexpr Quantity& operator*=(T scalar) { val_ *= scalar; return *this; }
    constexpr Quantity& operator/=(T scalar) { val_ /= scalar; return *this; }

    constexpr bool operator==(const Quantity& other) const { return val_ == other.val_; }
    constexpr bool operator!=(const Quantity& other) const { return val_ != other.val_; }
    constexpr bool operator<(const Quantity& other) const { return val_ < other.val_; }
    constexpr bool operator>(const Quantity& other) const { return val_ > other.val_; }
    constexpr bool operator<=(const Quantity& other) const { return val_ <= other.val_; }
    constexpr bool operator>=(const Quantity& other) const { return val_ >= other.val_; }
};

template<typename Tag, typename T>
constexpr Quantity<Tag, T> operator*(T scalar, const Quantity<Tag, T>& q) {
    return q * scalar;
}

// Unit Tags
struct DegreesTag {};
struct RadiansTag {};
struct MetersPerSecondTag {};
struct MilesPerHourTag {};

// Strongly-typed aliases
using Degrees = Quantity<DegreesTag, double>;
using Radians = Quantity<RadiansTag, double>;
using MetersPerSecond = Quantity<MetersPerSecondTag, double>;
using MilesPerHour = Quantity<MilesPerHourTag, double>;

// Conversion functions (explicit only)
constexpr Degrees to_degrees(Radians rad) {
    return Degrees(rad.value() * 180.0 / 3.14159265358979323846);
}

constexpr Radians to_radians(Degrees deg) {
    return Radians(deg.value() * 3.14159265358979323846 / 180.0);
}

constexpr MilesPerHour to_mph(MetersPerSecond mps) {
    return MilesPerHour(mps.value() * 2.2369362920544);
}

constexpr MetersPerSecond to_mps(MilesPerHour mph) {
    return MetersPerSecond(mph.value() / 2.2369362920544);
}
