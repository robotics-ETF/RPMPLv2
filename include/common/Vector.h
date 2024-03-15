#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <valarray>

namespace base {

// Forward declaration
template <class ValueType> struct Vector;

using VectorF = Vector<float>;
using VectorD = Vector<double>;

template <class ValueType> struct Vector {
  using VectorType = std::valarray<ValueType>;
  VectorType coord{};
  static constexpr ValueType ZERO{};

  // constructors
  Vector();
  explicit Vector(std::valarray<ValueType> valArray);
  Vector(std::initializer_list<ValueType> il);
  Vector(const Vector &other);
  Vector(Vector &&other) noexcept;

  // operators
  ValueType &operator()(size_t index);
  const ValueType &operator()(size_t index) const;
  Vector &operator=(const Vector &other);
  Vector &operator=(Vector &&other) noexcept;

  // arithmetic operators
  Vector operator+(const Vector &other) const;
  Vector operator+(const ValueType &other) const;
  ValueType dot(const Vector &other) const;
  ValueType norm() const;

  // overload compound operators
  Vector& operator+=(const ValueType& other);
  Vector& operator+=(const Vector& other);

  // zero, one
  static Vector zeros(size_t dim);
  static Vector ones(size_t dim);

  // methods
  size_t size() const;
};

// IMPLEMENTATION
template <class ValueType>
Vector<ValueType>::Vector() : coord(VectorType()) {}

template <class ValueType>
Vector<ValueType>::Vector(std::valarray<ValueType> valArray)
         : coord(valArray) {}

template <class ValueType>
Vector<ValueType>::Vector(std::initializer_list<ValueType> il)
    : coord(il) {}

template <class ValueType>
Vector<ValueType>::Vector(const Vector<ValueType> &other)
    : coord(other.coord) {}

template <class ValueType>
Vector<ValueType>::Vector(Vector<ValueType> &&other) noexcept
    : coord(std::move(other.coord)) {}

template <class ValueType>
ValueType &Vector<ValueType>::operator()(size_t index) {
  if (index >= coord.size()) {
    throw std::out_of_range("Index out of bounds");
  }
  return coord[index];
}

template <class ValueType>
Vector<ValueType> &
Vector<ValueType>::operator=(const Vector<ValueType> &other) {
  if (this != &other) {
    coord = other.coord;
  }
  return *this;
}

template <class ValueType>
Vector<ValueType>& Vector<ValueType>::operator=(Vector<ValueType> &&other) noexcept {
  if (this != &other) {
    coord = std::move(other.coord);
  }
  return *this;
}

template <class ValueType>
const ValueType &Vector<ValueType>::operator()(size_t index) const {
  if (index >= coord.size()) {
    throw std::out_of_range("Index out of bounds");
  }
  return coord[index];
}

template <class ValueType> 
size_t Vector<ValueType>::size() const {
  return coord.size();
}

template <class ValueType>
Vector<ValueType> Vector<ValueType>::operator+(const Vector<ValueType> &other) const {
  assert(coord.size() == other.coord.size());
  return Vector{coord + other.coord};
}

template <class ValueType>
Vector<ValueType> Vector<ValueType>::operator+(const ValueType &other) const {
  return Vector{coord + other};
}

template <class ValueType>
Vector<ValueType>& Vector<ValueType>::operator+=(const Vector<ValueType>& other) {
  assert(coord.size() == other.coord.size());
  this->coord += other.coord;
  return *this;
}

template <class ValueType>
Vector<ValueType>& Vector<ValueType>::operator+=(const ValueType& other) {
  this->coord += other;
  return *this;
}

template <class ValueType>
ValueType Vector<ValueType>::dot(const Vector<ValueType> &other) const {
  assert(coord.size() == other.coord.size());
  return (coord * other.coord).sum();
}

template <class ValueType> 
ValueType Vector<ValueType>::norm() const {
  return std::sqrt(this->dot(*this));
}

template <class ValueType> 
Vector<ValueType> Vector<ValueType>::zeros(size_t dim) {
  return Vector({VectorType(ZERO, dim)});
}

template <class ValueType> 
Vector<ValueType> Vector<ValueType>::ones(size_t dim) {
  return Vector({VectorType(ZERO+1, dim)});
}

} // namespace base