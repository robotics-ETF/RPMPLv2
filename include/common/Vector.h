#pragma once

#include <cstddef>
#include <stdexcept>
#include <valarray>

namespace base {

// Forward declaration
template <class ValueType> 
struct Vector;

using VectorF = Vector<float>;


template <class ValueType> 
struct Vector {
  using VectorType = std::valarray<ValueType>;
  VectorType coord{};

  // constructors
  Vector();
  Vector(std::initializer_list<ValueType> il);

  // operators
  ValueType &operator()(size_t index);
  const ValueType &operator()(size_t index) const;
  Vector operator+(const Vector &other) const;

  // methods
  size_t size() const;
};

// IMPLEMENTATION
template <class ValueType>
base::Vector<ValueType>::Vector()
    : coord(VectorType()) {}

template <class ValueType>
base::Vector<ValueType>::Vector(std::initializer_list<ValueType> il)
    : coord(il) {}

template <class ValueType>
ValueType &base::Vector<ValueType>::operator()(size_t index) {
  if (index >= coord.size()) {
    throw std::out_of_range("Index out of bounds");
  }
  return coord[index];
}
template <class ValueType>
const ValueType &base::Vector<ValueType>::operator()(size_t index) const {
  if (index >= coord.size()) {
    throw std::out_of_range("Index out of bounds");
  }
  return coord[index];
}

template <class ValueType> size_t base::Vector<ValueType>::size() const {
  return coord.size();
}

} // namespace base