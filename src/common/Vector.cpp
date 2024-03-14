#include "Vector.h"

template <class ValueType> 
base::Vector<ValueType>::Vector(std::initializer_list<ValueType> il) : coord(il) {}

template <class ValueType> 
ValueType& base::Vector<ValueType>::operator()(size_t index) {
  if (index >= coord.size()) {
    throw std::out_of_range("Index out of bounds");
  }
  return coord[index];
}
template <class ValueType> 
const ValueType& base::Vector<ValueType>::operator()(size_t index) const {
  if (index >= coord.size()) {
    throw std::out_of_range("Index out of bounds");
  }
  return coord[index];
}