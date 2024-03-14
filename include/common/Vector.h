#pragma once

#include <stdexcept>
#include <valarray>
#include <cstddef>

namespace base {

    template <class ValueType>
    struct Vector {
        using VectorType = std::valarray<ValueType>;
        VectorType coord{};

        // constructor
        Vector(std::initializer_list<ValueType> il);

        // operators
        ValueType& operator()(size_t index);
        const ValueType& operator()(size_t index) const;
        Vector operator+(const Vector& other) const;
    };   
    
    using VectorF = Vector<float>;
    using VectorD = Vector<double>;
}