// Copyright (c) 2021 ICHIRO ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SHISEN_CPP__UTILITY__EMPTIABLE_IMPL_HPP_
#define SHISEN_CPP__UTILITY__EMPTIABLE_IMPL_HPP_

#include <shisen_cpp/utility/emptiable.hpp>

namespace shisen_cpp
{

template<typename T>
template<typename ... Types>
Emptiable<T>::Emptiable(Types ... types)
: empty(true),
  value(types ...)
{
}

template<typename T>
Emptiable<T>::~Emptiable()
{
}

template<typename T>
Emptiable<T>::operator T() const
{
  return value;
}

template<typename T>
const T & Emptiable<T>::operator=(const T & new_value)
{
  set(new_value);
  return get();
}

template<typename T>
void Emptiable<T>::clear()
{
  empty = true;
}

template<typename T>
void Emptiable<T>::set(const T & new_value)
{
  empty = false;
  value = new_value;
}

template<typename T>
bool Emptiable<T>::is_empty() const
{
  return empty;
}

template<typename T>
bool Emptiable<T>::is_not_empty() const
{
  return !is_empty();
}

template<typename T>
const T & Emptiable<T>::get() const
{
  return value;
}

template<typename T>
const T & Emptiable<T>::get(const T & default_value) const
{
  if (is_empty()) {
    return default_value;
  }

  return get();
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__UTILITY__EMPTIABLE_IMPL_HPP_
