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

#include <gtest/gtest.h>
#include <shisen_cpp/shisen_cpp.hpp>

struct Foo
{
  int a;
  double b;

  Foo(const int & a, const double & b)
  : a(a), b(b)
  {
  }
};

TEST(EmptiableTest, Empty) {
  shisen_cpp::Emptiable<int> foo;
  ASSERT_TRUE(foo.is_empty());
}

TEST(CompileTest, Initialized) {
  shisen_cpp::Emptiable<Foo> foo(-1, 5.0);
  ASSERT_TRUE(foo.is_empty());
  ASSERT_DOUBLE_EQ(-1, foo.get().a);
  ASSERT_DOUBLE_EQ(5.0, foo.get().b);
}

TEST(CompileTest, SetAndUnset) {
  shisen_cpp::Emptiable<double> foo;
  ASSERT_TRUE(foo.is_empty());
  ASSERT_DOUBLE_EQ(5.0, foo.get(5.0));

  foo.set(10.0);
  ASSERT_TRUE(foo.is_not_empty());
  ASSERT_DOUBLE_EQ(10.0, foo.get());

  foo = 20.0;
  ASSERT_DOUBLE_EQ(20.0, foo);

  foo.clear();
  ASSERT_FALSE(foo.is_not_empty());
  ASSERT_DOUBLE_EQ(20.0, foo);
}
