#include <Eigen/Core>
#include <stdexcept>

#include "agimus_controller/fixed_capacity.hpp"
#include "agimus_controller/status.hpp"
#include "rt_guard.hpp"

using agimus_controller::FixedString;
using agimus_controller::FixedVector;
using agimus_controller::Status;

TEST_CASE("FixedString: setup constructor") {
  const FixedString<4> s("abcd");
  CHECK(s == "abcd");
  CHECK(s.size() == 4);
  CHECK(std::string_view(s.c_str()) == "abcd");
  CHECK_THROWS_AS(FixedString<4>("abcde"), std::invalid_argument);
  CHECK(FixedString<4>().empty());
}

TEST_CASE("RtFixedString.assign: fits, too long, empty") {
  FixedString<8> s("init");
  bool ok = false;
  CHECK_RT_SAFE(ok = s.assign("frame_1"));
  CHECK(ok);
  CHECK(s == "frame_1");
  CHECK_RT_SAFE(ok = s.assign("this is far too long"));
  CHECK_FALSE(ok);
  CHECK(s == "frame_1");  // Unchanged on failure.
  CHECK_RT_SAFE(ok = s.assign(""));
  CHECK(ok);
  CHECK(s.empty());
}

TEST_CASE("RtFixedString.view: returns the stored characters") {
  const FixedString<8> s("abc");
  std::string_view v;
  CHECK_RT_SAFE(v = s.view());
  CHECK(v == "abc");
}

TEST_CASE("RtFixedString.operator==: with FixedString and string_view") {
  const FixedString<8> a("abc");
  const FixedString<8> b("abc");
  const FixedString<8> c("abd");
  bool r1 = false, r2 = true, r3 = false, r4 = true;
  CHECK_RT_SAFE(r1 = (a == b); r2 = (a == c); r3 = (a == "abc");
                r4 = (a == "ab"));
  CHECK(r1);
  CHECK_FALSE(r2);
  CHECK(r3);
  CHECK_FALSE(r4);
}

TEST_CASE("RtFixedVector.push_back: until full") {
  FixedVector<int, 3> v;
  bool ok[4] = {};
  CHECK_RT_SAFE(for (int i = 0; i < 4; ++i) ok[i] = v.push_back(i));
  CHECK(ok[0]);
  CHECK(ok[1]);
  CHECK(ok[2]);
  CHECK_FALSE(ok[3]);
  CHECK(v.full());
  CHECK(v.size() == 3);
  CHECK(v[2] == 2);
}

TEST_CASE("RtFixedVector.push_back: elements keep their Eigen storage") {
  // Elements are constructed up front; once sized (setup), clear() and
  // push_back() reuse their storage without allocating.
  FixedVector<Eigen::VectorXd, 2> v;
  const Eigen::VectorXd a = Eigen::VectorXd::Constant(5, 1.0);
  const Eigen::VectorXd b = Eigen::VectorXd::Constant(5, 2.0);
  v.push_back(a);  // Setup: sizes element 0.
  v.push_back(a);  // Setup: sizes element 1.
  v.clear();
  CHECK_RT_SAFE(v.push_back(b); v.push_back(a));
  CHECK(v[0] == b);
  CHECK(v[1] == a);
}

TEST_CASE("RtFixedVector.clear: empties") {
  FixedVector<int, 3> v;
  v.push_back(1);
  CHECK_RT_SAFE(v.clear());
  CHECK(v.empty());
}

TEST_CASE("RtFixedVector.operator[]: reads and writes") {
  FixedVector<int, 3> v;
  v.push_back(1);
  v.push_back(2);
  int r = 0;
  CHECK_RT_SAFE(v[1] = 5; r = v[0] + v[1]);
  CHECK(r == 6);
}

TEST_CASE("FixedVector: equality and iteration") {
  FixedVector<int, 3> a, b;
  a.push_back(1);
  b.push_back(1);
  CHECK(a == b);
  b.push_back(2);
  CHECK_FALSE(a == b);
  int sum = 0;
  for (int x : b) {
    sum += x;
  }
  CHECK(sum == 3);
}

TEST_CASE("RtFree.to_string: every status has a name") {
  const char* name = nullptr;
  CHECK_RT_SAFE(name = agimus_controller::to_string(Status::kBufferFull));
  CHECK(std::string_view(name) == "buffer full");
  for (Status s : {Status::kOk, Status::kNotEnoughData, Status::kBufferFull,
                   Status::kUnknownFrame, Status::kMissingTransform,
                   Status::kSizeMismatch, Status::kNoPreviousSolution}) {
    CHECK(std::string_view(agimus_controller::to_string(s)) !=
          "unknown status");
  }
}
