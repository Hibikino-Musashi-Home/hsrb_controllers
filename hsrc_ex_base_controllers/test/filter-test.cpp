/// @file filter-test.cpp
/// @brief フィルタクラスのテスト
/// @copyright Copyright (C) 2018 Toyota Motor Corporation

#include <vector>
#include <gtest/gtest.h>
#include <hsrc_ex_base_controllers/filter.hpp>

namespace {
const double kEpsilon = 1.0e-5;
}  // anonymous namespace

namespace hsrc_ex_base_controllers {

TEST(FilterTest, Default) {
  // デフォルトは、a=[1.0], b=[1.0]で初期化される
  Filter<> filter;
  // 何もフィルタされない
  EXPECT_EQ(1.0, filter.update(1.0));
  EXPECT_EQ(2.0, filter.update(2.0));
  EXPECT_EQ(3.0, filter.update(3.0));
}

TEST(FilterTest, Normal) {
  // デフォルトは、{1.0, 0.1, 0.9}, b={0.2, 0.8}で初期化
  double aa[] = {1.0, 0.1, 0.9};
  double bb[] = {0.2, 0.8};
  std::vector<double> a(aa, aa+3);
  std::vector<double> b(bb, bb+2);
  Filter<> filter(a, b);
  // 内部状態は0で初期化される
  // y = 1.0*0.2 + 0.0*0.8 - 0.0*0.1 - 0.0*0.9
  EXPECT_NEAR(0.2, filter.update(1.0), kEpsilon);
  // y = 2.0*0.2 + 1.0*0.8 - 0.2*0.1 - 0.0*0.9
  EXPECT_NEAR(1.18, filter.update(2.0), kEpsilon);
  // y = 3.0*0.2 + 2.0*0.8 - 1.18*0.1 - 0.2*0.9
  EXPECT_NEAR(1.902, filter.update(3.0), kEpsilon);
  // y = 4.0*0.2 + 3.0*0.8 - 1.902*0.1 - 1.18*0.9
  EXPECT_NEAR(1.9478, filter.update(4.0), kEpsilon);
}

TEST(FilterTest, Reset) {
  // デフォルトは、{1.0, 0.1, 0.9}, b={0.2, 0.8}で初期化
  double aa[] = {1.0, 0.1, 0.9};
  double bb[] = {0.2, 0.8};
  std::vector<double> a(aa, aa+3);
  std::vector<double> b(bb, bb+2);
  Filter<> filter(a, b);
  // 内部状態を1.0で初期化(入力と出力が1.0で平衡になった状態)
  filter.reset(1.0);
  // y = 1.0*0.2 + 1.0*0.8 - 1.0*0.1 - 1.0*0.9
  EXPECT_NEAR(0.0, filter.update(1.0), kEpsilon);
  // y = 2.0*0.2 + 1.0*0.8 - 0.0*0.1 - 1.0*0.9
  EXPECT_NEAR(0.3, filter.update(2.0), kEpsilon);
  // y = 3.0*0.2 + 2.0*0.8 - 0.3*0.1 - 0.0*0.9
  EXPECT_NEAR(2.17, filter.update(3.0), kEpsilon);
  // y = 4.0*0.2 + 3.0*0.8 - 2.17*0.1 - 0.3*0.9
  EXPECT_NEAR(2.713, filter.update(4.0), kEpsilon);
}

}  // namespace hsrc_ex_base_controllers

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
