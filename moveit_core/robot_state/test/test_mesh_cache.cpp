/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Rivelin Robotics, Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rivelin Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/robot_state/mesh_cache.h>
#include <gtest/gtest.h>
#include <thread>

namespace moveit
{
namespace core
{
static void expect_near(const shapes::Shape* shape, const shapes::Shape* expected,
                        double tolerance = std::numeric_limits<double>::epsilon())
{
  auto shape_mesh = dynamic_cast<const shapes::Mesh*>(shape);
  auto expected_mesh = dynamic_cast<const shapes::Mesh*>(expected);
  ASSERT_EQ((bool)shape_mesh, (bool)expected_mesh);
  if (expected_mesh)
  {
    ASSERT_EQ(shape_mesh->triangle_count, expected_mesh->triangle_count);
    for (unsigned i = 0; i < 3 * expected_mesh->triangle_count; ++i)
    {
      EXPECT_EQ(shape_mesh->triangles[i], expected_mesh->triangles[i]);
      EXPECT_NEAR(shape_mesh->triangle_normals[i], expected_mesh->triangle_normals[i], tolerance);
    }
    ASSERT_EQ(shape_mesh->vertex_count, expected_mesh->vertex_count);
    for (unsigned i = 0; i < 3 * expected_mesh->vertex_count; ++i)
    {
      EXPECT_NEAR(shape_mesh->vertices[i], expected_mesh->vertices[i], tolerance);
      EXPECT_NEAR(shape_mesh->vertex_normals[i], expected_mesh->vertex_normals[i], tolerance);
    }
  }
}

class MeshCacheTest : public ::testing::Test
{
public:
  MeshCacheTest()
  {
    // Generate some points for constructing meshes.
    geometry_msgs::Point origin;
    geometry_msgs::Point x;
    x.x = 1;
    geometry_msgs::Point y;
    y.y = 1;
    geometry_msgs::Point z;
    z.z = 1;

    // Generate some triangles for constructing meshes.
    shape_msgs::MeshTriangle triangle_a;
    triangle_a.vertex_indices = { 0, 1, 2 };
    shape_msgs::MeshTriangle triangle_b;
    triangle_b.vertex_indices = { 0, 2, 3 };
    shape_msgs::MeshTriangle triangle_c;
    triangle_c.vertex_indices = { 0, 3, 1 };

    // Generate some meshes for testing.
    single_triangle_mesh_.vertices = { origin, x, y };
    single_triangle_mesh_.triangles = { triangle_a };
    multi_triangle_mesh_a_.vertices = { origin, x, y, z };
    multi_triangle_mesh_a_.triangles = { triangle_a, triangle_b, triangle_c };
    multi_triangle_mesh_b_.vertices = { x, origin, z, y };
    multi_triangle_mesh_b_.triangles = { triangle_c, triangle_a, triangle_b };
  }
  shape_msgs::Mesh empty_mesh_;
  shape_msgs::Mesh single_triangle_mesh_;
  shape_msgs::Mesh multi_triangle_mesh_a_;
  shape_msgs::Mesh multi_triangle_mesh_b_;
};

TEST_F(MeshCacheTest, returnsCorrectShapeFromEmptyMesh)
{
  MeshCache cache;
  expect_near(cache.getShape(empty_mesh_).get(), shapes::constructShapeFromMsg(empty_mesh_));
}

TEST_F(MeshCacheTest, returnsCorrectShapeFromNonEmptyMesh)
{
  MeshCache cache;
  expect_near(cache.getShape(single_triangle_mesh_).get(), shapes::constructShapeFromMsg(single_triangle_mesh_));
}

TEST_F(MeshCacheTest, aliasesDuplicateMeshes)
{
  MeshCache cache;
  auto shape_a = cache.getShape(single_triangle_mesh_);
  auto shape_b = cache.getShape(single_triangle_mesh_);
  EXPECT_EQ(shape_a, shape_b);
}

TEST_F(MeshCacheTest, doesNotAliasNonDuplicateMeshes)
{
  MeshCache cache;
  auto shape_a = cache.getShape(single_triangle_mesh_);
  auto shape_b = cache.getShape(multi_triangle_mesh_a_);
  EXPECT_NE(shape_a, shape_b);
}

TEST_F(MeshCacheTest, doesNotCacheMeshesSmallerThanMinSize)
{
  MeshCache cache(1e3);
  auto shape_a = cache.getShape(single_triangle_mesh_);
  auto shape_b = cache.getShape(single_triangle_mesh_);
  EXPECT_NE(shape_a, shape_b);
}

TEST_F(MeshCacheTest, doesNotCacheMeshesLargerThanMaxSize)
{
  MeshCache cache(0, 0);
  auto shape_a = cache.getShape(single_triangle_mesh_);
  auto shape_b = cache.getShape(single_triangle_mesh_);
  EXPECT_NE(shape_a, shape_b);
}

TEST_F(MeshCacheTest, expellsOldestAddedMeshWhenCacheFull)
{
  MeshCache cache(0, 2500);
  auto shape_a = cache.getShape(single_triangle_mesh_);
  auto shape_b = cache.getShape(multi_triangle_mesh_a_);
  auto shape_c = cache.getShape(multi_triangle_mesh_b_);
  EXPECT_EQ(cache.getShape(multi_triangle_mesh_a_), shape_b);
  EXPECT_EQ(cache.getShape(multi_triangle_mesh_b_), shape_c);
  EXPECT_NE(cache.getShape(single_triangle_mesh_), shape_a);
}

TEST_F(MeshCacheTest, expellsOldestUsedMeshWhenCacheFull)
{
  MeshCache cache(0, 2500);
  auto shape_a = cache.getShape(multi_triangle_mesh_a_);
  auto shape_b = cache.getShape(single_triangle_mesh_);
  auto shape_c = cache.getShape(multi_triangle_mesh_a_);
  auto shape_d = cache.getShape(multi_triangle_mesh_b_);
  EXPECT_EQ(cache.getShape(multi_triangle_mesh_a_), shape_a);
  EXPECT_EQ(cache.getShape(multi_triangle_mesh_b_), shape_d);
  EXPECT_NE(cache.getShape(single_triangle_mesh_), shape_b);
}

TEST_F(MeshCacheTest, threadLocalCacheAliasesBetweenCalls)
{
  auto shape_a = MeshCache::threadLocalCache().getShape(single_triangle_mesh_);
  auto shape_b = MeshCache::threadLocalCache().getShape(single_triangle_mesh_);
  EXPECT_EQ(shape_a, shape_b);
}

TEST_F(MeshCacheTest, threadLocalCacheDoesNotAliasBetweenThreads)
{
  auto shape_a = MeshCache::threadLocalCache().getShape(single_triangle_mesh_);
  std::thread thread([&shape_a, this]() {
    auto shape_b = MeshCache::threadLocalCache().getShape(single_triangle_mesh_);
    EXPECT_NE(shape_a, shape_b);
  });
  thread.join();
}

}  // namespace core
}  // namespace moveit

int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
