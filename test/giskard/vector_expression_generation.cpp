#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class VectorExpressionGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      x = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
      y = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
      z = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());

      x->set_value(1.1);
      y->set_value(2.2);
      z->set_value(3.3);
    }

    virtual void TearDown(){}

    giskard::ConstDoubleSpecPtr x, y, z;
};

TEST_F(VectorExpressionGenerationTest, Constructor)
{
  giskard::ConstructorVectorSpec descr;
  giskard::Scope scope;

  descr.set_x(x);
  descr.set_y(y);
  descr.set_z(z);

  KDL::Vector v = descr.get_expression(scope)->value();
  
  EXPECT_DOUBLE_EQ(v.x(), 1.1);
  EXPECT_DOUBLE_EQ(v.y(), 2.2);
  EXPECT_DOUBLE_EQ(v.z(), 3.3);
}
