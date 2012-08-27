#include <feature_constraints/Analysis.h>

#include <gtest/gtest.h>

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <vector>

using namespace std;
using namespace KDL;


TEST(Analysis, axis_sampler_coverage)
{
  // 'model' coordinate axes and their negatives.
  vector<Vector> axes;

  // 'counts[aa][a]' counts, how many times axis 'aa' was aligned with axis 'a'.
  vector< vector<int> > counts;

  axes.push_back(Vector( 1, 0, 0));
  axes.push_back(Vector(-1, 0, 0));
  axes.push_back(Vector( 0, 1, 0));
  axes.push_back(Vector( 0,-1, 0));
  axes.push_back(Vector( 0, 0, 1));
  axes.push_back(Vector( 0, 0,-1));

  counts.resize(3);
  counts[0].resize(6);
  counts[1].resize(6);
  counts[2].resize(6);

  for(int i=0; i < 24; i++)
  {
    Frame f = axis_sampler(i);

    for(int aa=0; aa < 3; aa++)
    {
      Vector v(f.M(0, aa), f.M(1, aa), f.M(2, aa));
   
      int a;
      for(a=0; a < 6; a++)
      {
        if(Equal(v, axes[a]))
        {
          counts[aa][a]++;
          break;
        }
      }
      if(a == 6)
        FAIL();  // result was not axis-aligend
    }  
  }

  for(int i=0; i < 3; i++)
    for(int j=0; j < 6; j++)
      EXPECT_EQ(4, counts[i][j]);
}


TEST(Analysis, axis_sampler_uniqueness)
{
  vector<Frame> samples;

  for(int i=0; i < 24; i++)
    samples.push_back(axis_sampler(i));

  // Assume distinctness between all pairs
  for(int i=0; i < 24; i++)
    for(int j=0; j < 24; j++)
      if(i != j)
        EXPECT_FALSE(Equal(samples[i], samples[j]));
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

