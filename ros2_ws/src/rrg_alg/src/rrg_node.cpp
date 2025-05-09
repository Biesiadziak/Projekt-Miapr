#include <cstdio>
#include "nav2_util/node_utils.hpp"

class rrg_node : public nav2_core::GlobalPlanner
{
private:
  /* data */
public:
  rrg_node : public nav2_core::GlobalPlanner(/* args */);
  ~rrg_node : public nav2_core::GlobalPlanner();
};

rrg_node : public nav2_core::GlobalPlanner::rrg_node : public nav2_core::GlobalPlanner(/* args */)
{
}

rrg_node : public nav2_core::GlobalPlanner::~rrg_node : public nav2_core::GlobalPlanner()
{
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world rrg_alg package\n");
  return 0;
}
