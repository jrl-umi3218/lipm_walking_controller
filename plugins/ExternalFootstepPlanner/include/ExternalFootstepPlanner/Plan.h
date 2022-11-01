#pragma once

#include <boost/optional.hpp>

#include <Eigen/Core>
#include <ExternalFootstepPlanner/SE2d.h>
#include <ostream>
#include <vector>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

enum Foot
{
  Right = 0,
  Left = 1
};

struct Contact
{
  Foot foot;
  SE2d pose;

public:
  Contact(Foot foot, const SE2d & pose) : foot(foot), pose(pose) {}

  static std::string footName(Foot foot)
  {
    if(foot == Left)
    {
      return "left";
    }
    return "right";
  }
};

struct Plan
{
  std::vector<Contact> contacts;
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin

inline std::ostream & operator<<(std::ostream & os, const mc_plugin::ExternalFootstepPlanner::Contact & contact)
{
  os << "Foot: " << mc_plugin::ExternalFootstepPlanner::Contact::footName(contact.foot) << ", pose: " << contact.pose;
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const mc_plugin::ExternalFootstepPlanner::Plan & plan)
{
  os << "Contacts:" << std::endl;
  for(const auto & contact : plan.contacts)
  {
    os << contact << std::endl;
  }
  return os;
}
