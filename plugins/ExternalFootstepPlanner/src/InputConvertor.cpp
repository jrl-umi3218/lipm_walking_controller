#include <ExternalFootstepPlanner/InputConvertor.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

SE2d InputConvertor::convert_PS4_to_SE2d(const sensor_msgs::Joy & joy_msg)
{
  int axis_forward_sgn = 1;
  int axis_lateral_sgn = 1;
  int axis_theta_sgn = 1;
  float origin_theta = 0.0f;
  double radius_max = 0.15;
  double radius;

  float val_forward = axis_forward_sgn * joy_msg.axes.at(1);
  float val_lateral = axis_lateral_sgn * joy_msg.axes.at(0);
  float val_theta = joy_msg.axes.at(3);

  if(abs(val_theta) < 0.0001)
  {
    // remove too small input as noise. (about right stick)
    val_theta = 0.0;
  }
  else
  {
    val_theta = axis_theta_sgn * val_theta * M_PI_2;
    // val_theta is -1.0 ~ +1.0. In order to project it to -Pi/2 ~ +Pi/2 [rad], I multiply M_PI/2 here.
  }

  if(L2norm(val_forward, val_lateral) < 0.01)
  {
    // remove too small input as noise. (about left stick)
    radius = 0.0;
  }
  else
  {
    double input_norm = Uniform_norm(val_forward, val_lateral);
    radius = calc_exponential_interpolation(2.0, radius_max, input_norm);
    origin_theta = atan2(val_lateral, val_forward);
  }

  SE2d result;
  result.x = radius * cos(origin_theta);
  result.y = radius * sin(origin_theta);
  result.theta = val_theta;

  return result;
}

SE2d InputConvertor::convert_Oculus_to_SE2d(const sensor_msgs::Joy & joy_msg)
{
  /// Convert_Oculus_to_SE2d function not fully tested (remove this comment if this has changed !)
  int axis_forward_sgn = 1;
  int axis_lateral_sgn = -1;
  int axis_theta_sgn = -1;
  float origin_theta = 0.0f;
  double radius_max = 0.15;
  double radius;

  float val_forward = axis_forward_sgn * joy_msg.axes.at(2);
  float val_lateral = axis_lateral_sgn * joy_msg.axes.at(3);
  float val_theta = joy_msg.axes.at(7);

  if(abs(val_theta) < 0.0001)
  {
    // remove too small input as noise. (about right stick)
    val_theta = 0.0;
  }
  else
  {
    val_theta = axis_theta_sgn * val_theta * M_PI_2;
    // val_theta is -1.0 ~ +1.0. In order to project it to -Pi/2 ~ +Pi/2 [rad], I multiply M_PI/2 here.
  }

  if(L2norm(val_forward, val_lateral) < 0.01)
  {
    // remove too small input as noise. (about left stick)
    radius = 0.0;
  }
  else
  {
    double input_norm = Uniform_norm(val_forward, val_lateral);
    radius = calc_exponential_interpolation(2.0, radius_max, input_norm);
    origin_theta = atan2(val_lateral, val_forward);
  }

  SE2d result;
  result.x = radius * cos(origin_theta);
  result.y = radius * sin(origin_theta);
  result.theta = val_theta;

  return result;
}

double InputConvertor::L2norm(double a_x, double a_y, double b_x, double b_y)
{
  return sqrt(pow(a_x - b_x, 2) + pow(a_y - b_y, 2));
}

double InputConvertor::Uniform_norm(double a_x, double a_y)
{
  if(abs(a_x) > abs(a_y))
  {
    return abs(a_x);
  }
  else
  {
    return abs(a_y);
  }
}

double InputConvertor::calc_exponential_interpolation(double exp, double y_max, double x)
{
  // calc y_max* x^exp  [0,1] -> [0,y_max]
  return y_max * pow(x, exp);
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin