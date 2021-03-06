#include "../include/interpolation/bezier.hpp"

// Default constructor.
BezierCurve::BezierCurve()
{
  std::cout<<"Bezier Curve created!"<< std::endl;
}
// Useful constructor for pure pursuit.
BezierCurve::BezierCurve(nav_msgs::Path path, int numb_ctr_points)
{
  // Save the path from the argument to the path-member variable.
  this->BezierCurve::setPath(path);
  // Set the number of control points (n+1) of the bezier curve.
  this->BezierCurve::setNumbCtrlPoints(numb_ctr_points);
  std::cout<<"Bezier Curve created!"<< std::endl;
}
BezierCurve::~BezierCurve()
{
  std::cout<<"Bezier Curve removed"std::endl;
}

// Object changing methods.
// Sets the path.
void BezierCurve::setPath(nav_msgs::Path path)
{
  *this.path_ = path;
}
// Set the number of control points of the bezier-curve.
void BezierCurve::setNumbCtrlPoints(int numb_ctr_points)
{
  *this.numb_ctr_points_ = numb_ctr_points;
}
// Set the point of path_ around which the bezier-curve should be centered.
void BezierCurve::setCenterIndex(int center_index)
{
    *this.center_index_ = center_index;
}
// After having defined the above parameters, the control points can be copied into x_path_ and y_path_.
void BezierCurve::setCtrlPoints()
{
  // Clear up what was saved earlier in the control points vectors.
  delete[] *this.x_path_;
  delete[] *this.y_path_;
  // Set new size of control points arrays.
  *this.x_path_ = new float[numb_ctr_points_];
  *this.y_path_ = new float[numb_ctr_points_];
  // Assign values to resized arrays.
  // The values are taken from path_ more or less symmetrically around center_index_.
  // Differentiate between numb_ctr_points_ being even or odd.
  if(numb_ctr_points_%2 == 0)
  	{
      // Number of Control Points is even ==> Look one point more ahead than behind!
  		for(int i = 0; i < *this.numb_ctr_points_; i++)
      {
        *this.x_path_[i] = *this.path_.poses[*this.center_index_ - ((*this.numb_ctr_points_ - 1)/2) + 1].pose.position.x;
        *this.y_path_[i] = *this.path_.poses[*this.center_index_ - ((*this.numb_ctr_points_ - 1)/2) + 1].pose.position.y;
      }
  	}
  	else if(*this.numb_ctr_points_%2 != 0)
  	{
      // Number of Control Points is odd.
  		for(int i = 0; i < *this.numb_ctr_points_; i++)
      {
        *this.x_path_[i] = *this.path_.poses[*this.center_index_ - ((*this.numb_ctr_points_ - 1)/2)].pose.position.x;
        *this.y_path_[i] = *this.path_.poses[*this.center_index_ - ((*this.numb_ctr_points_ - 1)/2)].pose.position.y;
      }
  	}
}
// Set the parameter t, for which properties like curvature or position should be calculated.
void BezierCurve::setActiveT(float t)
{
  *this.active_t_ = t;
}

// Object using methods.
// Return the interpolated coordinate at parameter t.
Eigen::Vector3d BezierCurve::getCoordinate(float t)
{
  float x_interpolated = 0.0;
  float y_interpolated = 0.0;
  Eigen::Vector3d coordinate_to_return;
  int n = *this.numb_ctr_points_;
  for(int i = 0; i < n + 1; i++)
  {
    float factor = pow(t, i)*pow(1.0 - t, n - i);
    x_interpolated = x_interpolated + *this.x_path_[i]*this->BezierCurve::binomial(*this.numb_ctr_points_ - 1, i)*factor;
    y_interpolated = y_interpolated + *this.y_path_[i]*this->BezierCurve::binomial(*this.numb_ctr_points_ -1, i)*factor;
  }
  coordinate_to_return[0] = x_interpolated;
  coordinate_to_return[1] = y_interpolated;
  return coordinate_to_return;
}
// Find and set the nearest t;
void BezierCurve::findNearestT(int path_index)
{
  float x_curr_arr_position = *this.x_path_[path_index];
  float y_curr_arr_position = *this.y_path_[path_index];
  float t_near_iteration = 0.0;
  float error = 1000.0;

  for (float t = 0.0, t < 1.001; t = t + 0.001)
  {
    Eigen::Vector3d returned_coordinate = this->BezierCurve::getCoordinate(t);
    float error_i = sqrt(pow(x_curr_arr_position - returned_coordinate[0], 2) + pow(y_curr_arr_position - returned_coordinate[1], 2));
    if (error_i < error)
    {
      error = error_i
      t_near_iteration = t;
    }
  }
  *this.nearest_t_ = t_near_iteration;
}
// For the defined bezier curve find the curvature nearest to center_index_.
void BezierCurve::calcCurvature()
{
  this->BezierCurve::findT(center_index_);
  *this.active_t_ = *this.nearest_t_;
  this->calcCurvature(*this.active_t_);
}
void BezierCurve::calcCurvature(float t)
{
  float curvature;
  x_dot = *this.x_dot_;
  x_dot_dot = *this.x_dot_dot_;
  y_dot = *this.y_dot_;
  y_dot_dot = *this.y_dot_dot_;
  this->BezierCurve::calcXyDot(t);
  this->BezierCurve::calcXyDotDot(t);
  curvature = (x_dot*y_dot_dot - x_dot_dot*y_dot)/pow(pow(x_dot,2) + pow(y_dot,2),1.5);
  *this.curvature_ = curvature;
}
void BezierCurve::calcXyDot(float t)
{
  float x_dot = 0.0;
  float y_dot = 0.0;
  int n = *this.numb_ctr_points_ - 1;

  for(int i = 0; i < n + 1; i++)
  {
    float dot_factor = i*pow(t, i - 1)*pow(1.0 - t, n - i) + pow(t, i)*(i - n)*pow(1.0 - t, n - i - 1) ;
    x_dot = x_dot + *this.x_path_[i]*this->BezierCurve::binomial(n, i)*dot_factor;
    y_dot = y_dot + *this.y_path_[i]*this->BezierCurve::binomial(n, i)*dot_factor;
  }

  *this.x_dot_ = x_dot;
  *this.y_dot_ = y_dot;
}
void BezierCurve::calcXyDotDot(float t)
{
  float x_dot_dot = 0.0;
  float y_dot_dot = 0.0;
  int n = *this.numb_ctr_points_ - 1;

  for(int i = 0; i < n + 1; i++)
  {
    float dot_dot_factor = i*(i - 1)*pow(t, i - 2)*pow(1.0 - t, n - i)
    + i*pow(t, i - 1)*(i - n)*pow(n - i - 1)
    + i*pow(t, i - 1)*(i - n)*pow(1.0 - t, n - i - 1)
    + pow(t, i)*(n - i)*(n - i - 1)*(-1.0)*pow(1.0 - t, n - i - 2);
    x_dot_dot = x_dot_dot + x_path_[i]*this->BezierCurve::binomial(n, i)*dot_dot_factor;
    y_dot_dot = y_dot_dot + y_path_[i]*this->BezierCurve::binomial(n, i)*dot_dot_factor;
  }

  *this.x_dot_dot_ = x_dot_dot;
  *this.y_dot_dot_ = y_dot_dot;
}
float BezierCurve::getCurvature
{
  return *this.curvature_;
}

// Helper methods.
// n-factorial ==> n!. Is a static method.
int BezierCurve::factorial(int n)
{
  int n_fac = 1;
  if(n == 0)
  {
    n_fac = 1;
  }
  else
  {
    n_fac = BezierCurve::factorial(n-1);
  }
  return n_fac;
}
// Calculates binomial-coefficient a lower b. Is a static method.
int BezierCurve::binomial(int a, int b)
{
  int binom = BezierCurve::factorial(a)/(BezierCurve::factorial(b)*BezierCurve::factorial(a - b));
  return binom;
}
