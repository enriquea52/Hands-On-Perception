#include <Eigen/Dense>

float risk_calc( Eigen::Vector3d x_0,   Eigen::Vector3d x_1,   Eigen::Vector3d x_2)
{

    float t_,d_; 
    float offset = 0.5;
    float delta_t = 1.0/10.0;
    float TTC, d_norm, sigma, risk_function;

    float k = 1, Offset = 0.5, max_value = 1, step_saturation = 5, min_weight = 0, max_weight = 1;
    step_saturation = step_saturation + Offset;


      t_ = -(x_1 - x_0).dot(x_2 - x_1)/std::pow((x_2 - x_1).norm(),2);
      d_ = ((x_2 - x_1).cross(x_1 - x_0)).norm() / (x_2 - x_1).norm();
      TTC = ((t_ - 1)*delta_t) + offset;
      d_norm = (d_ / (x_2 - x_1).norm()) * delta_t;
      sigma = k*TTC;
      risk_function = k/(sigma) * std::exp(- std::pow(d_norm,2) / (2 * std::pow(sigma,2)) );
      
      // Risk saturation
      if(max_value != 0){
          risk_function = (step_saturation*max_value) * risk_function;
          if(risk_function > max_value){
              risk_function = max_value;
          }
      }else{
          risk_function = step_saturation * risk_function;
      }

      return risk_function;
}


