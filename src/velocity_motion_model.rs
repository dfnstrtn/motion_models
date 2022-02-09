use crate::base::ChangeParams;
use crate::base;
pub struct VelocityMotionModel{
    odom_l:f32,
    odom_r:f32,
    time_step:f32,
    base_length:f32,
    wheel_radius:f32
}
impl VelocityMotionModel{
    pub fn new(base_length:f32,wheel_radius:f32,time_step:f32)->VelocityMotionModel{
        VelocityMotionModel{
            odom_l:0.0,
            odom_r:0.0,
            time_step,
            base_length,
            wheel_radius
        }
    }
    

    /// Returns radius of turning , angle of turn and distance travelled 
    /// If motion is in a straight line , returns Error(distancetravelled)
    /// As stated in the struct definition odometry_l represents the DISTANCE covered by the wheel
    /// This function does not affect any value of the state of the differential robot model.
    /// If you intend to update the odometry motion model use the function 
    /// `update_odometry_readings()`
    pub fn update_get_radius_angle_distance(&mut self, odom_l:f32,odom_r:f32)->Result<ChangeParams,ChangeParams>{
        let L = self.base_length;
        let diff_v_l =  (odom_l - self.odom_l)/self.time_step;
        let diff_v_r = (odom_r - self.odom_r)/self.time_step;
        
        let omega = (diff_v_r - diff_v_l)/L;
        let v = (diff_v_l+diff_v_r)/2.0;
        
        if omega==0.0{
            return Err(ChangeParams::new(0.0,0.0,v))
        }

        let R = v/omega;
        Ok(ChangeParams::new(R,omega,v)) 
    }
    





    pub fn update_coords_odometry_stateless(&mut self,state:crate::base::Model2D,odom_l:f32,odom_r:f32)->crate::base::Model2D{
        match self.update_get_radius_angle_distance(odom_l,odom_r){
            Ok(pos_change)=>{
                 let y_new = pos_change.R*state.theta.cos() - pos_change.R*(state.theta + pos_change.alpha*self.time_step).cos() + state.y;
                let x_new = pos_change.R*(state.theta + pos_change.alpha*self.time_step).sin() - pos_change.R*state.theta.sin() + state.x;
                let theta_new = state.theta + pos_change.alpha;
                crate::base::Model2D::new(x_new,y_new,theta_new)
            }
            Err(distance)=>{
                let y_new = state.y + distance.s*state.theta.sin();
                let x_new = state.x + distance.s*state.theta.cos();
                crate::base::Model2D::new(x_new,y_new,state.theta)
            }
        }
    }

    
    /// Gets the jacobian of a function under normal conditions.
    /// If motion is in a straight line the radius of curvature is infinity , because of which you
    /// may  have to use  [OdometryModel::update_get_jacobian_straight_line_stateless]
    pub fn update_get_jacobian_stateless(state:crate::base::Model2D, pos_change:ChangeParams)->base::JacobianModel2D{
        let mut data = base::JacobianModel2D::zeros();
        let y_jacobian = -pos_change.R*state.theta.sin() + pos_change.R*(state.theta + pos_change.alpha).sin();
        
        let x_jacobian = pos_change.R*(state.theta + pos_change.alpha).cos() - pos_change.R*state.theta.cos();

        let theta_jacobian = 0.0;

        data.column(2,(x_jacobian,y_jacobian,theta_jacobian));
        data
    }





}
