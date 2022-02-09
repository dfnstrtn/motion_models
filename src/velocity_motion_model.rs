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

    
    pub fn update_get_jacobian_stateless(&mut self, state:crate::base::Model2D, odom_l:f32,odom_r:f32)->base::JacobianModel2D{
        match self.update_get_radius_angle_distance(odom_l,odom_r){
            Ok(omega_change)=>{
                let mut data = base::JacobianModel2D::zeros();
                let y_jacobian = -omega_change.R*state.theta.sin() + omega_change.R*(state.theta + omega_change.alpha).sin();
                let x_jacobian = omega_change.R*(state.theta + omega_change.alpha).cos() - omega_change.R*state.theta.cos();
                let theta_jacobian = 0.0;
                data.column(2,(x_jacobian,y_jacobian,theta_jacobian));
                data
            }
            Err(velocity)=>{ 
                let mut data = base::JacobianModel2D::zeros();
                let y_jacobian = velocity.s*state.theta.cos();
                let x_jacobian = velocity.s*state.theta.sin();
                // FIXME DELETE
                if cfg!(test)
                {
                    println!("x_jacobian : {}, y_jacobian:{}",x_jacobian,y_jacobian);
                }
                let theta_jacobian = 0.0;
                data.column(2,(x_jacobian,y_jacobian,theta_jacobian));
                data
            }
        }
    }


}
