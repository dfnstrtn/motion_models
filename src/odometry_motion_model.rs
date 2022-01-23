use super::base;

pub struct ChangeParams{
    R:f32,
    alpha:f32,
    s:f32
}
impl ChangeParams{
    pub fn new(R:f32,alpha:f32,s:f32)->ChangeParams{
        ChangeParams{
            R,
            alpha,
            s
        }
    }
}



/// A struct representing a differential drive robot
/// odometry_l and odometry_r represent the DISTANCE covered by the wheel 
/// odometry_l is NOT the encoder reading you obtain using a rotation sensor 
/// rather , it is Radius_of_wheel * angle_moved_by_wheel
pub struct OdometryModel{
    odometry_l:f32,
    odometry_r:f32,
    pub x_t:base::Model2D,
    x_tprev:base::Model2D,
    base_length:f32
}

impl OdometryModel{
    
    pub fn new(base_length:f32)->OdometryModel{
        OdometryModel{
            odometry_l:0.0,
            odometry_r:0.0,
            x_t:base::Model2D::new(0.0,0.0,0.0),
            x_tprev:base::Model2D::new(0.0,0.0,0.0),
            base_length,
        }
    }


    /// Returns radius of turning , angle of turn and distance travelled 
    /// If motion is in a straight line , returns Error(distancetravelled)
    /// As stated in the struct definition odometry_l represents the DISTANCE covered by the wheel 
    pub fn update_get_radius_angle_distance(&mut self, odometry_l:f32,odometry_r:f32)->Result<ChangeParams,ChangeParams>{
        let L = self.base_length;
        let diff_l =  odometry_l - self.odometry_l;
        let diff_r = odometry_r - self.odometry_r;
        let alpha = (diff_r - diff_l)/(2.0*L);
        let delta_s = (diff_l+diff_r)/2.0;
        
        if alpha==0.0{
            return Err(ChangeParams::new(0.0,0.0,delta_s))
        }

        let R = diff_l/alpha;
        Ok(ChangeParams::new(R,alpha,delta_s)) 
    }
    

    /// Updates position coordinates and returns the new position coordinates 
    pub fn update_position_coords(&mut self,pos_change:ChangeParams)->base::Model2D{
        let y_new = pos_change.R*self.x_t.theta.cos() - pos_change.R*(self.x_t.theta + pos_change.alpha).cos() + self.x_t.y;
        let x_new = pos_change.R*(self.x_t.theta + pos_change.alpha).sin() - pos_change.R*self.x_t.theta.sin() + self.x_t.x;
        let theta_new = self.x_t.theta + pos_change.alpha;
        
        self.x_tprev.x = self.x_t.x;
        self.x_tprev.y = self.x_t.y;
        self.x_tprev.theta = self.x_t.theta;

        self.x_t.x = x_new;
        self.x_t.y = y_new;
        self.x_t.theta = theta_new;
        base::Model2D::new(x_new,y_new, theta_new)
    }
    

    /// If the angle of curvature is zero update position_coords will not work
    pub fn update_position_coords_straight_line(&mut self, distance:ChangeParams)->base::Model2D{
        let y_new = self.x_t.y + distance.s*self.x_t.theta.sin();
        let x_new = self.x_t.x + distance.s*self.x_t.theta.cos();
        
        self.x_tprev.x = self.x_t.x;
        self.x_tprev.y = self.x_t.y;
        self.x_tprev.theta = self.x_t.theta;
        
        self.x_t.x = x_new;
        self.x_t.y = y_new;
        base::Model2D::new(x_new,y_new,self.x_t.theta)

    }


    /// Converts an angle value to distance, the input is the angle data
    pub fn angle_to_distance(angle_l:f32,angle_r:f32,wheel_radius:f32)->(f32,f32){
        return (angle_l*wheel_radius,angle_r*wheel_radius)
    }

}



#[cfg(test)]
mod tests {
    #[test]
    fn odometry_model_test() {
        let mut newodommodel = super::OdometryModel::new(0.1);
        let wheel_l = 21.0;
        let wheel_r = 20.9;
        match newodommodel.update_get_radius_angle_distance(wheel_l,wheel_r){
            Ok(v)=>{
                    newodommodel.update_position_coords(v);
            },
            Err(e)=>{
                    newodommodel.update_position_coords_straight_line(e);
            }
        };
        println!("ODOM TEST");
        println!("x : {:?} , y: {:?}, theta: {:?}",newodommodel.x_t.x,newodommodel.x_t.y,newodommodel.x_t.theta);
    }

    #[test]
    fn odometry_model_zero_test() {
        let mut newodommodel = super::OdometryModel::new(20.0);
        let wheel_l = 20.0;
        let wheel_r = wheel_l;
        match newodommodel.update_get_radius_angle_distance(wheel_l,wheel_r){
            Ok(v)=>{

                    newodommodel.update_position_coords(v);
            },
            Err(e)=>{
                    newodommodel.update_position_coords_straight_line(e);
            }
        };

        println!("ZERO TEST");
        println!("x : {:?} , y: {:?}, theta: {:?}",newodommodel.x_t.x,newodommodel.x_t.y,newodommodel.x_t.theta);
    }


}
