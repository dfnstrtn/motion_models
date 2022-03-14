use super::base;

pub struct ChangeParams{
    pub R:f32,
    pub alpha:f32,
    pub s:f32
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
    /// This function does not affect any value of the state of the differential robot model.
    /// If you intend to update the odometry motion model use the function 
    /// `update_odometry_readings()`
    pub fn update_get_radius_angle_distance(&mut self, odometry_l:f32,odometry_r:f32)->Result<ChangeParams,ChangeParams>{
        let L = self.base_length;
        let diff_l =  odometry_l - self.odometry_l;
        let diff_r = odometry_r - self.odometry_r;
        
        let alpha = (diff_r - diff_l)/L;
        let delta_s = (diff_l+diff_r)/2.0;
        
        if alpha==0.0{
            return Err(ChangeParams::new(0.0,0.0,delta_s))
        }

        let R = diff_l/alpha;
        Ok(ChangeParams::new(R,alpha,delta_s)) 
    }
    
    
    #[deprecated]
    pub fn update_get_radius_angle_distance_depr(&mut self, odometry_l:f32,odometry_r:f32)->Result<ChangeParams,ChangeParams>{
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



    /// Updates odometry readings 
    /// No function does this internally (except the trait implementations for now)
    /// If you want a hassle free robot working,call this function
    /// The reasin why this is not called internally is because the functions that return the
    /// jacobian are not supposed to update state in any way
    pub fn update_odometry_readings(&mut self,odometry_l:f32,odometry_r:f32){
        self.odometry_l=odometry_l;
        self.odometry_r=odometry_r;
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
    
    
    // TODO TEST 
    /// Updates position coordinates and returns the new position coordinates
    /// But stateless. You have to provide the inputs, useful for working with matrices 
    pub fn update_position_coords_stateless(state:base::Model2D,pos_change:ChangeParams)->base::Model2D{
        let y_new = pos_change.R*state.theta.cos() - pos_change.R*(state.theta + pos_change.alpha).cos() + state.y;
        let x_new = pos_change.R*(state.theta + pos_change.alpha).sin() - pos_change.R*state.theta.sin() + state.x;
        let theta_new = state.theta + pos_change.alpha;
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

    /// If the angle of curvature is zero update_position_coords will not work
    pub fn update_position_coords_straight_line_stateless(state:base::Model2D, distance:ChangeParams)->base::Model2D{
        let y_new = state.y + distance.s*state.theta.sin();
        let x_new = state.x + distance.s*state.theta.cos();
        base::Model2D::new(x_new,y_new,state.theta)

    }

    // TODO ADD DOCS!!
    // TODO test 
    /// Gets the jacobian of a function under normal conditions.
    /// If motion is in a straight line the radius of curvature is infinity , because of which you
    /// may  have to use  [OdometryModel::update_get_jacobian_straight_line_stateless]
    pub fn update_get_jacobian_stateless(state:base::Model2D, pos_change:ChangeParams)->base::JacobianModel2D{
        let mut data = base::JacobianModel2D::zeros();
        let y_jacobian = -pos_change.R*state.theta.sin() + pos_change.R*(state.theta + pos_change.alpha).sin();
        
        let x_jacobian = pos_change.R*(state.theta + pos_change.alpha).cos() - pos_change.R*state.theta.cos();

        let theta_jacobian = 0.0;

        data.column(2,(x_jacobian,y_jacobian,theta_jacobian));
        data
    }

    
    // TODO ADD DOCS!!
    // TODO test 
    pub fn update_get_jacobian_straight_line_stateless(state:base::Model2D, distance:ChangeParams)->base::JacobianModel2D{
        let mut data = base::JacobianModel2D::zeros();
        let y_jacobian = distance.s*state.theta.cos();
        let x_jacobian = -distance.s*state.theta.sin();
        
        // FIXME DELETE
        if cfg!(test)
        {
            println!("x_jacobian : {}, y_jacobian:{}",x_jacobian,y_jacobian);
        }

        let theta_jacobian = 0.0;
        data.column(2,(x_jacobian,y_jacobian,theta_jacobian));
        data
    }





    /// Converts an angle value to distance, the input is the angle data
    pub fn angle_to_distance(angle_l:f32,angle_r:f32,wheel_radius:f32)->(f32,f32){
        return (angle_l*wheel_radius,angle_r*wheel_radius)
    }

}

impl base::MotionUpdate2D for OdometryModel{
    
    fn update_coords_odometry(&mut self,odom_l:f32, odom_r:f32)->base::Model2D{ 
        let params = match self.update_get_radius_angle_distance(odom_l,odom_r){
            Ok(v)=>{
                    self.update_position_coords(v)
            },
            Err(e)=>{
                    self.update_position_coords_straight_line(e)
            }
        };
        self.update_odometry_readings(odom_l,odom_r);
        params
    }



    /// The stateless here does not mean parameters of the struct are not used 
    /// The update_get_radius_angle_distance does update the current odometry value 
    /// It does not however do anything to the varibale that maybe probabilistic like the x , y and
    /// theta coordinates 
    fn update_coords_odometry_stateless(&mut self,pos:base::Model2D,odom_l:f32, odom_r:f32)->base::Model2D{ 
        let params = match self.update_get_radius_angle_distance(odom_l,odom_r){
            Ok(v)=>{
                    Self::update_position_coords_stateless(pos,v)
            },
            Err(e)=>{
                    Self::update_position_coords_straight_line_stateless(pos,e)
            }
        };
        self.update_odometry_readings(odom_l,odom_r);
        params
    }


    /// If working in an environment where you have to get the jacobian and the updated odometry 
    /// get the jacobian first. The functions `update_coords_odometry_{}_stateless` change the value
    /// of odometry of the Model internally which affects the jacobian values
    fn get_jacobian_stateless(&mut self, pos:base::Model2D, odom_l:f32, odom_r:f32)->base::JacobianModel2D{
        let params = match self.update_get_radius_angle_distance(odom_l,odom_r){
            Ok(v)=>{
                    Self::update_get_jacobian_stateless(pos,v)
            },
            Err(e)=>{
                    Self::update_get_jacobian_straight_line_stateless(pos,e)
            }
        };
        params
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

    #[test]
    fn stateless_odometry_model_test(){
        use super::base::MotionUpdate2D;
        let mut initial_state = super::base::Model2D::new(0.,0.,0.);
        let mut newodommodel = super::OdometryModel::new(0.1);
        let wheel_l = 21.0;
        let wheel_r  =20.9;
        initial_state = newodommodel.update_coords_odometry_stateless(initial_state,wheel_l,wheel_r);
    
        println!("STATELESS ODOM TEST");
        println!("x : {:?} , y: {:?}, theta: {:?}",initial_state.x,initial_state.y,initial_state.theta);


    }



    // TODO TEST !!
    #[test]
    fn stateless_odometry_jacobian_test(){
        use super::base::MotionUpdate2D;
        let mut initial_state = super::base::Model2D::new(0.,0.,0.5);  
        let mut newodommodel = super::OdometryModel::new(0.1);
        let wheel_l = 21.0;
        let wheel_r  =20.9;
        

        let jacobian = newodommodel.get_jacobian_stateless(initial_state.clone(),wheel_l,wheel_r);
        

        let state = newodommodel.update_coords_odometry_stateless(initial_state.clone(),wheel_l,wheel_r);

        
        println!("NON LINE");
        jacobian.data.iter().for_each(|m|{
            for p in m{
                print!("{} ",p);
            }
            println!()
        });


        
        // straight line condition 
        let mut newnewodommodel = super::OdometryModel::new(0.1);
        let wheel_l = 20.0;
        let wheel_r  =20.0;
        
        let jacobian = newnewodommodel.get_jacobian_stateless(initial_state.clone(),wheel_l,wheel_r);

        let state = newnewodommodel.update_coords_odometry_stateless(initial_state.clone(),wheel_l,wheel_r);


        println!("NON LINE");
        jacobian.data.iter().for_each(|m|{
            for p in m{
                print!("{} ",p);
            }
            println!()
        });

    }






    #[test]
    fn files_odometry_model_test(){
        use super::base::MotionUpdate2D;
        let mut initial_state = super::base::Model2D::new(0.,0.,0.);
        let mut newodommodel = super::OdometryModel::new(0.1);
        let wheel_l = 21.0;
        let wheel_r  =20.9;
        initial_state = newodommodel.update_coords_odometry_stateless(initial_state,wheel_l,wheel_r);
    
        println!("STATELESS ODOM TEST");
        println!("x : {:?} , y: {:?}, theta: {:?}",initial_state.x,initial_state.y,initial_state.theta);


    }











}
