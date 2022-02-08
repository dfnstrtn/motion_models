pub mod odometry_motion_model;

#[cfg(test)]
mod tests;
pub mod base{

    type Real=f32;
   
    #[derive(Copy,Clone)]
    pub struct Model2D{
        pub x:f32,
        pub y:f32,
        pub theta:f32
    }
    
    impl Model2D{
        
        pub fn new(x:f32,y:f32,theta:f32)->Model2D{
            Model2D{
                x,
                y,
                theta
            }
        }
    }

    // TODO : DOCUMENT!!
    /// A 3x3 jacobian matrix for updating values 
    /// Mostly used in kalman filters
    pub struct JacobianModel2D{
        pub data:[[f32;3];3]
    }
    impl JacobianModel2D{
        pub fn zeros()->JacobianModel2D{
            JacobianModel2D{
              data:[[0.0;3];3] 
            }
        }

        pub fn row(&mut self,index:usize, value:(f32,f32,f32))->Result<(),()>{
            if index<3{
                self.data[index][0] = value.0;
                self.data[index][1] = value.1;
                self.data[index][2] = value.2;
            }else{
                return Err(())
            }
            Ok(()) 
        }

        pub fn column(&mut self,index:usize, value:(f32,f32,f32))->Result<(),()>{
            if index<3{
                self.data[0][index] = value.0;
                self.data[1][index] = value.1;
                self.data[2][index] = value.2;
            }else{
                return Err(())
            }
            Ok(()) 
        }
        
        
        
    }




    /// All motion models in 2 dimensions employ this trait, which takes int odometry information
    /// (Total DISTANCE travelled by the wheels (angle*radius)) and gives the new coordinates as
    /// output 
    pub trait MotionUpdate2D{
        fn update_coords_odometry(&mut self, odom_l:f32, odom_r:f32)->Model2D;
        fn update_coords_odometry_stateless(&mut self, pos:Model2D,odom_l:f32,odom_r:f32)->Model2D;
        fn get_jacobian_stateless(&mut self, pos:Model2D, odom_l:f32, odom_r:f32)->JacobianModel2D;
    }


}


#[cfg(test)]
mod testsinternal {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
