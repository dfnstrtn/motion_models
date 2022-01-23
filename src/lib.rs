pub mod odometry_motion_model;

pub mod base{
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

   
    /// All motion models in 2 dimensions employ this trait, which takes int odometry information
    /// (Total DISTANCE travelled by the wheels (angle*radius)) and gives the new coordinates as
    /// output 
    pub trait MotionUpdate2D{
        fn update_coords_odometry(&mut self, odom_l:f32, odom_r:f32)->Model2D;
    }
}


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
