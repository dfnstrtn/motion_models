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

}


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
