use crate::odometry_motion_model::OdometryModel;
use crate::base::MotionUpdate2D;
use std::fs::File;
use std::io::{Read,Write};
use std::io::prelude::*;
use std::io::BufRead;


#[test]
fn odometry_model_test() {
    // file containing sample odometry data collected from webots 
    let odom_data  = get_raw_odometry_data().expect("Couldn't open file"); 
    let mut write_file = File::create("sample_data/x_y_t_values.txt").expect("Couldn't create");
    
    let mut debug_file = File::create("sample_data/x_y_t_values_debug.txt").expect("Couldn't create");
    




    
    let mut newodommodel  = OdometryModel::new(0.1054);
    let wheel_radius = 0.021;
    let mut initial_state = crate::base::Model2D::new(0.,0.,1.57);  
    let mut coords:Vec<(f32,f32,f32)> =vec![(0.,0.,1.57)]; 
    
    odom_data.iter().for_each(|m|{
        let initial = coords[coords.len()-1];
        let initial_state = crate::base::Model2D::new(initial.0,initial.1,initial.2);
        let initial_state = newodommodel
            .update_coords_odometry_stateless(initial_state.clone(),m.0*wheel_radius,m.1*wheel_radius);
        coords.push((initial_state.x,initial_state.y,initial_state.theta));
    });
    
    coords.iter().for_each(|m|{
        write!(write_file,"{}|{}|{}\n",m.0,m.1,m.2);
    });
    write_file.flush().unwrap();    
}




pub fn get_raw_odometry_data()->std::io::Result<Vec<(f32,f32)>>{
    let mut readings = std::fs::File::open("sample_data/test8008.txt")?;
    let mut readings_us = std::fs::File::open("sample_data/test_us8008.txt")?;
    let mut file_data = String::new();
    let mut file_data_us = String::new();
    let mut reader = std::io::BufReader::new(readings);
    let mut reader_us = std::io::BufReader::new(readings_us);
    let mut data_anal:Vec<f32>=Vec::new();

    let mut ws_data:Vec<f32>=Vec::new();
    let mut us_data:Vec<f32>=Vec::new();
    let mut laser_data:Vec<f32>=Vec::new();
    let mut lines = reader_us.lines();
    
    let mut odometry_data = Vec::<(f32,f32)>::new();

    while let Some(v) = lines.next(){
        if let Ok(mut m) = v{
            if let Ok(())=load_WS_to_array(&mut ws_data, &mut m){
                odometry_data.push((ws_data[0],ws_data[1]))
            }
        }
        ws_data.clear();
    } 
   println!("{:?}",odometry_data); 
    Ok(odometry_data)
}




fn load_WS_to_array(array:&mut Vec<f32>, text:&mut String)->Result<(),()>{
    let txt = text.clone();
    let mut a = txt.split_once(':');
    if let Some(p)=a{
        if p.0=="WS"{
            array.clear();
            p.1.split("|").for_each(|m|{
                if let Ok(f) = m.parse::<f32>(){
                    array.push(f)
                }
            });
            return Ok(())
        }else{
            return Err(())
        }
    }else{
        return Err(())
    }
}

