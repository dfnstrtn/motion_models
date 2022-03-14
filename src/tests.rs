use crate::odometry_motion_model::OdometryModel;
use crate::base::MotionUpdate2D;
use std::fs::File;
use std::io::{Read,Write};
use std::io::prelude::*;
use std::io::BufRead;


#[test]
fn us_ws_model_test(){
    // file containing sample odometry data collected from webots 
    
    let odom_data_n = file_read_odom_accurate(std::path::Path::new("sample_data/ws_pos8008.txt"), Path::new("sample_data/abs_pos8008.txt")).unwrap();



    let odom_data  = get_raw_odometry_data().expect("Couldn't open file"); 
    let mut write_file = File::create("sample_data/your_values.txt").expect("Couldn't create");

    let mut abs_file = File::create("sample_data/x_y_t_abs_values.txt").expect("Couldn't create");

    let mut debug_file = File::create("sample_data/x_y_t_values_debug.txt").expect("Couldn't create");
     
    

    let mut newodommodel  = OdometryModel::new(0.1054);
    let wheel_radius = 0.021;
    let mut initial_state = crate::base::Model2D::new(0.,0.,1.57);  
    let mut coords:Vec<(f32,f32,f32)> =vec![(0.,0.,1.57)]; 
    let mut abs_coords:Vec<(f32,f32,f32)> = Vec::new();


    odom_data_n.iter().for_each(|m|{
        let initial = coords[coords.len()-1];
        let initial_state = crate::base::Model2D::new(initial.0,initial.1,initial.2);
        let initial_state = newodommodel
            .update_coords_odometry_stateless(initial_state.clone(),m.0.0*wheel_radius,m.0.1*wheel_radius);
        coords.push((initial_state.x,initial_state.y,initial_state.theta));
        abs_coords.push((m.1.0,m.1.1,1.57));
    });
    


    coords.iter().for_each(|m|{
      write!(write_file,"{}|{}|{}\n",m.0,m.1,m.2);
    });

    abs_coords.iter().for_each(|m|{
        write!(abs_file,"{}|{}|{}\n",m.0,m.1,m.2);
    });
    write_file.flush().unwrap();
    abs_file.flush().unwrap();
}













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




use std::io::{BufReader};
use std::path::Path;
pub fn file_read_odom_accurate(path_odometry:&Path, path_abs:&Path)->std::io::Result<Vec<( (f32,f32),(f32,f32) )>>{
    let mut odomfile = File::open(path_odometry)?;
    let mut odom_reader  = BufReader::new(&mut odomfile);
    let mut odom_str = String::new();

    let mut absfile = File::open(path_abs)?;
    let mut abs_reader  = BufReader::new(&mut absfile);
    let mut abs_str = String::new();
    
    let mut output_vec = Vec::<((f32,f32),(f32,f32))>::new();

    loop{
        let mut datas = ((0.,0.),(0.,0.));
        if let Ok(p) = odom_reader.read_line(&mut odom_str){
            if p==0{
                break;
            }
            odom_str = odom_str.replace("\r\n","");
            //println!("ODOM STR {}",odom_str);
            let mut odom_iter = odom_str.split("|");
            let odom_l = odom_iter.next().unwrap().parse::<f32>().unwrap_or_else(|_|{0.0});
            let odom_r = odom_iter.next().unwrap().parse::<f32>().unwrap_or_else(|_|{0.0});
            datas.0.0=odom_l;
            datas.0.1=odom_r;
            //println!("ODOM {},{}",odom_l,odom_r);
            odom_str.clear();
        }else{
            println!("EOF");
            break;
        }

        if let Ok(p) = abs_reader.read_line(&mut abs_str){
            if p==0{
                break;
            }
            abs_str = abs_str.replace("\r\n","");
            let mut abs_iter = abs_str.split("|");
            let x = abs_iter.next().unwrap().parse::<f32>();
            let y = abs_iter.next().unwrap().parse::<f32>();
            datas.1.0=x.unwrap_or_else(|_|{0.0});
            datas.1.1=y.unwrap_or_else(|_|{0.0});
            //println!("ABS {},{}",datas.1.0,datas.1.1);
            abs_str.clear();
        }else{
            break;
        }
        output_vec.push(datas);
    }
    
    Ok(output_vec)
}



