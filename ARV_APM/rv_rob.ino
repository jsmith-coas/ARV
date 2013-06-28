/*****************************************
* Sensors - Read the A-frame state
*****************************************/
static void read_aframe(void)
{
    //read aft sensor
    if (g.aframe_aft_pin != -1) {                                               //Sensor exists, read value
        if (check_digital_pin(g.aframe_aft_pin) != aframe.aft_sensor_state) {   //Sensor has changed
            if (aframe.aft_sensor_count < 127) {                              //Sensor hasn't rolled over error
                aframe.aft_sensor_count++;
            } else {
                aframe.aft_sensor_state = 0;                                    //Sensor has an issue, set aframe logic to deployed. 
                aframe.aft_sensor_count = 0;              
            }
            
            if (aframe.aft_sensor_count >= g.aframe_debounce) {
                aframe.aft_sensor_state = !aframe.aft_sensor_state;           //Invert aft sensor state
                aframe.aft_sensor_count = 0;
                gcs_send_text_fmt(PSTR("Aframe AFT %i"), aframe.aft_sensor_state);                
            }     
            
            aframe.detected_time_ms = millis();                            
        }
    }
    
    //read foward sensor
    if (g.aframe_for_pin != -1) {                                               //Sensor exists, read value
        if (check_digital_pin(g.aframe_for_pin) != aframe.for_sensor_state) {   //Sensor has changed
            if (aframe.for_sensor_count < 127) {                              //Sensor hasn't rolled over error
                aframe.for_sensor_count++;
            } else {
                aframe.for_sensor_state = 1;                                    //Sensor has an issue, set aframe logic to deployed.
                aframe.for_sensor_count = 0;
            }
             
            if (aframe.for_sensor_count >= g.aframe_debounce) {
                aframe.for_sensor_state = !aframe.for_sensor_state;           //Invert forward sensor state
                aframe.for_sensor_count = 0;        
                gcs_send_text_fmt(PSTR("Aframe FOR %i"), aframe.for_sensor_state);                
            }  
            
            aframe.detected_time_ms = millis();                            
        }
    }    
    
    // no aframe move recently detected - reset after 1 second of inactivity   
    if (millis() > aframe.detected_time_ms + 1000) { 
        aframe.aft_sensor_count = 0;
        aframe.for_sensor_count = 0;
        }    
}


/*****************************************
* Steering - Set the aframe/winch/camera control servos
*****************************************/
static void set_winch(void) {
    if (control_mode == MANUAL) {                                              // Manual winch control is enabled only in MANUAL mode, not LEARNING
        g.channel_winch_clutch.radio_out = hal.rcin->read(CH_WINCH_CLUTCH);    // Read winch servo commands through
        g.channel_winch_motor.radio_out  = hal.rcin->read(CH_WINCH_MOTOR);     // Read winch motor commands through
    } //else let the ctd_cast_do function set the a-frame/winch servos
    
    ctd_cast_do();  // Check to see if we need to be casting the CTD
  
    // Limit motor speed depending on A-frame position
        //NOTE: sensor_state is HIGH when open, LOW when closed (pulls to ground)
    if (aframe.for_sensor_state == 0) {                                                // --Aframe is retracted
        g.channel_winch_motor.radio_out = g.channel_winch_motor.radio_trim;               // Disable the winch motor  
        // NOTE: To get the full 180° range of the servo, RC_INPUT_MAX_PULSEWIDTH and RC_INPUT_MIN_PULSEWIDTH must be changed in libraries/AP_HAL/RCInput.h    
        g.channel_camera_servo.radio_out = g.channel_camera_servo.radio_max;         // Point camera forward           
    } else if (aframe.aft_sensor_state == 1) {                                         // --Aframe is retracting
        g.channel_winch_motor.radio_out = constrain_int16(g.channel_winch_motor.radio_out, // Limit winch motor to slow speed
                                                          g.channel_winch_motor.radio_trim, 
                                                          g.w_motor_slow);
        g.channel_camera_servo.radio_out = g.channel_camera_servo.radio_min;           // Point camera aft                                                                                                                                   
    } else {                                                                           // --Aframe is extended
        g.channel_winch_motor.radio_out = constrain_int16(g.channel_winch_motor.radio_out, // Limit winch motor to high speed
                                                          g.channel_winch_motor.radio_trim, 
                                                          g.channel_winch_motor.radio_max);
        g.channel_camera_servo.radio_out = g.channel_camera_servo.radio_min;           // Point camera aft                                                                                                                                                                                             
    }

}  

/*****************************************
* Commands_logic - CTD Cast Verify Function
*****************************************/
static bool verify_ctd_cast()
{
    if (ctd.cast_depth_m > 0) {   // There is a CTD cast at this waypoint
        if (ctd.cast_end_time_ms == 0) {    // The cast has not yet been started happened, lets start it
            ctd.cast_end_time_ms = millis() + CTD_DEPLOY_TIME_MS  + (ctd.cast_depth_m * g.ctd_depth_to_time_ms); 
            gcs_send_text_fmt(PSTR("Started CTD, %im, %ius"), ctd.cast_depth_m, (ctd.cast_end_time_ms - millis()));                          
            return false;
        } else {
            gcs_send_text_fmt(PSTR("Current CTD, remaining %ius"), (ctd.cast_end_time_ms - millis()));                          
            return ctd.cast_done;  // Check to see what the status of the CTD cast is
        }
    }
}   


/*****************************************
* Commands_logic - CTD Cast Do Function
*****************************************/
static void ctd_cast_do()
{
    if (ctd.cast_end_time_ms > 0  && !ctd.cast_done) {  // CTD underway
        if (millis() < (ctd.cast_end_time_ms * CTD_TIME_SNAG_FACTOR)) {  // CTD cast still in allotted time
            if (millis() < ctd.cast_end_time_ms) {
                winch_freefall();
            } else {
                winch_retract(g.w_motor_sample);        // winch to sample speed
                if (aframe.for_sensor_state == 0) {
                    winch_hold();                    
                    ctd.cast_done = true;
                    gcs_send_text_fmt(PSTR("CTD Successful"));                    
                }                  
            } 
        } else {
            gcs_send_text_fmt(PSTR("CTD Snagged"));                    
            ctd.cast_done = true;
        }
    // } else { // CTD cast is complete or not required, don't need to do anything here
    }
}


/*****************************************
* Steering - Winch/servo basic functions
*****************************************/
static void winch_hold()
{
    g.channel_winch_motor.radio_out = g.channel_winch_motor.radio_trim;          // turn off the winch's motor   
    g.channel_winch_clutch.radio_out = g.channel_winch_clutch.radio_min;         // engage the winch clutch
}    
  
static void winch_freefall()
{
    g.channel_winch_motor.radio_out = g.channel_winch_motor.radio_trim;          // turn off the winch's motor   
    g.channel_winch_clutch.radio_out = g.channel_winch_clutch.radio_max;         // dis-engage the winch clutch
}

static void winch_retract(uint16_t winch_speed)
{
    g.channel_winch_clutch.radio_out = g.channel_winch_clutch.radio_min;         // engage the winch clutch
    g.channel_winch_motor.radio_out = winch_speed;                               // turn on the winch's motor to winch_speed
}  

/*****************************************
* Commands - Setup for next CTD cast, called from set_next_WP()
*****************************************/
static void ctd_cast_set_for_next()
{
    ctd.cast_end_time_ms = 0;  // reset the cast timer 
    ctd.cast_done = false;     // reset the bool completed flag
    
    // NOTE: using the WayPoints intended altitude since it isn't used on the rover and Mission Planner changes were not working
    ctd.cast_depth_m = constrain_int16(next_WP.alt/100, 0, g.ctd_max_depth); //convert from cm to m for cast depth, constrain to winch line length
}
