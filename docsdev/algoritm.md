ALGORITHM Autonomous_Surface_vessel
BEGIN
    INITIALIZE System_Peripherals and Safety_Parameters
    
    REPEAT UNTIL Destination_Reached
        // 1. Persepsi (Sense)
        data = ACQUIRE_SENSOR_DATA(GPS, IMU, LiDAR, Camera)
        objects = DETECT_OBJECTS(data.Camera) via YOLO_Engine
        
        // 2. Perencanaan (Plan)
        base_path = CALCULATE_PATH(data.GPS, target_waypoint)
        safe_path = OBSTACLE_AVOIDANCE(base_path, objects, data.LiDAR)
        
        // 3. Aksi (Act)
        IF data.LiDAR.distance < Safety_Threshold THEN
            EXECUTE Emergency_Maneuver()
        ELSE
            EXECUTE Control_Steering(safe_path)
        ENDIF
        
        // 4. Monitoring
        UPDATE Mission_Status
    UNTIL Mission_Completed

    SHUTDOWN System
END
