
local PAN_SERVO_PORT = 5
local PAN_MAX_PWM = 2110
local PAN_MIN_PWM = 890
local PAN_MAX_ANGLE = 179.99
local PAN_MIN_ANGLE = -180.0
local PAN_OFFSET = -189 --add to go left
local TILT_SERVO_PORT = 6
local TILT_MAX_PWM = 2300
local TILT_MIN_PWM = 800
local TILT_MAX_ANGLE = 50.0
local TILT_MIN_ANGLE = -90.0
local CURR_PAN_PWM = 0
local CURR_TILT_PWM = 0

function make(xval, yval, zval)
    return {x=xval, y=yval, z=zval}
end

function plus(lhs, rhs)
    return make(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z)
end

function minus(lhs, rhs)
    return make(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)
end

function times(lhs, scale)
    return make(scale * lhs.x, scale * lhs.y, scale * lhs.z)
end

function dot(lhs, rhs)
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z
end

function tostr(val)
    return "(" .. val.x .. ", " .. val.y .. ", " .. val.z .. ")"
end

function intersectPoint(rayVector, rayPoint, planeNormal, planePoint)
    diff = minus(rayPoint, planePoint)
    prod1 = dot(diff, planeNormal)
    prod2 = dot(rayVector, planeNormal)
    prod3 = prod1 / prod2
    return minus(rayPoint, times(rayVector, prod3))
end

function haversine(x1, y1, x2, y2)
    r = 0.017453292519943295769236907684886127;
    x1= x1*r; x2= x2*r; y1= y1*r; y2= y2*r; dy = y2-y1; dx = x2-x1;
    a = (math.sin(dx/2))^2 + math.cos(x1) * math.cos(x2) * (math.sin(dy/2))^2; c = 2 * math.asin(math.sqrt(a)); d = 6372.8 * c;
    return d;
end

Vector = {}
function Vector.new( _x, _y, _z )
    return { x=_x, y=_y, z=_z }
end

function Vector.length(A)
    return math.sqrt(A.x^2 + A.y^2 + A.z^2)
end

function Vector.str(A)
    return string.format("X: %.2f, Y: %.2f, Z: %.2f", A.x, A.y, A.z)
end

function Vector.dot( A, B )
    return A.x*B.x + A.y*B.y + A.z*B.z
end

function Vector.cross( A, B )
    return { x = A.y*B.z - A.z*B.y,
             y = A.z*B.x - A.x*B.z,
             z = A.x*B.y - A.y*B.x }
end

function Vector.norm( A )
    local len = math.sqrt(A.x^2 + A.y^2 + A.z^2)
    return { x=A.x/len,
             y=A.y/len,
             z=A.z/len }
end

function Vector.scalar_triple( A, B, C )
    return Vector.dot( A, Vector.cross( B, C ) )
end

function Vector.vector_triple( A, B, C )
    return Vector.cross( A, Vector.cross( B, C ) )
end

function normalize(val, min, max)
    return (val - min) / (max - min)
end

function tilt_gimbal(angle)
    if angle >= TILT_MAX_ANGLE then
        SRV_Channels:set_output_pwm_chan(TILT_SERVO_PORT, TILT_MAX_PWM)
        return
    end
    if angle <= TILT_MIN_ANGLE then
        SRV_Channels:set_output_pwm_chan(TILT_SERVO_PORT, TILT_MIN_PWM)
        return
    end
    local pwm_range = TILT_MAX_PWM - TILT_MIN_PWM
    local angle_range = TILT_MAX_ANGLE - TILT_MIN_ANGLE

    local pwm_goal = pwm_range*(normalize(angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE))+TILT_MIN_PWM
    local pwm_output = pwm_goal
    if math.abs(CURR_TILT_PWM-pwm_goal)> 10 then
      if pwm_goal > CURR_TILT_PWM then
        pwm_goal = CURR_TILT_PWM + 10
         CURR_TILT_PWM = pwm_output
        SRV_Channels:set_output_pwm_chan(TILT_SERVO_PORT, math.floor(pwm_output))
        return
      end
      if pwm_goal < CURR_TILT_PWM then
        pwm_goal = CURR_TILT_PWM - 10
         CURR_TILT_PWM = pwm_output
        SRV_Channels:set_output_pwm_chan(TILT_SERVO_PORT, math.floor(pwm_output))
        return
        end
    end
    --gcs:send_text(6, string.format("Tilt Angle, PWM: %.1f, %.1f", angle, pwm_output))
    CURR_TILT_PWM = pwm_output
    SRV_Channels:set_output_pwm_chan(TILT_SERVO_PORT, math.floor(pwm_output))
end

function pan_gimbal(angle)

    --if angle >= PAN_MAX_ANGLE then
    --    SRV_Channels:set_output_pwm_chan(PAN_SERVO_PORT, PAN_MAX_PWM)
    --    return
    --end
    --if angle <= PAN_MIN_ANGLE then
    --   SRV_Channels:set_output_pwm_chan(PAN_SERVO_PORT, PAN_MIN_PWM)
    --    return
    --end

    while (angle > 180) do
        angle = angle - 360
    end
    while (angle < -180) do
        angle = angle + 360
    end

    local pwm_range = PAN_MAX_PWM - PAN_MIN_PWM
    local pwm_goal = pwm_range*(normalize(angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE))+PAN_MIN_PWM
    local pwm_output = pwm_goal
    if math.abs(CURR_PAN_PWM-pwm_goal)> 10 then
      if pwm_goal > CURR_PAN_PWM then
        pwm_output = CURR_PAN_PWM + 10
        CURR_PAN_PWM = pwm_output
        SRV_Channels:set_output_pwm_chan(PAN_SERVO_PORT, math.floor(pwm_output))
        return
      end
      if pwm_goal < CURR_PAN_PWM then
        pwm_output = CURR_PAN_PWM - 10
        CURR_PAN_PWM = pwm_output
        SRV_Channels:set_output_pwm_chan(PAN_SERVO_PORT, math.floor(pwm_output))
        return
        end
    end
    --gcs:send_text(6, string.format("Tilt Angle, PWM: %.1f, %.1f", angle, pwm_output))
    --SRV_Channels:set_output_pwm_chan(TILT_SERVO_PORT, math.floor(pwm_output))
    CURR_PAN_PWM = pwm_output
    SRV_Channels:set_output_pwm_chan(PAN_SERVO_PORT, math.floor(pwm_output))
end

local iter = 0

function calculate_pan_and_tilt(roll, pitch, yaw, target_lat, target_lng, target_alt, curr_pos)
    local theta = yaw
    local phi = pitch
    local psi = -roll

    --https://math.stackexchange.com/questions/1637464/find-unit-vector-given-roll-pitch-and-yaw
    local unit_vector_x = -math.sin(psi)*math.cos(theta)-math.cos(psi)*math.sin(phi)*math.sin(theta)
    local unit_vector_y = math.sin(psi)*math.sin(theta)-math.cos(psi)*math.sin(phi)*math.cos(theta)
    local unit_vector_z = math.cos(psi)*math.cos(phi)

    --gcs:send_text(6, string.format("Unit Vector: (%.1f, %.1f, %.1f)", unit_vector_x, unit_vector_y, unit_vector_z))

    --distance = r * (longitude 2 - longitude 1)
    --with longitude in radians and where r, the radius of the parallel of latitude, is
    --r = R * cosine latitude

    -- Lat is Y
    -- Long is X
    -- Alt is Z

    local rel_lat = (target_lat-curr_pos:lat()) / 10000000 * 111.32 * 1000 --m is N
    local rel_lng = (6372.8 * math.cos(math.rad(target_lat / 10000000))) * ((target_lng - curr_pos:lng()) / 1000000) --m is W
    local rel_alt = (target_alt-curr_pos:alt())/100 --meter
    --gcs:send_text(6, string.format("Relative Coords: (%.1f, %.1f, %.1f)", rel_lng, rel_lat, rel_alt))


    local planeNormal = make(unit_vector_x, unit_vector_y, unit_vector_z)
    local planePoint = make(0, 0, 0)
    local rayPoint = make(rel_lng, rel_lat, rel_alt)
    local rayVector = make(unit_vector_x, unit_vector_y, unit_vector_z)

    local pan_aim = intersectPoint(rayVector, rayPoint, planeNormal, planePoint)
    local pan_aim_length = math.sqrt( pan_aim.x^2 + pan_aim.y^2 + pan_aim.z^2)
    --local pan_aim_normal = Vector.new(pan_aim.x/pan_aim_length, pan_aim.y/pan_aim_length, pan_aim.z/pan_aim_length)
    local pan_aim_angle = math.atan(pan_aim.y,pan_aim.x)*180/math.pi

    --gcs:send_text(6, string.format("Pan AIM: (%.1f, %.1f, %.1f)", pan_aim.x, pan_aim.y, pan_aim.z))
    gcs:send_text(6, string.format("Pan AIM Angle: %.1f", pan_aim_angle))

    --https://math.stackexchange.com/a/1347612
    local dist_between_intersect_and_home = math.sqrt(rel_lng^2 + rel_lat^2)

    local tilt_aim_angle = math.deg(math.atan(math.sqrt((rel_lng-pan_aim.x)^2 + (rel_lat-pan_aim.y)^2 + (rel_alt-pan_aim.z)^2), pan_aim_length))
    if rel_alt < pan_aim.z then
        tilt_aim_angle = -tilt_aim_angle
    end
    --gcs:send_text(6, string.format("Rel Alt, Pan_Aim_Z: (%.1f, %.1f)", rel_alt, pan_aim.z))
    gcs:send_text(6, string.format("Tilt AIM Angle: %.1f", tilt_aim_angle))
    pan_gimbal((pan_aim_angle + math.deg(yaw) + PAN_OFFSET))
    tilt_gimbal((tilt_aim_angle))
end

function finite_horizon_position(curr_velocity, curr_position)
  local distance_traveled_north = curr_velocity[1]*5
  local new_lat = curr_position:lat() + (distance_traveled_north*360/(2*math.pi*6371000))
  local distance_traveled_east = curr_velocity[2]*5
  local new_lng = curr_position:lng() + (distance_traveled_north*360/(2*math.pi*6371000))
  find_target_theory(new_lat, new_lng) 
  end
  
function find_target_theory(theory_lat, theory_long)
  
  local input = io.open("/APM/scripts/GCS_locations.txt", "r")
  local counter = 0;
  io.input(input)
  PAN_SERVO_PORT = io.read("*number")
  PAN_MIN_PWM = io.read("*number")
  PAN_MAX_PWM = io.read("*number")
  TILT_SERVO_PORT = io.read("*number")
  TILT_MIN_PWM = io.read("*number")
  TILT_MAX_PWM = io.read("*number")
  
  num_GCS = io.read("*number")
  min_dist = 1000000000000000000
  target_lat = 0
  target_lng =0
  target_alt = 0
  GCS_target = 0
  for i=1, num_GCS do
    local GCS_lat, GCS_lng, GCS_alt = io.read("*number", "*number", "*number")
    --gcs:send_text(0, string.format("GCS loc: (%.2f, %.2f, %.2f)", GCS_lat, GCS_lng, GCS_alt))
    local dist = haversine(((theory_lat)/10000000), ((theory_long)/10000000), GCS_lat, GCS_lng)
    if (dist<min_dist)
    then
      min_dist = dist
      local theory_target_alt = GCS_alt*1000
      local theory_target_lat = GCS_lat*10000000
      local theory_target_lng = GCS_lng*10000000
      local theory_GCS_target = i
    end
    if (theory_GCS_target != GCS_target)
    then
      counter = counter + 1;
      if (counter == 5)
      then
        GCS_target = theory_GCS_target
        target_alt = theory_target_alt
        target_lat = theory_target_lat
        target_lng = theory_target_lng
        counter = 0
      end
        
    end
    
  end
end
function find_obstructions(curr_pos, target_lat, target_lng, target_alt)
  local vector_to_target = vector.new(((curr_pos:lat()/10000000)-target_lat), (((curr_pos:lng())/10000000)-target_lng), ((curr_pos:alt/1000)-target_alt))
  end
function find_target(curr_pos)
  local input = io.open("/APM/scripts/GCS_locations.txt", "r")
  io.input(input)
  PAN_SERVO_PORT = io.read("*number")
  PAN_MIN_PWM = io.read("*number")
  PAN_MAX_PWM = io.read("*number")
  TILT_SERVO_PORT = io.read("*number")
  TILT_MIN_PWM = io.read("*number")
  TILT_MAX_PWM = io.read("*number")
  
  num_GCS = io.read("*number")
  min_dist = 1000000000000000000
  target_lat = 0
  target_lng =0
  target_alt = 0
  GCS_target = 0
  for i=1, num_GCS do
    local GCS_lat, GCS_lng, GCS_alt = io.read("*number", "*number", "*number")
    --gcs:send_text(0, string.format("GCS loc: (%.2f, %.2f, %.2f)", GCS_lat, GCS_lng, GCS_alt))
    local dist = haversine(((curr_pos:lat())/10000000), ((curr_pos:lng())/10000000), GCS_lat, GCS_lng)
    if (dist<min_dist)
    then
      min_dist = dist
      target_alt = GCS_alt*1000
      target_lat = GCS_lat*10000000
      target_lng = GCS_lng*10000000
      GCS_target = i
    end
    
  end
end
    
      


function update() -- this is the loop which periodically runs
    --gcs:send_text(0, "running")
    local curr_pos = ahrs:get_position()    -- fetch the current location of the vehicle
    local home = ahrs:get_home()            -- fetch the origin position of the vehicle
    local curr_velocity = ahrs:groundspeed_vector()

    --gcs:send_text(6, tostring(vector_from_home))
     --vector needed to verify EKF origin is set. EKF origin is needed to get alt
    --gcs:send_text(0, tostring(temp))

    --local target_lat = home:lat()/10000000
    --local target_lng = home:lng()/10000000
    --local target_alt = home:alt()/10000000
    
    local roll = ahrs:get_roll()
    local pitch = ahrs:get_pitch()
    local yaw = ahrs:get_yaw()

   -- 38.650960, -76.420342, 100
    --38.654445, -76.420342, 100
     --quarter mile north

    if home and curr_pos and arming:is_armed() then
        
        find_target(curr_pos)
        
        gcs:send_text(0, string.format("Curr Pos: (%.5f, %.5f, %.2f)", curr_pos:lat(), curr_pos:lng(), curr_pos:alt()))
        gcs:send_text(0, string.format("Target: %.1f", GCS_target))
        gcs:send_text(0, string.format("Target: (%.4f, %.4f, %.4f)", target_lat, target_lng, target_alt))
        --gcs:send_text(6, string.format("Target location: (%.2f, %.2f, %.2f)", home:lat, home:lng, home:alt))
        calculate_pan_and_tilt(roll, pitch, yaw, target_lat, target_lng, target_alt, curr_pos)
        --gcs:send_text(0, "calculated")
        finite_horizon_position(curr_velocity, curr_pos)
    end

    return update, 5 -- reschedules the loop
  --end
end



return update() -- run immediately before starting to reschedule
