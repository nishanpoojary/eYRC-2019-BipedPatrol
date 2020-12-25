global A = csvread('csv_matter.csv');  #do not change this line

################################################
#######Declare your global variables here#######
#global C = csvread('csv_output.csv');
global f_cut = 5;
global B;

#Global Variables for Low Pass Filter
global y_ax = 0;
global y_ay = 0;
global y_az = 0;

#Global Variables for High Pass Filter
global prev_x_gx = 0;
global prev_x_gy = 0;
global prev_x_gz = 0;
global y_gx = 0;
global y_gy = 0;
global y_gz = 0;

#Global Declaration for comp_filter_pitch
global prevtimeacc = 0.00;
global PrevGyroAngleX = 0;
global pitch = 0;

#Global Declaration comp_filter_roll
global prevtimegyro = 0;
global PrevGyroAngleY = 0;
global roll = 0;
################################################


function read_accel(axl,axh,ayl,ayh,azl,azh)  
  global f_cut;
  AccSens = 16384.00;
  #################################################
  ####### Write a code here to combine the ########
  #### HIGH and LOW values from ACCELEROMETER #####


  ax = bitshift(axh,8) + axl;
  ay = bitshift(ayh,8) + ayl;
  az = bitshift(azh,8) + azl;
  
  if (ax > 32767)
    ax = ax - 65536;
  endif
  if (ay > 32767)
    ay = ay - 65536;
  endif
  if (az > 32767)
    az = az - 65536;
  endif
  
  ax = ax/AccSens;
  ay = ay/AccSens;
  az = az/AccSens;
  
  #################################################


  ####################################################
  # Call function lowpassfilter(ax,ay,az,f_cut) here #
  lowpassfilter(ax,ay,az,f_cut)
  ####################################################

endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  global f_cut;
  GyroSens = 131.00;
  #################################################
  ####### Write a code here to combine the ########
  ###### HIGH and LOW values from GYROSCOPE #######
  
  gx = bitshift(gxh,8) + gxl;
  gy = bitshift(gyh,8) + gyl;
  gz = bitshift(gzh,8) + gzl;
  
  if (gx > 32767)
    gx = gx - 65536;
  endif
  if (gy > 32767)
    gy = gy - 65536;
  endif
  if (gz > 32767)
    gz = gz - 65536;
  endif
  
  gx = gx/GyroSens;
  gy = gy/GyroSens;
  gz = gz/GyroSens;
  
  #################################################


  #####################################################
  # Call function highpassfilter(ax,ay,az,f_cut) here #
  highpassfilter(gx,gy,gz,f_cut)
  #####################################################;

endfunction



function lowpassfilter(ax,ay,az,f_cut)
  #Global Declaration
  global y_ax;
  global y_ay;
  global y_az;
  
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  y_ax = (1-alpha)*ax + alpha*y_ax;
  
  y_ay = (1-alpha)*ay + alpha*y_ay;
  
  y_az = (1-alpha)*az + alpha*y_az;
  
  ################################################
  
endfunction



function highpassfilter(gx,gy,gz,f_cut)
  #Global Declaration
  global prev_x_gx;
  global prev_x_gy;
  global prev_x_gz;
  global y_gx;
  global y_gy;
  global y_gz;
  
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  y_gx = (1-alpha)*y_gx + (1-alpha)*(gx - prev_x_gx);
  prev_x_gx = gx;
  
  y_gy = (1-alpha)*y_gy + (1-alpha)*(gy - prev_x_gy);
  prev_x_gy = gy;
  
  y_gz = (1-alpha)*y_gz + (1-alpha)*(gz - prev_x_gz);
  prev_x_gz = gz;
  
  ################################################
  
endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)
  #Global Declaration
  global prevtimeacc;
  global PrevGyroAngleX;
  global pitch;

  ##############################################
  ####### Write a code here to calculate  ######
  ####### PITCH using complementry filter ######
  alpha = 0.03;
  elapsedTime = 0.01;

  AccAngleX = rad2deg(atan2(ay,az));
  if (AccAngleX > 90)
     AccAngleX = -(AccAngleX - 180);
  endif
  if (AccAngleX < -90)
     AccAngleX = -(AccAngleX + 180);
  endif
  
  GyroAngleX = PrevGyroAngleX + gx*elapsedTime;
  
  if (GyroAngleX > 90)
     GyroAngleX = -(GyroAngleX - 180);
  endif
  if (GyroAngleX < -90)
     GyroAngleX = -(GyroAngleX + 180);
  endif
  
  pitch = (1 - alpha)*GyroAngleX + alpha*AccAngleX;
  PrevGyroAngleX = pitch;
  
  ##############################################

endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)
  #Global Declaration
  global prevtimegyro;
  global PrevGyroAngleY;
  global roll;

  ##############################################
  ####### Write a code here to calculate #######
  ####### ROLL using complementry filter #######
  alpha = 0.03;
  elapsedTime = 0.01;

  AccAngleY = rad2deg(atan2(ax,az));
  
  if (AccAngleY > 90)
     AccAngleY = -(AccAngleY - 180);
  endif
  if (AccAngleY < -90)
     AccAngleY = -(AccAngleY + 180);
  endif
  
  GyroAngleY = PrevGyroAngleY + gy*elapsedTime;
  
  if (GyroAngleY > 90)
     GyroAngleY = -(GyroAngleY - 180);
  endif
  if (GyroAngleY < -90)
     GyroAngleY = -(GyroAngleY + 180);
  endif
  
  roll = (1 - alpha)*GyroAngleY + alpha*AccAngleY;
  PrevGyroAngleY = roll;
  
  ##############################################

endfunction 

function execute_code
  global B;
  global A;
  global y_ax = 0;
  global y_ay = 0;
  global y_az = 0;
  global y_gx = 0;
  global y_gy = 0;
  global y_gz = 0;
  global pitch = 0;
  global roll = 0;
  global PrevGyroAngleX = 0;
  global PrevGyroAngleY = 0;
  #global C;

  for n = 1:rows(A)                    #do not change this line
    
    ###############################################
    ####### Write a code here to calculate  #######
    ####### PITCH using complementry filter #######
    read_accel(A(n,2),A(n,1),A(n,4),A(n,3),A(n,6),A(n,5));
    read_gyro(A(n,8),A(n,7),A(n,10),A(n,9),A(n,12),A(n,11));
    comp_filter_pitch(y_ax,y_ay,y_az,y_gx,y_gy,y_gz);
    comp_filter_roll(y_ax,y_ay,y_az,y_gx,y_gy,y_gz);
    B(n,1) = pitch;
    B(n,2) = roll;
    ###############################################
    
  endfor
  #t = 1:1000
  #plot(t,B(1:1000,2))
  #hold on;
  #plot(t,C(1:1000,2))
  
  csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
