# UAS-Test-Site-Antenna-Tracking
Lua targeting script for an off board Ardupilot gimbal controlled antenna system
AUTOPILOT CONFIGURATION
-------------------------------------

1. Open Mission Planner with Autopilot connected
2. Navigate to config tab and full paramter tree section
3. Search scr_enable and switch from 0 to 1
4. Upload paramters, disconnect, power cycle, and reconnect autopilot

FORMATTING FILES
--------------------------------------

- The lua script is ready to use, no change necessary
- create a text file (.txt) called "GCS_locations"
- input the data fields following GCS_locations_FORMAT or by mimicking format of GCS_locations_EXAMPLE files. 

UPLOADING FILES
-------------------------------------

1. Open Mission planner with Autopilot connected
2. Navigate to Config tab and MAVF tp section
3. Click on the folder cooresponding to the SD card installed in FCU
4. Click on APM subfolder and again into scripts sub folder
	note: if there is not a scripts sub folder already you can just make one titles "scripts"
5. Upload your lua script "gimbal control and targeting" as well as your formatted text file titled "GCS_locations" to this directory
6. Disconnect, power cycle, and reconnect autopilot. 
