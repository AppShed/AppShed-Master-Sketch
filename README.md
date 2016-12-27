# ESP8266_softAP_Pro_AppShed
An Arduino sketch for the ESP8266 NodeMCU boards.

  This sketch combines the functionality of running a local Access Point, 
   or using the aREST Pro service.
   
   The default mode is "Local AP".
   
   To use the "aREST Pro Mode", the device must have pins D0 and D1 bridged at startup 
   (can remain bridged if those pins not used for anything else.)
  
  This sketch provides additional functions to support API calls from the AppShed app.js repo
   which can be found at https://github.com/AppShed/app.js 
