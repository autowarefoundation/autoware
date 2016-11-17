### My AutowareDider  ###

### Project Description ###
This project is modified from Autoware https://github.com/CPFL/Autoware. Modication:

1) make it an Android Studio app
2) break inner classes into its own classes
3) make all socket calls in working thread.
4) replaced c/c++ sockets with Java sockets.

### To run, may  ###
This is the way how run it, since it did not work, some steps may need change, please modify here if you know how

1) run Autoware in Ubuntu 14.x
2) load sample_morijama_data  
3) enable the Tablet interface via 'Interface' tab.
4) Additional info

   a) install AutowareRoute.apk. found at @https://github.com/CPFL/Autoware/tree/master/ui/tablet/AutowareRoute, then The 'Nav' button will not crash. Click the 'Nav' button launches AutowareRoute. A configure dialog pops first. All in Japanese and I haven't had time to translate them. If you know the words in English please, help out.

  b) 'Pursuit' button will end the app - That is what the code says and the app acts, don't ask me why.

Screenshots

![main](https://github.com/mingrutar/MyAutowareRider/blob/master/screenshots/MyAutowareRider.png?raw=true)
