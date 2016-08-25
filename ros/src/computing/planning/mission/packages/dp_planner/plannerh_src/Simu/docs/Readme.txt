Hard Coded Important information: 

#Mapppig: 

1- Path to vector map csv files: -> Simu-> PlannerTestDraw.cpp
#define VectorMapPath "/home/hatem/workspace/Data/VectorMap/"

2- Path to template kml file, for writing kml maps: Simu-> PlannerTestDraw.cpp
#define kmlTemplateFile "/home/hatem/workspace/Data/templates/PlannerX_MapTemplate.kml"

3- Origin point for Aisan Technology map. (cartisian origin) : PlannerH -> MappingHelpers.h -> GetTestToyotaOrigin()

4- lane IDs for predefined path .. : Simu-> PlannerTestDraw.cpp
#define PreDefinedPath  "11,333,1090,1704,147, 1791,801, 431, 1522, 372, 791, 1875, 1872,171,108,21,"


#Graphics and UI: 
1- use or not use GPU capabilities: Simu -> MainWindowWrapper.cpp -> bGPU (true , false)