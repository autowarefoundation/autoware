wget https://github.com/lgsvl/simulator/releases/download/2018.12-rc1/lgsvlsimulator-linux64-2018.12-rc1.zip -P bin
cd bin
unzip lgsvlsimulator-linux64-2018.12-rc1.zip
cd lgsvlsimulator-linux64-2018.12-rc1
export LGSVL_SIM_DIR_PATH=$PWD
echo "export LGSVL_SIM_DIR_PATH=$PWD" >> ~/.bashrc