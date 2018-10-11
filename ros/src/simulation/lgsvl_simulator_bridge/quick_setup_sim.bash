wget https://media.githubusercontent.com/media/lgsvl/simulator/binaries/auto-simulator-linux64-474.zip -P bin
cd bin
unzip auto-simulator-linux64-474.zip
cd auto-simulator-linux64-474
export LGSVL_SIM_DIR_PATH=$PWD
echo "export LGSVL_SIM_DIR_PATH=$PWD" >> ~/.bashrc