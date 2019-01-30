/// \file data_rw.cpp
/// \brief File operations for loading vector map files, loading kml map files and writing log .csv files
/// \author Hatem Darweesh
/// \date Jun 23, 2016

#include "op_utility/data_rw.h"
#include <stdlib.h>
#include <tinyxml.h>
#include <sys/stat.h>
#include "op_utility/utility.h"

namespace op_utility_ns
{

std::string DataRW::LoggingMainfolderName = "/autoware_openplanner_logs/";
std::string DataRW::ControlLogFolderName = "ControlLogs/";
std::string DataRW::GlobalPathLogFolderName = "GlobalPathLogs/";
std::string DataRW::PathLogFolderName = "TrajectoriesLogs/";
std::string DataRW::StatesLogFolderName = "BehaviorsLogs/";
std::string DataRW::SimulationFolderName = "SimulationData/";
std::string DataRW::KmlMapsFolderName = "KmlMaps/";
std::string DataRW::PredictionFolderName = "PredictionResults/";
std::string DataRW::TrackingFolderName = "TrackingLogs/";

DataRW::DataRW()
{
}

DataRW::~DataRW()
{
}

void DataRW::createLoggingFolder()
{
  std::string main_folder = UtilityH::getHomeDirectory() +
    DataRW::LoggingMainfolderName;
  int dir_err = mkdir(main_folder.c_str(),
      S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (-1 == dir_err) {
    std::cout << "Can't Create OpenPlanner Log Path!n" << std::endl;
  }

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::ControlLogFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::GlobalPathLogFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::PathLogFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::StatesLogFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::SimulationFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::PredictionFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    DataRW::TrackingFolderName;
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    "SimulatedCar1";
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    "SimulatedCar2";
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    "SimulatedCar3";
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    "SimulatedCar4";
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  main_folder = UtilityH::getHomeDirectory() + DataRW::LoggingMainfolderName +
    "SimulatedCar5";
  dir_err = mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void DataRW::writeLogData(
  const std::string & logFolder,
  const std::string & logTitle, const std::string & header,
  const std::vector<std::string> & logData)
{
  if (logData.size() < 2) {
    return;
  }

  std::ostringstream fileName;
  fileName << logFolder;
  fileName << logTitle;
  fileName << UtilityH::getFilePrefixHourMinuteSeconds();
  fileName << ".csv";

  std::ofstream f(fileName.str().c_str());

  if (f.is_open()) {
    if (header.size() > 0) {
      f << header << "\r\n";
    }
    for (unsigned int i = 0; i < logData.size(); i++) {
      f << logData.at(i) << "\r\n";
    }
  }

  f.close();
}

void DataRW::writeKMLFile(
  const std::string & fileName,
  const std::vector<std::string> & gps_list)
{
  TiXmlDocument kmldoc(
    UtilityH::getHomeDirectory() + DataRW::KmlMapsFolderName +
    "KmlTemplate.kml");

  bool bkmlFileLoaded = kmldoc.LoadFile();

  assert(bkmlFileLoaded == true);

  TiXmlElement * pElem = kmldoc.FirstChildElement();

  if (!pElem) {
    printf("\n Empty KML File !");
    return;
  }

  TiXmlElement * pV = 0;
  TiXmlHandle hKmlFile(pElem);

  //pV = hKmlFile.FirstChild("Folder").FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
  pV =
    hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild(
    "Placemark").FirstChild("LineString").FirstChild(
    "coordinates").Element();
  if (!pV) {
    pV =
      hKmlFile.FirstChild("Placemark").FirstChild("LineString").FirstChild(
      "coordinates").Element();
  }

  if (pV) {
    std::ostringstream val;
    val.precision(18);

    for (unsigned int i = 0; i < gps_list.size(); i++) {
      val << gps_list[i] << " ";
    }

    TiXmlText * text = new TiXmlText(val.str());
    pV->LinkEndChild(text);
  }

  kmldoc.SaveFile(fileName);
}

void DataRW::writeKMLFile(
  const std::string & fileName,
  const std::vector<std::vector<std::string>> & gps_list)
{
  TiXmlDocument kmldoc(
    UtilityH::getHomeDirectory() + DataRW::KmlMapsFolderName +
    "KmlTemplate.kml");

  bool bkmlFileLoaded = kmldoc.LoadFile();

  assert(bkmlFileLoaded == true);

  TiXmlElement * pElem = kmldoc.FirstChildElement();

  if (!pElem) {
    printf("\n Empty KML File !");
    return;
  }

  TiXmlNode * pV = 0;
  TiXmlNode * pPlaceMarkNode = 0;
  TiXmlElement * pDocument = 0;
  TiXmlHandle hKmlFile(pElem);

  //pV = hKmlFile.FirstChild("Folder").FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();

  pDocument = hKmlFile.FirstChild("Folder").FirstChild("Document").Element();
  pPlaceMarkNode =
    hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild(
    "Placemark").Node();

  if (!pDocument) {
    pDocument = hKmlFile.Element();
    pPlaceMarkNode = hKmlFile.FirstChild("Placemark").Node();
  }

//	        pV = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
//	        if(!pV)
//	                pV = hKmlFile.FirstChild( "Placemark" ).FirstChild("LineString").FirstChild("coordinates").Element();

  if (pDocument) {
    for (unsigned int l = 0; l < gps_list.size(); l++) {

      pV = pPlaceMarkNode->Clone();
      TiXmlElement * pElement = pV->FirstChild("LineString")->FirstChild(
        "coordinates")->ToElement();

      std::ostringstream val;
      val.precision(18);

      for (unsigned int i = 0; i < gps_list[l].size(); i++) {
        val << gps_list[l][i] << " ";
      }

      TiXmlText * text = new TiXmlText(val.str());
      pElement->LinkEndChild(text);

      pDocument->InsertEndChild(*pV);

    }

  }

  kmldoc.SaveFile(fileName);
}

SimpleReaderBase::SimpleReaderBase(
  const std::string & fileName,
  const int & nHeaders, const char & separator, const int & iDataTitles,
  const int & nVariablesForOneObject, const int & nLineHeaders,
  const std::string & headerRepeatKey)
{
  if (fileName.compare("d") != 0) {
    m_pFile = new std::ifstream(fileName.c_str(), std::ios::in);
    if (!m_pFile->is_open()) {
      printf("\n Can't Open Map File !, %s", fileName.c_str());
      return;
    }

    m_nHeders = nHeaders;
    m_iDataTitles = iDataTitles;
    m_nVarPerObj = nVariablesForOneObject;
    m_HeaderRepeatKey = headerRepeatKey;
    m_nLineHeaders = nLineHeaders;
    m_Separator = separator;
    m_pFile->precision(16);

    readHeaders();
  }
}

SimpleReaderBase::~SimpleReaderBase()
{
  if (m_pFile->is_open()) {
    m_pFile->close();
  }
}

bool SimpleReaderBase::readSingleLine(
  std::vector<std::vector<std::string>> & line)
{
  if (!m_pFile->is_open() || m_pFile->eof()) {
    return false;
  }

  std::string strLine, innerToken;
  line.clear();
  getline(*m_pFile, strLine);
  std::istringstream str_stream(strLine);

  std::vector<std::string> header;
  std::vector<std::string> obj_part;

  if (m_nVarPerObj == 0) {
    while (getline(str_stream, innerToken, m_Separator)) {
      obj_part.push_back(innerToken);
    }

    line.push_back(obj_part);
    return true;
  } else {
    int iCounter = 0;
    while (iCounter < m_nLineHeaders &&
      getline(str_stream, innerToken, m_Separator))
    {
      header.push_back(innerToken);
      iCounter++;
    }
    obj_part.insert(obj_part.begin(), header.begin(), header.end());

    iCounter = 1;

    while (getline(str_stream, innerToken, m_Separator)) {
      obj_part.push_back(innerToken);
      if (iCounter == m_nVarPerObj) {
        line.push_back(obj_part);
        obj_part.clear();

        iCounter = 0;
        obj_part.insert(obj_part.begin(), header.begin(), header.end());

      }
      iCounter++;
    }
  }

  return true;
}

int SimpleReaderBase::readAllData()
{
  if (!m_pFile->is_open()) {
    return 0;
  }

  m_AllData.clear();
  std::vector<std::vector<std::string>> singleLine;
  while (!m_pFile->eof()) {
    readSingleLine(singleLine);
    m_AllData.push_back(singleLine);
  }

  return m_AllData.size();
}

void SimpleReaderBase::readHeaders()
{
  if (!m_pFile->is_open()) {
    return;
  }

  std::string strLine;
  int iCounter = 0;
  m_RawHeaders.clear();
  while (!m_pFile->eof() && iCounter < m_nHeders) {
    getline(*m_pFile, strLine);
    m_RawHeaders.push_back(strLine);
    if (iCounter == m_iDataTitles) {
      parseDataTitles(strLine);
    }
    iCounter++;
  }
}

void SimpleReaderBase::parseDataTitles(const std::string & header)
{
  if (header.size() == 0) {
    return;
  }

  std::string innerToken;
  std::istringstream str_stream(header);
  m_DataTitlesHeader.clear();
  while (getline(str_stream, innerToken, m_Separator)) {
    if (innerToken.compare(m_HeaderRepeatKey) != 0) {
      m_DataTitlesHeader.push_back(innerToken);
    }
  }
}

bool GPSDataReader::readNextLine(GPSBasicData & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    data.lat = strtod(lineData.at(0).at(2).c_str(), NULL);
    data.lon = strtod(lineData.at(0).at(3).c_str(), NULL);
    data.alt = strtod(lineData.at(0).at(4).c_str(), NULL);
    data.distance = strtod(lineData.at(0).at(5).c_str(), NULL);

    return true;

  } else {
    return false;
  }
}

int GPSDataReader::readAllData(std::vector<GPSBasicData> & data_list)
{
  data_list.clear();
  GPSBasicData data;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

bool SimulationFileReader::readNextLine(SimulationPoint & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 6) {
      return false;
    }

    data.x = strtod(lineData.at(0).at(0).c_str(), NULL);
    data.y = strtod(lineData.at(0).at(1).c_str(), NULL);
    data.z = strtod(lineData.at(0).at(2).c_str(), NULL);
    data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
    data.c = strtod(lineData.at(0).at(4).c_str(), NULL);
    data.v = strtod(lineData.at(0).at(5).c_str(), NULL);
    data.name = lineData.at(0).at(6);

    return true;

  } else {
    return false;
  }
}

int SimulationFileReader::readAllData(SimulationData & data_list)
{
  data_list.simuCars.clear();
  SimulationPoint data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    if (count == 0) {
      data_list.startPoint = data;
    } else if (count == 1) {
      data_list.goalPoint = data;
    } else {
      data_list.simuCars.push_back(data);
    }

    count++;
  }

  return count;
}

bool LocalizationPathReader::readNextLine(LocalizationWayPoint & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    //data.t = strtod(lineData.at(0).at(0).c_str(), NULL);
    data.x = strtod(lineData.at(0).at(0).c_str(), NULL);
    data.y = strtod(lineData.at(0).at(1).c_str(), NULL);
    data.z = strtod(lineData.at(0).at(2).c_str(), NULL);
    data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
    data.v = strtod(lineData.at(0).at(4).c_str(), NULL);

    return true;

  } else {
    return false;
  }
}

int LocalizationPathReader::readAllData(
  std::vector<LocalizationWayPoint> & data_list)
{
  data_list.clear();
  LocalizationWayPoint data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//Nodes

AisanNodesFileReader::AisanNodesFileReader(
  const vector_map_msgs::NodeArray & _nodes)
: SimpleReaderBase("d", 1)
{
  if (_nodes.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  //TODO Fix PID and NID problem

  m_data_list.clear();
  AisanNode data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _nodes.data.size(); i++) {
    parseNextLine(_nodes.data.at(i), data);

    m_data_list.push_back(data);
    if (data.NID < m_min_id) {
      m_min_id = data.NID;
    }

    if (data.NID > max_id) {
      max_id = data.NID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).NID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanNodesFileReader::parseNextLine(
  const vector_map_msgs::Node & _rec,
  AisanNode & data)
{
  data.NID = _rec.nid;
  data.PID = _rec.pid;
}

AisanNodesFileReader::AisanNode * AisanNodesFileReader::getDataRowById(
  int _nid)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _nid - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).NID == _nid) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanNodesFileReader::readNextLine(AisanNode & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 2) {
      return false;
    }

    data.NID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.PID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanNodesFileReader::readAllData(std::vector<AisanNode> & data_list)
{
  m_data_list.clear();
  AisanNode data;
  //double logTime = 0;
  int max_id = std::numeric_limits<int>::min();
  while (readNextLine(data)) {
    m_data_list.push_back(data);
    if (data.NID < m_min_id) {
      m_min_id = data.NID;
    }

    if (data.NID > max_id) {
      max_id = data.NID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).NID - m_min_id) = &m_data_list.at(i);
  }

  data_list = m_data_list;
  return m_data_list.size();
}

//Points

AisanPointsFileReader::AisanPointsFileReader(
  const vector_map_msgs::PointArray & _points)
: SimpleReaderBase("d", 1)
{
  if (_points.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanPoints data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _points.data.size(); i++) {
    parseNextLine(_points.data.at(i), data);

    m_data_list.push_back(data);
    if (data.PID < m_min_id) {
      m_min_id = data.PID;
    }

    if (data.PID > max_id) {
      max_id = data.PID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).PID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanPointsFileReader::parseNextLine(
  const vector_map_msgs::Point & _rec,
  AisanPoints & data)
{
  data.B = _rec.b;
  data.Bx = _rec.bx;
  data.H = _rec.h;
  data.L = _rec.l;
  data.Ly = _rec.ly;
  data.MCODE1 = _rec.mcode1;
  data.MCODE2 = _rec.mcode2;
  data.MCODE3 = _rec.mcode3;
  data.PID = _rec.pid;
  data.Ref = _rec.ref;
}

AisanPointsFileReader::AisanPoints * AisanPointsFileReader::getDataRowById(
  int _pid)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _pid - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).PID == _pid) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanPointsFileReader::readNextLine(AisanPoints & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 10) {
      return false;
    }

    data.PID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.B = strtod(lineData.at(0)[1].c_str(), NULL);
    data.L = strtod(lineData.at(0)[2].c_str(), NULL);
    data.H = strtod(lineData.at(0)[3].c_str(), NULL);

    data.Bx = strtod(lineData.at(0)[4].c_str(), NULL);
    data.Ly = strtod(lineData.at(0)[5].c_str(), NULL);
    data.Ref = strtol(lineData.at(0).at(6).c_str(), NULL, 10);
    data.MCODE1 = strtol(lineData.at(0).at(7).c_str(), NULL, 10);
    data.MCODE2 = strtol(lineData.at(0).at(8).c_str(), NULL, 10);
    data.MCODE3 = strtol(lineData.at(0).at(9).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanPointsFileReader::readAllData(std::vector<AisanPoints> & data_list)
{
  m_data_list.clear();
  AisanPoints data;
  //double logTime = 0;
  int max_id = std::numeric_limits<int>::min();
  while (readNextLine(data)) {
    m_data_list.push_back(data);
    if (data.PID < m_min_id) {
      m_min_id = data.PID;
    }

    if (data.PID > max_id) {
      max_id = data.PID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).PID - m_min_id) = &m_data_list.at(i);
  }

  data_list = m_data_list;
  return m_data_list.size();
}

// Lines

AisanLinesFileReader::AisanLinesFileReader(
  const vector_map_msgs::LineArray & _nodes)
: SimpleReaderBase("d", 1)
{
  if (_nodes.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanLine data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _nodes.data.size(); i++) {
    parseNextLine(_nodes.data.at(i), data);

    m_data_list.push_back(data);
    if (data.LID < m_min_id) {
      m_min_id = data.LID;
    }

    if (data.LID > max_id) {
      max_id = data.LID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).LID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanLinesFileReader::parseNextLine(
  const vector_map_msgs::Line & _rec,
  AisanLine & data)
{
  data.BLID = _rec.blid;
  data.BPID = _rec.bpid;
  data.FLID = _rec.flid;
  data.FPID = _rec.fpid;
  data.LID = _rec.lid;
}

AisanLinesFileReader::AisanLine * AisanLinesFileReader::getDataRowById(
  int _lid)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _lid - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).LID == _lid) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanLinesFileReader::readNextLine(AisanLine & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    data.LID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.BPID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.FPID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.BLID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
    data.FLID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

    return true;
  } else {
    return false;
  }
}

int AisanLinesFileReader::readAllData(std::vector<AisanLine> & data_list)
{
  data_list.clear();
  AisanLine data;
  //double logTime = 0;

  int max_id = std::numeric_limits<int>::min();
  while (readNextLine(data)) {
    m_data_list.push_back(data);
    if (data.LID < m_min_id) {
      m_min_id = data.LID;
    }

    if (data.LID > max_id) {
      max_id = data.LID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).LID - m_min_id) = &m_data_list.at(i);
  }

  data_list = m_data_list;
  return m_data_list.size();
}

//dt Lanes (center lines)

AisanCenterLinesFileReader::AisanCenterLinesFileReader(
  const vector_map_msgs::DTLaneArray & _Lines)
: SimpleReaderBase("d", 1)
{
  if (_Lines.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanCenterLine data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _Lines.data.size(); i++) {
    parseNextLine(_Lines.data.at(i), data);

    m_data_list.push_back(data);
    if (data.DID < m_min_id) {
      m_min_id = data.DID;
    }

    if (data.DID > max_id) {
      max_id = data.DID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).DID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanCenterLinesFileReader::parseNextLine(
  const vector_map_msgs::DTLane & _rec, AisanCenterLine & data)
{
  data.Apara = _rec.apara;
  data.DID = _rec.did;
  data.Dir = _rec.dir;
  data.Dist = _rec.dist;
  data.LW = _rec.lw;
  data.PID = _rec.pid;
  data.RW = _rec.rw;
  data.cant = _rec.cant;
  data.r = _rec.r;
  data.slope = _rec.slope;
}

AisanCenterLinesFileReader::AisanCenterLine * AisanCenterLinesFileReader::getDataRowById(
  int _did)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _did - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).DID == _did) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanCenterLinesFileReader::readNextLine(AisanCenterLine & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 10) {
      return false;
    }

    data.DID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.Dist = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.PID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

    data.Dir = strtod(lineData.at(0)[3].c_str(), NULL);
    data.Apara = strtod(lineData.at(0)[4].c_str(), NULL);
    data.r = strtod(lineData.at(0)[5].c_str(), NULL);
    data.slope = strtod(lineData.at(0)[6].c_str(), NULL);
    data.cant = strtod(lineData.at(0)[7].c_str(), NULL);
    data.LW = strtod(lineData.at(0)[8].c_str(), NULL);
    data.RW = strtod(lineData.at(0)[9].c_str(), NULL);

    return true;
  } else {
    return false;
  }
}

int AisanCenterLinesFileReader::readAllData(
  std::vector<AisanCenterLine> & data_list)
{
  data_list.clear();
  AisanCenterLine data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//Lane

AisanLanesFileReader::AisanLanesFileReader(
  const vector_map_msgs::LaneArray & _lanes)
: SimpleReaderBase("d", 1)
{
  if (_lanes.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanLane data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _lanes.data.size(); i++) {
    parseNextLine(_lanes.data.at(i), data);

    m_data_list.push_back(data);
    if (data.LnID < m_min_id) {
      m_min_id = data.LnID;
    }

    if (data.LnID > max_id) {
      max_id = data.LnID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).LnID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanLanesFileReader::parseNextLine(
  const vector_map_msgs::Lane & _rec,
  AisanLane & data)
{
  data.BLID = _rec.blid;
  data.BLID2 = _rec.blid2;
  data.BLID3 = _rec.blid3;
  data.BLID4 = _rec.blid4;
  data.BNID = _rec.bnid;
  data.ClossID = _rec.clossid;
  data.DID = _rec.did;
  data.FLID = _rec.flid;
  data.FLID2 = _rec.flid2;
  data.FLID3 = _rec.flid3;
  data.FLID4 = _rec.flid4;

  data.FNID = _rec.fnid;
  data.JCT = _rec.jct;
  data.LCnt = _rec.lcnt;
  data.LaneChgFG = _rec.lanecfgfg;
  //data.LaneDir = _rec.;
  data.LaneType = _rec.lanetype;
  //data.LeftLaneId = _rec.;
  data.LimitVel = _rec.limitvel;
  data.LinkWAID = _rec.linkwaid;
  data.LnID = _rec.lnid;
  data.Lno = _rec.lno;
  data.RefVel = _rec.refvel;
  //data.RightLaneId = _rec.;
  data.RoadSecID = _rec.roadsecid;
  data.Span = _rec.span;
  //data.originalMapID = _rec.;

}

AisanLanesFileReader::AisanLane * AisanLanesFileReader::getDataRowById(
  int _lnid)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _lnid - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).LnID == _lnid) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanLanesFileReader::readNextLine(AisanLane & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 17) {
      return false;
    }

    data.LnID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.DID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.BLID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.FLID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
    data.BNID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);
    data.FNID = strtol(lineData.at(0).at(5).c_str(), NULL, 10);
    data.JCT = strtol(lineData.at(0).at(6).c_str(), NULL, 10);
    data.BLID2 = strtol(lineData.at(0).at(7).c_str(), NULL, 10);
    data.BLID3 = strtol(lineData.at(0).at(8).c_str(), NULL, 10);
    data.BLID4 = strtol(lineData.at(0).at(9).c_str(), NULL, 10);
    data.FLID2 = strtol(lineData.at(0).at(10).c_str(), NULL, 10);
    data.FLID3 = strtol(lineData.at(0).at(11).c_str(), NULL, 10);
    data.FLID4 = strtol(lineData.at(0).at(12).c_str(), NULL, 10);
    data.ClossID = strtol(lineData.at(0).at(13).c_str(), NULL, 10);
    data.Span = strtod(lineData.at(0).at(14).c_str(), NULL);
    data.LCnt = strtol(lineData.at(0).at(15).c_str(), NULL, 10);
    data.Lno = strtol(lineData.at(0).at(16).c_str(), NULL, 10);

    if (lineData.at(0).size() < 23) {
      return true;
    }

    data.LaneType = strtol(lineData.at(0).at(17).c_str(), NULL, 10);
    data.LimitVel = strtol(lineData.at(0).at(18).c_str(), NULL, 10);
    data.RefVel = strtol(lineData.at(0).at(19).c_str(), NULL, 10);
    data.RoadSecID = strtol(lineData.at(0).at(20).c_str(), NULL, 10);
    data.LaneChgFG = strtol(lineData.at(0).at(21).c_str(), NULL, 10);
    data.LinkWAID = strtol(lineData.at(0).at(22).c_str(), NULL, 10);

    if (lineData.at(0).size() > 23) {
      std::string str_dir = lineData.at(0).at(23);
      if (str_dir.size() > 0) {
        data.LaneDir = str_dir.at(0);
      } else {
        data.LaneDir = 'F';
      }
    }

//		data.LeftLaneId  = 0;
//		data.RightLaneId = 0;
//		data.LeftLaneId         = strtol(lineData.at(0).at(24).c_str(), NULL, 10);
//		data.RightLaneId        = strtol(lineData.at(0).at(25).c_str(), NULL, 10);

    return true;
  } else {
    return false;
  }
}

int AisanLanesFileReader::readAllData(std::vector<AisanLane> & data_list)
{
  data_list.clear();
  AisanLane data;
  //double logTime = 0;
  int max_id = std::numeric_limits<int>::min();

  while (readNextLine(data)) {
    m_data_list.push_back(data);
    if (data.LnID < m_min_id) {
      m_min_id = data.LnID;
    }

    if (data.LnID > max_id) {
      max_id = data.LnID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).LnID - m_min_id) = &m_data_list.at(i);
  }

  data_list = m_data_list;

  return m_data_list.size();
}

//Area

AisanAreasFileReader::AisanAreasFileReader(
  const vector_map_msgs::AreaArray & _areas)
: SimpleReaderBase("d", 1)
{
  if (_areas.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanArea data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _areas.data.size(); i++) {
    parseNextLine(_areas.data.at(i), data);

    m_data_list.push_back(data);
    if (data.AID < m_min_id) {
      m_min_id = data.AID;
    }

    if (data.AID > max_id) {
      max_id = data.AID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).AID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanAreasFileReader::parseNextLine(
  const vector_map_msgs::Area & _rec,
  AisanArea & data)
{
  data.AID = _rec.aid;
  data.ELID = _rec.elid;
  data.SLID = _rec.slid;
}

AisanAreasFileReader::AisanArea * AisanAreasFileReader::getDataRowById(
  int _aid)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _aid - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).AID == _aid) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanAreasFileReader::readNextLine(AisanArea & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 3) {
      return false;
    }

    data.AID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.SLID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.ELID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanAreasFileReader::readAllData(std::vector<AisanArea> & data_list)
{
  data_list.clear();
  AisanArea data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//Intersection

AisanIntersectionFileReader::AisanIntersectionFileReader(
  const vector_map_msgs::CrossRoadArray & _inters)
: SimpleReaderBase("d", 1)
{
  if (_inters.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanIntersection data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _inters.data.size(); i++) {
    parseNextLine(_inters.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanIntersectionFileReader::parseNextLine(
  const vector_map_msgs::CrossRoad & _rec, AisanIntersection & data)
{
  data.AID = _rec.aid;
  data.ID = _rec.id;
  data.LinkID = _rec.linkid;
}

AisanIntersectionFileReader::AisanIntersection * AisanIntersectionFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanIntersectionFileReader::readNextLine(AisanIntersection & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 3) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.AID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanIntersectionFileReader::readAllData(
  std::vector<AisanIntersection> & data_list)
{
  data_list.clear();
  AisanIntersection data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//StopLine

AisanStopLineFileReader::AisanStopLineFileReader(
  const vector_map_msgs::StopLineArray & _stopLines)
: SimpleReaderBase("d", 1)
{
  if (_stopLines.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanStopLine data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _stopLines.data.size(); i++) {
    parseNextLine(_stopLines.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanStopLineFileReader::parseNextLine(
  const vector_map_msgs::StopLine & _rec, AisanStopLine & data)
{
  data.ID = _rec.id;
  data.LID = _rec.lid;
  data.LinkID = _rec.linkid;
  data.SignID = _rec.signid;
  data.TLID = _rec.tlid;
}

AisanStopLineFileReader::AisanStopLine * AisanStopLineFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanStopLineFileReader::readNextLine(AisanStopLine & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.LID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.TLID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.SignID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanStopLineFileReader::readAllData(
  std::vector<AisanStopLine> & data_list)
{
  data_list.clear();
  AisanStopLine data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//RoadSign

AisanRoadSignFileReader::AisanRoadSignFileReader(
  const vector_map_msgs::RoadSignArray & _signs)
: SimpleReaderBase("d", 1)
{
  if (_signs.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanRoadSign data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _signs.data.size(); i++) {
    parseNextLine(_signs.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanRoadSignFileReader::parseNextLine(
  const vector_map_msgs::RoadSign & _rec, AisanRoadSign & data)
{
  data.ID = _rec.id;
  data.LinkID = _rec.linkid;
  data.PLID = _rec.plid;
  data.Type = _rec.type;
  data.VID = _rec.vid;
}

AisanRoadSignFileReader::AisanRoadSign * AisanRoadSignFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanRoadSignFileReader::readNextLine(AisanRoadSign & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.VID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.PLID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.Type = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanRoadSignFileReader::readAllData(
  std::vector<AisanRoadSign> & data_list)
{
  data_list.clear();
  AisanRoadSign data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//Signal

AisanSignalFileReader::AisanSignalFileReader(
  const vector_map_msgs::SignalArray & _signal)
: SimpleReaderBase("d", 1)
{
  if (_signal.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanSignal data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _signal.data.size(); i++) {
    parseNextLine(_signal.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanSignalFileReader::parseNextLine(
  const vector_map_msgs::Signal & _rec,
  AisanSignal & data)
{
  data.ID = _rec.id;
  data.LinkID = _rec.linkid;
  data.PLID = _rec.plid;
  data.Type = _rec.type;
  data.VID = _rec.vid;
}

AisanSignalFileReader::AisanSignal * AisanSignalFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanSignalFileReader::readNextLine(AisanSignal & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.VID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.PLID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.Type = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanSignalFileReader::readAllData(std::vector<AisanSignal> & data_list)
{
  data_list.clear();
  AisanSignal data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }

  return count;
}

//std::vector

AisanVectorFileReader::AisanVectorFileReader(
  const vector_map_msgs::VectorArray & _vectors)
: SimpleReaderBase("d", 1)
{
  if (_vectors.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanVector data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _vectors.data.size(); i++) {
    parseNextLine(_vectors.data.at(i), data);

    m_data_list.push_back(data);
    if (data.VID < m_min_id) {
      m_min_id = data.VID;
    }

    if (data.VID > max_id) {
      max_id = data.VID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).VID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanVectorFileReader::parseNextLine(
  const vector_map_msgs::Vector & _rec,
  AisanVector & data)
{
  data.Hang = _rec.hang;
  data.PID = _rec.pid;
  data.VID = _rec.vid;
  data.Vang = _rec.vang;
}

AisanVectorFileReader::AisanVector * AisanVectorFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).VID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanVectorFileReader::readNextLine(AisanVector & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 4) {
      return false;
    }

    data.VID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.PID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.Hang = strtod(lineData.at(0).at(2).c_str(), NULL);
    data.Vang = strtod(lineData.at(0).at(3).c_str(), NULL);

    return true;

  } else {
    return false;
  }
}

int AisanVectorFileReader::readAllData(std::vector<AisanVector> & data_list)
{
  data_list.clear();
  AisanVector data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//Curb

AisanCurbFileReader::AisanCurbFileReader(
  const vector_map_msgs::CurbArray & _curbs)
: SimpleReaderBase("d", 1)
{
  if (_curbs.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanCurb data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _curbs.data.size(); i++) {
    parseNextLine(_curbs.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanCurbFileReader::parseNextLine(
  const vector_map_msgs::Curb & _rec,
  AisanCurb & data)
{
  data.Height = _rec.height;
  data.ID = _rec.id;
  data.LID = _rec.lid;
  data.LinkID = _rec.linkid;
  data.Width = _rec.width;
  data.dir = _rec.dir;
}

AisanCurbFileReader::AisanCurb * AisanCurbFileReader::getDataRowById(int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanCurbFileReader::readNextLine(AisanCurb & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 6) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.LID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.Height = strtod(lineData.at(0).at(2).c_str(), NULL);
    data.Width = strtod(lineData.at(0).at(3).c_str(), NULL);
    data.dir = strtol(lineData.at(0).at(4).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(5).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanCurbFileReader::readAllData(std::vector<AisanCurb> & data_list)
{
  data_list.clear();
  AisanCurb data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

// RoadEdge

AisanRoadEdgeFileReader::AisanRoadEdgeFileReader(
  const vector_map_msgs::RoadEdgeArray & _edges)
: SimpleReaderBase("d", 1)
{
  if (_edges.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanRoadEdge data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _edges.data.size(); i++) {
    parseNextLine(_edges.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanRoadEdgeFileReader::parseNextLine(
  const vector_map_msgs::RoadEdge & _rec, AisanRoadEdge & data)
{
  data.ID = _rec.id;
  data.LID = _rec.lid;
  data.LinkID = _rec.linkid;
}

AisanRoadEdgeFileReader::AisanRoadEdge * AisanRoadEdgeFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanRoadEdgeFileReader::readNextLine(AisanRoadEdge & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 3) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.LID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanRoadEdgeFileReader::readAllData(
  std::vector<AisanRoadEdge> & data_list)
{
  data_list.clear();
  AisanRoadEdge data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//CrossWalk

AisanCrossWalkFileReader::AisanCrossWalkFileReader(
  const vector_map_msgs::CrossWalkArray & _crossWalks)
: SimpleReaderBase("d", 1)
{
  if (_crossWalks.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanCrossWalk data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _crossWalks.data.size(); i++) {
    parseNextLine(_crossWalks.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanCrossWalkFileReader::parseNextLine(
  const vector_map_msgs::CrossWalk & _rec, AisanCrossWalk & data)
{
  data.AID = _rec.aid;
  data.BdID = _rec.bdid;
  data.ID = _rec.id;
  data.LinkID = _rec.linkid;
  data.Type = _rec.type;
}

AisanCrossWalkFileReader::AisanCrossWalk * AisanCrossWalkFileReader::getDataRowById(
  int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanCrossWalkFileReader::readNextLine(AisanCrossWalk & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 5) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.AID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.Type = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.BdID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanCrossWalkFileReader::readAllData(
  std::vector<AisanCrossWalk> & data_list)
{
  data_list.clear();
  AisanCrossWalk data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//WayArea

AisanWayareaFileReader::AisanWayareaFileReader(
  const vector_map_msgs::WayAreaArray & _wayAreas)
: SimpleReaderBase("d", 1)
{
  if (_wayAreas.data.size() == 0) {
    return;
  }

  m_min_id = std::numeric_limits<int>::max();

  m_data_list.clear();
  AisanWayarea data;
  int max_id = std::numeric_limits<int>::min();

  for (unsigned int i = 0; i < _wayAreas.data.size(); i++) {
    parseNextLine(_wayAreas.data.at(i), data);

    m_data_list.push_back(data);
    if (data.ID < m_min_id) {
      m_min_id = data.ID;
    }

    if (data.ID > max_id) {
      max_id = data.ID;
    }
  }

  m_data_map.resize(max_id - m_min_id + 2);
  for (unsigned int i = 0; i < m_data_list.size(); i++) {
    m_data_map.at(m_data_list.at(i).ID - m_min_id) = &m_data_list.at(i);
  }
}

void AisanWayareaFileReader::parseNextLine(
  const vector_map_msgs::WayArea & _rec,
  AisanWayarea & data)
{
  data.AID = _rec.aid;
  data.ID = _rec.waid;
  //data.LinkID = _rec.;
}

AisanWayareaFileReader::AisanWayarea * AisanWayareaFileReader::getDataRowById(int _id)
{
  if (m_data_map.size() == 0) {
    return nullptr;
  }

  int index = _id - m_min_id;
  if (index >= 0 && index < m_data_map.size()) {
    return m_data_map.at(index);
  } else {
    for (unsigned int i = 0; i < m_data_list.size(); i++) {
      if (m_data_list.at(i).ID == _id) {
        return &m_data_list.at(i);
      }
    }
  }

  return nullptr;
}

bool AisanWayareaFileReader::readNextLine(AisanWayarea & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 3) {
      return false;
    }

    data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.AID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanWayareaFileReader::readAllData(std::vector<AisanWayarea> & data_list)
{
  data_list.clear();
  AisanWayarea data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

//Data Conn
bool AisanDataConnFileReader::readNextLine(DataConn & data)
{
  std::vector<std::vector<std::string>> lineData;
  if (readSingleLine(lineData)) {
    if (lineData.size() == 0) {
      return false;
    }
    if (lineData.at(0).size() < 4) {
      return false;
    }

    data.LID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
    data.SLID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
    data.SID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
    data.SSID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);

    return true;

  } else {
    return false;
  }
}

int AisanDataConnFileReader::readAllData(std::vector<DataConn> & data_list)
{
  data_list.clear();
  DataConn data;
  //double logTime = 0;
  int count = 0;
  while (readNextLine(data)) {
    data_list.push_back(data);
    count++;
  }
  return count;
}

} /* namespace UtilityHNS */
