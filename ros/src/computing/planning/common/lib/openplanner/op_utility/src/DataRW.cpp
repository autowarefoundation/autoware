/*
 * DataRW.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: hatem
 */

#include "DataRW.h"
#include <stdlib.h>
#include <tinyxml.h>
#include "UtilityH.h"

using namespace std;

namespace UtilityHNS
{

std::string DataRW::LoggingMainfolderName 	= "/SimuLogs/";
std::string DataRW::ControlLogFolderName 	= "ControlLogs/";
std::string DataRW::GlobalPathLogFolderName = "GlobalPathLogs/";
std::string DataRW::PathLogFolderName 		= "TrajectoriesLogs/";
std::string DataRW::StatesLogFolderName 	= "BehaviorsLogs/";
std::string DataRW::SimulationFolderName 	= "SimulationData/";
std::string DataRW::KmlMapsFolderName 		= "KmlMaps/";


DataRW::DataRW()
{
}

DataRW::~DataRW()
{
}

void DataRW::WriteLogData(const std::string& logFolder, const std::string& logTitle, const std::string& header, const std::vector<std::string>& logData)
{
	if(logData.size() < 2)
		return;

	ostringstream fileName;
	fileName << logFolder;
	fileName << logTitle;
	fileName << UtilityH::GetFilePrefixHourMinuteSeconds();
	fileName << ".csv";

	std::ofstream f(fileName.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < logData.size(); i++)
			f << logData.at(i) << "\r\n";
	}

	f.close();
}

void DataRW::WriteKMLFile(const string& fileName, const vector<string>& gps_list)
{
	TiXmlDocument kmldoc(UtilityH::GetHomeDirectory()+DataRW::KmlMapsFolderName + "KmlTemplate.kml");

	bool bkmlFileLoaded =  kmldoc.LoadFile();

	assert(bkmlFileLoaded== true);

	TiXmlElement* pElem = kmldoc.FirstChildElement();

	if(!pElem)
	{
		printf("\n Empty KML File !");
		return;
	}

	TiXmlElement* pV=0;
	TiXmlHandle hKmlFile(pElem);

	//pV = hKmlFile.FirstChild("Folder").FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
	pV = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
	if(!pV)
		pV = hKmlFile.FirstChild( "Placemark" ).FirstChild("LineString").FirstChild("coordinates").Element();

	if(pV)
	{
			ostringstream val;
			val.precision(18);

			for(unsigned int i =0; i < gps_list.size(); i++)
			{
				val << gps_list[i] <<  " ";
			}

			TiXmlText * text = new TiXmlText( val.str() );
			pV->LinkEndChild(text);
	}

	kmldoc.SaveFile(fileName);
}

void DataRW::WriteKMLFile(const string& fileName, const vector<vector<string> >& gps_list)
  {
	  TiXmlDocument kmldoc(UtilityH::GetHomeDirectory()+DataRW::KmlMapsFolderName + "KmlTemplate.kml");

	  	bool bkmlFileLoaded =  kmldoc.LoadFile();

	  	assert(bkmlFileLoaded== true);

	  	TiXmlElement* pElem = kmldoc.FirstChildElement();

	  	if(!pElem)
	  	{
	  		printf("\n Empty KML File !");
	  		return;
	  	}

	  	TiXmlNode* pV=0;
	  	TiXmlNode* pPlaceMarkNode=0;
	  	TiXmlElement* pDocument=0;
	  	TiXmlHandle hKmlFile(pElem);

	  	//pV = hKmlFile.FirstChild("Folder").FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();

	  	pDocument = hKmlFile.FirstChild("Folder").FirstChild("Document").Element();
	  	pPlaceMarkNode = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").Node();

	  	if(!pDocument)
	  	{
	  		pDocument = hKmlFile.Element();
	  		pPlaceMarkNode = hKmlFile.FirstChild( "Placemark" ).Node();
	  	}



//	  	pV = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
//	  	if(!pV)
//	  		pV = hKmlFile.FirstChild( "Placemark" ).FirstChild("LineString").FirstChild("coordinates").Element();


	  	if(pDocument)
	  	{
	  		for(unsigned int l = 0; l < gps_list.size(); l++)
	  		{

	  			pV = pPlaceMarkNode->Clone();
	  			TiXmlElement* pElement = pV->FirstChild("LineString")->FirstChild("coordinates")->ToElement();

	  			ostringstream val;
				val.precision(18);

				for(unsigned int i =0; i < gps_list[l].size(); i++)
				{
					val << gps_list[l][i] <<  " ";
				}

				TiXmlText * text = new TiXmlText( val.str() );
				pElement->LinkEndChild(text);

				pDocument->InsertEndChild(*pV);

	  		}

	  	}

	  	kmldoc.SaveFile(fileName);
  }

SimpleReaderBase::SimpleReaderBase(const string& fileName, const int& nHeaders,const char& separator,
		  const int& iDataTitles, const int& nVariablesForOneObject ,
		  const int& nLineHeaders, const string& headerRepeatKey)
{
	  m_pFile = new ifstream(fileName.c_str(), ios::in);
	  if(!m_pFile->is_open())
	  {
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

	ReadHeaders();
}

SimpleReaderBase::~SimpleReaderBase()
{
	if(m_pFile->is_open())
		m_pFile->close();
}

bool SimpleReaderBase::ReadSingleLine(vector<vector<string> >& line)
{
	if(!m_pFile->is_open() || m_pFile->eof()) return false;

	string strLine, innerToken;
	line.clear();
	getline(*m_pFile, strLine);
	istringstream str_stream(strLine);

	vector<string> header;
	vector<string> obj_part;

	if(m_nVarPerObj == 0)
	{
		while(getline(str_stream, innerToken, m_Separator))
		{
			obj_part.push_back(innerToken);
		}

		line.push_back(obj_part);
		return true;
	}
	else
	{
		int iCounter = 0;
		while(iCounter < m_nLineHeaders && getline(str_stream, innerToken, m_Separator))
		{
			header.push_back(innerToken);
			iCounter++;
		}
		obj_part.insert(obj_part.begin(), header.begin(), header.end());

		iCounter = 1;

		while(getline(str_stream, innerToken, m_Separator))
		{
			obj_part.push_back(innerToken);
			if(iCounter == m_nVarPerObj)
			{
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

int SimpleReaderBase::ReadAllData()
{
	if(!m_pFile->is_open()) return 0;

	m_AllData.clear();
	vector<vector<string> > singleLine;
	while(!m_pFile->eof())
	{
		ReadSingleLine(singleLine);
		m_AllData.push_back(singleLine);
	}

	return m_AllData.size();
}

void SimpleReaderBase::ReadHeaders()
{
	if(!m_pFile->is_open()) return;

	string strLine;
	int iCounter = 0;
	m_RawHeaders.clear();
	while(!m_pFile->eof() && iCounter < m_nHeders)
	{
		getline(*m_pFile, strLine);
		m_RawHeaders.push_back(strLine);
		if(iCounter == m_iDataTitles)
			ParseDataTitles(strLine);
		iCounter++;
	}
}

void SimpleReaderBase::ParseDataTitles(const string& header)
{
	if(header.size()==0) return;

	string innerToken;
	istringstream str_stream(header);
	m_DataTitlesHeader.clear();
	while(getline(str_stream, innerToken, m_Separator))
	{
		if(innerToken.compare(m_HeaderRepeatKey)!=0)
			m_DataTitlesHeader.push_back(innerToken);
	}
}

bool GPSDataReader::ReadNextLine(GPSBasicData& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.lat = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.lon = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.alt = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.distance = strtod(lineData.at(0).at(5).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int GPSDataReader::ReadAllData(vector<GPSBasicData>& data_list)
{
	data_list.clear();
	GPSBasicData data;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool SimulationFileReader::ReadNextLine(SimulationPoint& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 6) return false;

		data.x = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(1).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.c = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.v = strtod(lineData.at(0).at(5).c_str(), NULL);
		data.name = lineData.at(0).at(6);

		return true;

	}
	else
		return false;
}

int SimulationFileReader::ReadAllData(SimulationData& data_list)
{
	data_list.simuCars.clear();
	SimulationPoint data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		if(count == 0)
			data_list.startPoint = data;
		else if(count == 1)
			data_list.goalPoint = data;
		else
			data_list.simuCars.push_back(data);

		count++;
	}

	return count;
}

bool LocalizationPathReader::ReadNextLine(LocalizationWayPoint& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		//data.t = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.x = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(1).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.v = strtod(lineData.at(0).at(4).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int LocalizationPathReader::ReadAllData(vector<LocalizationWayPoint>& data_list)
{
	data_list.clear();
	LocalizationWayPoint data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanNodesFileReader::ReadNextLine(AisanNode& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 10) return false;

		data.NID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.PID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanNodesFileReader::ReadAllData(vector<AisanNode>& data_list)
{
	data_list.clear();
	AisanNode data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanPointsFileReader::ReadNextLine(AisanPoints& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 10) return false;

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

	}
	else
		return false;
}

int AisanPointsFileReader::ReadAllData(vector<AisanPoints>& data_list)
{
	data_list.clear();
	AisanPoints data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanLinesFileReader::ReadNextLine(AisanLine& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.LID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.BPID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.FPID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.BLID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.FLID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;
	}
	else
		return false;
}

int AisanLinesFileReader::ReadAllData(vector<AisanLine>& data_list)
{
	data_list.clear();
	AisanLine data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanCenterLinesFileReader::ReadNextLine(AisanCenterLine& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.DID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.Dist 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		data.Dir 	= strtod(lineData.at(0)[3].c_str(), NULL);
		data.Apara 	= strtod(lineData.at(0)[4].c_str(), NULL);
		data.r 		= strtod(lineData.at(0)[5].c_str(), NULL);
		data.slope 	= strtod(lineData.at(0)[6].c_str(), NULL);
		data.cant 	= strtod(lineData.at(0)[7].c_str(), NULL);
		data.LW 	= strtod(lineData.at(0)[8].c_str(), NULL);
		data.RW 	= strtod(lineData.at(0)[9].c_str(), NULL);

		return true;
	}
	else
		return false;
}

int AisanCenterLinesFileReader::ReadAllData(vector<AisanCenterLine>& data_list)
{
	data_list.clear();
	AisanCenterLine data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanLanesFileReader::ReadNextLine(AisanLane& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size() == 0) return false;
		if(lineData.at(0).size() < 17) return false;

		data.LnID		= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.DID		= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.BLID		= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.FLID		= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.BNID	 	= strtol(lineData.at(0).at(4).c_str(), NULL, 10);
		data.FNID		= strtol(lineData.at(0).at(5).c_str(), NULL, 10);
		data.JCT		= strtol(lineData.at(0).at(6).c_str(), NULL, 10);
		data.BLID2	 	= strtol(lineData.at(0).at(7).c_str(), NULL, 10);
		data.BLID3		= strtol(lineData.at(0).at(8).c_str(), NULL, 10);
		data.BLID4		= strtol(lineData.at(0).at(9).c_str(), NULL, 10);
		data.FLID2	 	= strtol(lineData.at(0).at(10).c_str(), NULL, 10);
		data.FLID3		= strtol(lineData.at(0).at(11).c_str(), NULL, 10);
		data.FLID4		= strtol(lineData.at(0).at(12).c_str(), NULL, 10);
		data.ClossID 	= strtol(lineData.at(0).at(13).c_str(), NULL, 10);
		data.Span 		= strtod(lineData.at(0).at(14).c_str(), NULL);
		data.LCnt	 	= strtol(lineData.at(0).at(15).c_str(), NULL, 10);
		data.Lno	  	= strtol(lineData.at(0).at(16).c_str(), NULL, 10);


		if(lineData.at(0).size() < 23) return true;

		data.LaneType	= strtol(lineData.at(0).at(17).c_str(), NULL, 10);
		data.LimitVel	= strtol(lineData.at(0).at(18).c_str(), NULL, 10);
		data.RefVel	 	= strtol(lineData.at(0).at(19).c_str(), NULL, 10);
		data.RoadSecID	= strtol(lineData.at(0).at(20).c_str(), NULL, 10);
		data.LaneChgFG 	= strtol(lineData.at(0).at(21).c_str(), NULL, 10);
		data.LinkWAID	= strtol(lineData.at(0).at(22).c_str(), NULL, 10);


		if(lineData.at(0).size() > 23)
		{
			string str_dir = lineData.at(0).at(23);
			if(str_dir.size() > 0)
				data.LaneDir 	= str_dir.at(0);
			else
				data.LaneDir  	= 'F';
		}

//		data.LeftLaneId  = 0;
//		data.RightLaneId = 0;
//		data.LeftLaneId 	= strtol(lineData.at(0).at(24).c_str(), NULL, 10);
//		data.RightLaneId 	= strtol(lineData.at(0).at(25).c_str(), NULL, 10);


		return true;
	}
	else
		return false;
}

int AisanLanesFileReader::ReadAllData(vector<AisanLane>& data_list)
{
	data_list.clear();
	AisanLane data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanAreasFileReader::ReadNextLine(AisanArea& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.AID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.SLID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.ELID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanAreasFileReader::ReadAllData(vector<AisanArea>& data_list)
{
	data_list.clear();
	AisanArea data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanIntersectionFileReader::ReadNextLine(AisanIntersection& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanIntersectionFileReader::ReadAllData(vector<AisanIntersection>& data_list)
{
	data_list.clear();
	AisanIntersection data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanStopLineFileReader::ReadNextLine(AisanStopLine& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.TLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.SignID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanStopLineFileReader::ReadAllData(vector<AisanStopLine>& data_list)
{
	data_list.clear();
	AisanStopLine data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanRoadSignFileReader::ReadNextLine(AisanRoadSign& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.VID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanRoadSignFileReader::ReadAllData(vector<AisanRoadSign>& data_list)
{
	data_list.clear();
	AisanRoadSign data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanSignalFileReader::ReadNextLine(AisanSignal& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.VID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanSignalFileReader::ReadAllData(vector<AisanSignal>& data_list)
{
	data_list.clear();
	AisanSignal data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}

	return count;
}

bool AisanVectorFileReader::ReadNextLine(AisanVector& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.VID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.PID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Hang 	= strtod(lineData.at(0).at(2).c_str(), NULL);
		data.Vang 	= strtod(lineData.at(0).at(3).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int AisanVectorFileReader::ReadAllData(vector<AisanVector>& data_list)
{
	data_list.clear();
	AisanVector data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanCurbFileReader::ReadNextLine(AisanCurb& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 6) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Height = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.Width 	= strtod(lineData.at(0).at(3).c_str(), NULL);
		data.dir 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanCurbFileReader::ReadAllData(vector<AisanCurb>& data_list)
{
	data_list.clear();
	AisanCurb data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanRoadEdgeFileReader::ReadNextLine(AisanRoadEdge& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanRoadEdgeFileReader::ReadAllData(vector<AisanRoadEdge>& data_list)
{
	data_list.clear();
	AisanRoadEdge data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool AisanDataConnFileReader::ReadNextLine(DataConn& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.LID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.SLID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.SID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.SSID 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanDataConnFileReader::ReadAllData(vector<DataConn>& data_list)
{
	data_list.clear();
	DataConn data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

} /* namespace UtilityHNS */
