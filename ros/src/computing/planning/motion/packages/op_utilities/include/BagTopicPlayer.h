/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BAGTOPICPLAYER_H_
#define BAGTOPICPLAYER_H_

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/player.h>

#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif


namespace UtilityHNS
{

#define MAX_RECORDS_BUFFER 25

template <class T>
class BagTopicPlayer
{
public:
	void InitPlayer(const rosbag::Bag& _bag, const std::string& topic_name)
	{
		if(_bag.getSize() == 0) return;

		rosbag::View view(_bag);
		std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
		std::vector<std::string> topics_to_subscribe;

		std::set<std::string> bagTopics;

		BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos)
		{
			bagTopics.insert(info->topic);
		}

		if (bagTopics.find(topic_name) == bagTopics.end())
		{
			ROS_WARN_STREAM("Can't Find LIDAR Topic in ROSBag File :" << topic_name);
			return;
		}
		else
		{
			topics_to_subscribe.push_back(std::string(topic_name));
		}

		m_BagView.addQuery(_bag, rosbag::TopicQuery(topics_to_subscribe));
		m_ViewIterator = m_BagView.begin();
		m_StartTime = m_BagView.getBeginTime();
		m_bReadNext = true;
	}

	bool ReadNext(boost::shared_ptr<T>& msg, ros::Time* pSyncTime)
	{

		if(m_ViewIterator != m_BagView.end())
		{
			if(m_bReadNext == true)
			{
				if(m_iPlayHead < m_PrevRecords.size())
				{
					m_CurrRecord = m_PrevRecords.at(m_iPlayHead);
				}
				else
				{
					rosbag::MessageInstance m = *m_ViewIterator;
					m_CurrRecord = m.instantiate<T>();

					if(m_CurrRecord == NULL)
					{
						std::cout << "Record is Null !! Skip" << std::endl;
						return false;
					}
				}

				m_bReadNext = false;
			}

			ros::Time sync_time = m_CurrRecord->header.stamp;
			ros::Time prev_time = m_CurrRecord->header.stamp;

			if(pSyncTime != NULL)
				sync_time = *pSyncTime;

			if(m_PrevRecord != NULL)
				prev_time = m_PrevRecord->header.stamp;

			ros::Duration rec_time_diff = m_CurrRecord->header.stamp - prev_time;
			ros::Duration actual_time_diff = ros::Time().now() - m_Timer;
			ros::Duration sync_time_diff = m_CurrRecord->header.stamp - sync_time;

			if(actual_time_diff >= rec_time_diff && actual_time_diff >= sync_time_diff)
			{
				msg = m_CurrRecord;
				m_PrevRecord = m_CurrRecord;
				m_Timer = ros::Time().now();
				m_bReadNext = true;
				m_iFrame++;

				if(m_iPlayHead == m_PrevRecords.size())
				{
					m_PrevRecords.push_back(m_CurrRecord);
					if(m_PrevRecords.size() > MAX_RECORDS_BUFFER)
						m_PrevRecords.erase(m_PrevRecords.begin()+0);
					m_iPlayHead = m_PrevRecords.size();
					m_ViewIterator++;
				}
				else
				{
					m_iPlayHead++;
				}

				return true;
			}

			return false;

		}
		else
		{
			return false;
		}
	}

	bool ReadPrev(boost::shared_ptr<T>& msg, ros::Time* pSyncTime)
	{
		if(m_PrevRecords.size() > 0 && m_iPlayHead > 0 )
		{
			boost::shared_ptr<T> currRecord;
			m_iPlayHead--;
			currRecord = m_PrevRecords.at(m_iPlayHead);

			ros::Time sync_time  = currRecord->header.stamp;
			ros::Time prev_time  = currRecord->header.stamp;

			if(pSyncTime != NULL)
				sync_time = *pSyncTime;

			if(m_PrevRecord != NULL)
				prev_time = m_PrevRecord->header.stamp;

			ros::Duration rec_time_diff = prev_time - currRecord->header.stamp;
			ros::Duration actual_time_diff = ros::Time().now() - m_Timer;
			ros::Duration sync_time_diff = sync_time - currRecord->header.stamp;

			//if(actual_time_diff >= rec_time_diff && actual_time_diff >= sync_time_diff)
			if(actual_time_diff >= sync_time_diff)
			{
				m_iFrame--;
				m_Timer = ros::Time().now();
				msg = currRecord;
				m_PrevRecord = currRecord;
				m_bReadNext = true;
				return true;
			}
		}

		return false;
	}

	void GetReadingInfo(int& _t_sec, int& _t_nsec, int& iFrame, int& nTotalFrames)
	{
		if(m_CurrRecord != NULL)
		{
			ros::Duration dur = (m_CurrRecord->header.stamp - m_StartTime);
			_t_sec = dur.sec;
			_t_nsec = dur.nsec;
		}

		iFrame = m_iFrame;
		nTotalFrames = m_BagView.size();
	}


	BagTopicPlayer()
	{
		m_bReadNext = true;
		m_iFrame = 0;
		m_iPlayHead = 0;
	}

	virtual ~BagTopicPlayer()
	{

	}


private:
	rosbag::View::iterator m_ViewIterator;
	rosbag::View m_BagView;
	std::vector<boost::shared_ptr<T> > m_PrevRecords;
	boost::shared_ptr<T> m_CurrRecord;
	boost::shared_ptr<T> m_PrevRecord;
	bool m_bReadNext;
	ros::Time m_Timer;
	int m_iFrame;
	ros::Time m_StartTime;
	int m_iPlayHead;

};

} /* namespace UtilityHNS */

#endif /* BAGTOPICPLAYER_H_ */
