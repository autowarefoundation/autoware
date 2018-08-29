/*
// *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
			ROS_WARN_STREAM("Can't Find LIDAR Topic in RosBag File :" << topic_name);
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
