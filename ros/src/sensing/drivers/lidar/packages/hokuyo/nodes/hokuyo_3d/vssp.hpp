/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/serialization/access.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/shared_array.hpp>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <vector>

#include "vsspdefs.hpp"

class vssp::vsspDriver
{
private:
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket socket;
	boost::asio::deadline_timer timer;
	bool closed;
	aux_factor_array aux_factor;

	boost::function<void(const vssp::header&,
                         const vssp::range_header&,
                         const vssp::range_index&,
                         const boost::shared_array<vssp::xyzi>&,
                         const std::chrono::microseconds &delayRead,
                         const vssp::data_range_size&)> cbPoint;
	boost::function<void(const vssp::header&,
                         const vssp::aux_header&,
                         const boost::shared_array<vssp::aux>&,
                         const std::chrono::microseconds &delayRead)> cbAux;
	boost::function<void(const vssp::header&,
                         const std::chrono::microseconds&)> cbPing;
	boost::function<void(bool)> cbConnect;
	boost::shared_array<double> tblH;
	boost::shared_array<table_sincos> tblV;
	bool tblH_loaded;
	bool tblV_loaded;
	double timeout;

	std::chrono::time_point<std::chrono::system_clock> timeReadLast;
	boost::asio::streambuf buf;

public:
	vsspDriver() :
		socket(io_service),
		timer(io_service),
		closed(false),
		aux_factor(aux_factor_default),
		cbPoint(0),
		cbAux(0),
		cbPing(0),
		tblH_loaded(false),
		tblV_loaded(false),
		timeout(1.0)
        {
        };
	void setTimeout(double to)
        {
            timeout = to;
        };
	void connect(const char *ip, unsigned int port, decltype(cbConnect) cb)
        {
            cbConnect = cb;
            boost::asio::ip::tcp::endpoint endpoint(
				boost::asio::ip::address::from_string(ip), port);
            timer.expires_from_now(boost::posix_time::seconds(timeout));
            timer.async_wait(boost::bind(&vsspDriver::on_timeout_connect, this,
                                         boost::asio::placeholders::error));
            socket.async_connect(endpoint,
                                 boost::bind(&vssp::vsspDriver::on_connect,
                                             this,
                                             boost::asio::placeholders::error));
            timeReadLast = std::chrono::system_clock::now();
        };
	void registerCallback(decltype(cbPoint) cb)
        {
            cbPoint = cb;
        };
	void registerAuxCallback(decltype(cbAux) cb)
        {
            cbAux = cb;
        };
	void registerPingCallback(decltype(cbPing) cb)
        {
            cbPing = cb;
        };
	void setInterlace(int itl)
        {
            send((boost::format("SET:_itl=0,%02d\n") % itl).str());
        };
	void requestVerticalTable()
        {
            send(std::string("GET:tblv\n"));
        };
	void requestHorizontalTable()
        {
            send(std::string("GET:tblh\n"));
        };
	void requestPing()
        {
            send(std::string("PNG\n"));
        };
	void requestAuxData(bool start = 1)
        {
            send((boost::format("DAT:ax=%d\n") % (int)start).str());
        }
	void requestData(bool intensity = 1, bool start = 1)
        {
            if(intensity)
            {
                send((boost::format("DAT:ri=%d\n") % (int)start).str());
            }
            else
            {
                send((boost::format("DAT:ro=%d\n") % (int)start).str());
            }
        };
	void receivePackets()
        {
            timer.cancel();
            timer.expires_from_now(boost::posix_time::seconds(timeout));
            timer.async_wait(boost::bind(&vsspDriver::on_timeout, this,
                                         boost::asio::placeholders::error));
            boost::asio::async_read(
				socket,
				buf,
				boost::asio::transfer_at_least(65536),
				boost::bind(
					&vsspDriver::on_read,
					this,
					boost::asio::placeholders::error)
				);
        };
	bool poll()
        {
            if(!closed)
            {
                boost::system::error_code ec;
                io_service.poll(ec);
                if(!ec) return true;
                closed = true;
            }
            return false;
        };

private:
	void send(std::string cmd)
        {
            boost::shared_ptr<std::string> data(new std::string(cmd));
            boost::asio::async_write(
				socket,
				boost::asio::buffer(*data),
				boost::bind(
					&vsspDriver::on_send,
					this,
					boost::asio::placeholders::error,
					data)
				);
        };
	void on_timeout_connect(const boost::system::error_code& error)
        {
            if(!error)
            {
                closed = true;
                socket.cancel();
            }
        }
	void on_timeout(const boost::system::error_code& error)
        {
            if(!error)
            {
                closed = true;
                socket.cancel();
            }
        }
	void on_connect(const boost::system::error_code& error)
        {
            timer.cancel();
            if(error)
            {
                closed = true;
                cbConnect(false);
                return;
            }
            cbConnect(true);
        };
	void on_send(const boost::system::error_code& error,
                 boost::shared_ptr<std::string> data)
        {
            if(error)
            {
                closed = true;
                return;
            }
        };
	template<class DATA_TYPE>
    void rangeToXYZ(const vssp::range_header &range_header,
                    const vssp::range_index &range_index,
                    boost::shared_array<uint16_t> &index,
                    boost::shared_array<vssp::xyzi> &points)
		{
			int i = 0;

			double h_head = range_header.line_head_h_angle_ratio * 2.0 * M_PI / 65535.0;
			double h_tail = range_header.line_tail_h_angle_ratio * 2.0 * M_PI / 65535.0;
			const DATA_TYPE *data =
				boost::asio::buffer_cast
				<const DATA_TYPE*>(buf.data());
			for(int s = 0; s < range_index.nspots; s ++)
			{
				double spot = s + range_header.spot;
				double h_rad = h_head + (h_tail - h_head) * tblH[spot];
				double h_cos = cos(h_rad);
				double h_sin = sin(h_rad);
				vssp::xyzi dir(tblV[spot].s, tblV[spot].c, h_sin, h_cos);
				for(int e = index[s]; e < index[s+1]; e++)
					points[i++] = dir * data[e];
			}
		};
	void on_read(const boost::system::error_code& error)
        {
            auto timeRead = std::chrono::system_clock::now();
            auto time = timeReadLast;
            auto duration = timeRead - timeReadLast;
            auto lengthTotal = buf.size();
            if(error == boost::asio::error::eof)
            {
                // Connection closed
                closed = true;
                return;
            }
            else if(error)
            {
                // Connection error
                closed = true;
                return;
            }
            while(true)
            {
                if(buf.size() < sizeof(vssp::header))
                {
                    break;
                }
                // Read packet header
                const vssp::header header =
                    *boost::asio::buffer_cast<const vssp::header*>(buf.data());
                if(header.mark != vssp::VSSP_MARK)
                {
                    // Invalid packet
                    // find VSSP mark
                    const uint8_t *data =
                        boost::asio::buffer_cast<const uint8_t*>(buf.data());
                    for(size_t i = 1; i < buf.size() - sizeof(uint32_t); i ++)
                    {
                        const uint32_t *mark =
                            reinterpret_cast<const uint32_t*>(data + i);
                        if(*mark == vssp::VSSP_MARK)
                        {
                            buf.consume(i);
                            break;
                        }
                    }
                    break;
                }
                if(buf.size() < header.length) break;
                auto delay = std::chrono::duration_cast
                    <std::chrono::microseconds>
                    (std::chrono::system_clock::now() - time);
                time += duration * header.length / lengthTotal;

                size_t length = header.length - header.header_length;
                buf.consume(header.header_length);

                do
                {
                    if(header.status != vssp::STATUS_OK) break;

                    switch(header.type)
                    {
                    case TYPE_GET:
                        // Response to get command
					{
						const std::string data(boost::asio::buffer_cast<const char*>(buf.data()));
						std::vector<std::string> lines;
						boost::algorithm::split(lines, data, boost::algorithm::is_any_of("\n\r"));
						if(lines.size() == 0) break;

						if(lines[0].compare(0, 7, "GET:tbl") == 0)
						{
							if(lines.size() < 2) break;
							std::vector<std::string> cells;
							boost::algorithm::split(cells, lines[1], boost::algorithm::is_any_of(","));
							int i = 0;

							if(lines[0].compare("GET:tblv") == 0)
							{
								tblV.reset(new table_sincos[cells.size()]);
								for(auto &cell: cells)
								{
									double rad = (double)std::strtol(cell.c_str(), NULL, 16) * 2.0 * M_PI / 65535;
									sincos(rad, &tblV[i].s, &tblV[i].c);
									i ++;
								}
								tblV_loaded = true;
							}
							else if(lines[0].compare("GET:tblh") == 0)
							{
								tblH.reset(new double[cells.size()]);
								for(auto &cell: cells)
								{
									tblH[i] = (double)std::strtol(cell.c_str(), NULL, 16) / 65535;
									i ++;
								}
								tblH_loaded = true;
							}
						}
					}
					break;
                    case TYPE_SET:
                        // Response to set command
                        break;
                    case TYPE_DAT:
                        // Response to data request command
                        break;
                    case TYPE_VER:
                        // Response to version request command
                        break;
                    case TYPE_PNG:
                        // Response to ping command
                        if(!cbPing.empty())
                            cbPing(header, delay);
                        break;
                    case TYPE_RI:
                    case TYPE_RO:
                        // Range data
                        if(!tblH_loaded || !tblV_loaded || cbPoint.empty())
                        {
                            // Something wrong
                            break;
                        }
                        {
                            // Decode range data header
                            const vssp::range_header range_header =
                                *boost::asio::buffer_cast<const vssp::range_header*>(buf.data());
                            buf.consume(range_header.header_length);
                            length -= range_header.header_length;

                            // Decode range index header
                            const vssp::range_index range_index =
                                *boost::asio::buffer_cast<const vssp::range_index*>(buf.data());
                            size_t index_length = range_index.index_length;
                            buf.consume(sizeof(vssp::range_index));
                            index_length -= sizeof(vssp::range_index);
                            length -= sizeof(vssp::range_index);

                            // Decode range index
                            boost::shared_array<uint16_t> index(new uint16_t[range_index.nspots+1]);
                            std::memcpy(index.get(),
                                        boost::asio::buffer_cast<const vssp::range_index*>(buf.data()),
                                        sizeof(uint16_t) * (range_index.nspots+1));
                            buf.consume(index_length);
                            length -= index_length;

                            // Decode echo size
                            const vssp::data_range_size data_range_size = {index[range_index.nspots]};

                            // Decode range data
                            boost::shared_array<vssp::xyzi> points(new vssp::xyzi[index[range_index.nspots]]);
                            switch(header.type)
                            {
                            case TYPE_RI:
                                // Range and Intensity
                                rangeToXYZ<vssp::data_range_intensity>
                                    (range_header, range_index, index, points);
                                break;
                            case TYPE_RO:
                                // Range
                                rangeToXYZ<vssp::data_range_only>
                                    (range_header, range_index, index, points);
                                break;
                            }
                            cbPoint(header, range_header, range_index, points, delay, data_range_size);
                        }
                        break;
                    case TYPE_AX:
                        // Aux data
					{
						// Decode range data header
						const vssp::aux_header aux_header =
							*boost::asio::buffer_cast<const vssp::aux_header*>(buf.data());
						buf.consume(aux_header.header_length);
						length -= aux_header.header_length;

						// Decode aux data
						boost::shared_array<vssp::aux> auxs(new vssp::aux[aux_header.data_count]);
						for(int i = 0; i < aux_header.data_count; i ++)
						{
							const vssp::aux_data *data =
								boost::asio::buffer_cast
								<const vssp::aux_data*>(buf.data());
							int offset = 0;
							for(aux_id b = vssp::AX_MASK_LAST;
                                b >= vssp::AX_MASK_FIRST;
                                b = static_cast<aux_id>(b - 1))
							{
								if(aux_header.data_bitfield & (1 << static_cast<int>(b)))
									auxs[i][b] = aux_factor[b] * data[offset ++].val;
							}
							buf.consume(sizeof(int32_t) * offset);
							length -= sizeof(int32_t) * offset;
						}
						if(!cbAux.empty()) cbAux(header, aux_header, auxs, delay);
					}
					break;
                    }
                }
                while(false);
                buf.consume(length);
            }
            receivePackets();
            timeReadLast = timeRead;
            return;
        };
};
