/*
 * Copyright 2016-2019 Autoware Foundation. All rights reserved.
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

#include <pcl_conversions/pcl_conversions.h>

struct Area {
	std::string path;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
};

typedef std::vector<Area> AreaList;
typedef std::vector<std::vector<std::string>> Tbl;

void write_csv(const std::string& path, const Tbl& tbl)
{
	std::ofstream ofs(path != "-" ? path.c_str() : "/dev/null");
	std::ostream& os = (path != "-") ? ofs : (std::cout);

	for (const std::vector<std::string>& cols : tbl) {
		std::string line;
		for (size_t i = 0; i < cols.size(); ++i) {
			if (i > 0) line += ",";
			line += cols[i];
		}
		os << line << std::endl;
	}
}

std::string fmt(double v)
{
	char s[64];
	snprintf(s, sizeof(s), "%.3f", v);
	return std::string(s);
}

void write_arealist(const std::string& path, const AreaList& areas)
{
	Tbl tbl;
	for (const Area& area : areas) {
		std::vector<std::string> cols;
		cols.push_back(area.path);
		cols.push_back(fmt(area.x_min));
		cols.push_back(fmt(area.y_min));
		cols.push_back(fmt(area.z_min));
		cols.push_back(fmt(area.x_max));
		cols.push_back(fmt(area.y_max));
		cols.push_back(fmt(area.z_max));
		tbl.push_back(cols);
	}
	write_csv(path, tbl);
}

int calc_area(const std::string& path, struct Area *area)
{
	pcl::PointCloud<pcl::PointXYZRGB> pcd;

	if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
		std::cerr << "load failed " << path << std::endl;
		return -1;
	}
	struct Area a;
	a.path = path;

	pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pcd.begin();
	pcl::PointCloud<pcl::PointXYZRGB>::iterator end = pcd.end();
	a.x_min = a.x_max = it->x;
	a.y_min = a.y_max = it->y;
	a.z_min = a.z_max = it->z;

	for (it++; it != end; it++) {
		double x = it->x;
		double y = it->y;
		double z = it->z;
		if (x < a.x_min) a.x_min = x;
		if (x > a.x_max) a.x_max = x;
		if (y < a.y_min) a.y_min = y;
		if (y > a.y_max) a.y_max = y;
		if (z < a.z_min) a.z_min = z;
		if (z > a.z_max) a.z_max = z;
	}
	*area = a;
	return 0;
}

void add_file(const std::string& path, AreaList& areas)
{
	struct Area area;
	if (calc_area(path, &area) == 0) {
		areas.push_back(area);
	}
}

void add_dir(const std::string& path, AreaList& areas)
{
	std::string cmd = "find " + path + " -name '*.pcd' | sort";
	FILE *fp = popen(cmd.c_str(), "r");
	char line[ PATH_MAX ];

	while (fgets(line, sizeof(line), fp)) {
		std::string buf(line);
		buf.erase(--buf.end()); // cut tail '\n'
		add_file(buf, areas);
	}
	pclose(fp);
}

int is_dir(const std::string& path)
{
	struct stat buf;
	if (stat(path.c_str(), &buf) != 0) {
		std::cerr << "not found " << path << std::endl;
		exit(1);
	}
	return S_ISDIR(buf.st_mode);
}

int main (int argc, char** argv)
{
	argc--;
	argv++;
	if (argc <= 0) {
		std::cout << "Usage: rosrun map_tools pcd_arealist [ -o OUTPUT ] INPUT" << std::endl;
		return 0;
	}
	std::string out;
	if (argc >= 2 &&  strcmp(*argv, "-o") == 0) {
		argc -= 2;
		argv++;
		out = *argv++;
	}
	AreaList areas;
	std::string d1;
	for (; argc > 0; argc--) {
		std::string in = *argv++;
		if (is_dir(in)) {
			if (d1.empty()) d1 = in;
			add_dir(in, areas);
		} else {
			add_file(in, areas);
		}		
	}
	if (out.empty()) {
		out = d1.empty() ? "-" : d1 + "/arealists.txt";
	}
	write_arealist(out, areas);
	return 0;
}
