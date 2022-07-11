// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * Author MapIV  Takanose
 */

#include <iostream>
#include <fstream>
#include <iomanip>

#ifndef KMLGENERATOR_H
#define KMLGENERATOR_H

class KmlGenerator
{
public:
  KmlGenerator(const std::string,const std::string);
  KmlGenerator(const std::string,const std::string,const std::string);


  void addPoint(double, double, double);
  void KmlGenerate(const std::string);
  std::string getKmlBody();


private:
  std::ofstream kmlfile_;
  std::stringstream header_;
  std::stringstream config_header_;
  std::stringstream body_;
  std::stringstream config_footer_;
  std::stringstream footer_;
};

KmlGenerator::KmlGenerator(const std::string kmlname,const std::string color="ff0000ff")
{
  header_ 
    << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<< "\n"
    << "<kml xmlns=\"http://earth.google.com/kml/2.2\">"<< "\n"
    << "<Document>"<< "\n"
    << "<name>"<< kmlname <<"</name>"<< "\n"
    << "\n";

  config_header_
    << "\t<Placemark>"<< "\n"
    << "\t\t<name>"<< kmlname <<"</name>"<< "\n"
    << "\t\t<Style>"<< "\n"
    << "\t\t\t<LineStyle>"<< "\n"
    << "\t\t\t\t<color>"<<color<<"</color>"<< "\n"
    << "\t\t\t\t<width>5.00</width>"<< "\n"
    << "\t\t\t</LineStyle>"<< "\n"
    << "\t\t</Style>"	<< "\n"
    << "\t\t<LineString>"<< "\n"
    << "\t\t\t<tessellate>1</tessellate>"<< "\n"
    << "\t\t\t<coordinates>"<< "\n";

  config_footer_
    << "\t\t\t</coordinates>"<< "\n"
    << "\t\t</LineString>"<< "\n"
    << "\t</Placemark>"<< "\n"
    << "\n";

  footer_ 
    << "</Document>"<< "\n"
    << "</kml>"<< "\n";
}

KmlGenerator::KmlGenerator(const std::string kmlname,const std::string filename,const std::string body)
{
  kmlfile_.open(filename, std::ios::out);
  if(!kmlfile_)	std::cerr << "KmlGenerator::KmlGenerate\tCould not open file file! " << filename << std::endl;
  std::cout << "KmlGenerator::KmlGenerate\topen file file! " << filename << std::endl;

  header_ 
    << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<< "\n"
    << "<kml xmlns=\"http://earth.google.com/kml/2.2\">"<< "\n"
    << "<Document>"<< "\n"
    << "<name>"<< kmlname <<"</name>"<< "\n"
    << "\n";

  footer_ 
    << "</Document>"<< "\n"
    << "</kml>"<< "\n";

  std::string s_header = header_.str();
  std::string s_footer = footer_.str();

  kmlfile_ << s_header << body << s_footer;
  kmlfile_.close();

}

void KmlGenerator::addPoint(double longitude, double latitude, double altitude)
{
  body_ << std::setprecision(13) << longitude <<","
        << std::setprecision(13) << latitude  <<","
        << std::setprecision(13) << altitude  <<"\n";
}

void KmlGenerator::KmlGenerate(const std::string filename)
{
  kmlfile_.open(filename, std::ios::out);
  if(!kmlfile_)	std::cerr << "KmlGenerator::KmlGenerate\tCould not open file file! " << filename << std::endl;

  std::string s_header        = header_.str();
  std::string s_config_header = config_header_.str();
  std::string s_body          = body_.str();
  std::string s_config_footer = config_footer_.str();
  std::string s_footer        = footer_.str();

  kmlfile_ << s_header << s_config_header << s_body << s_config_footer << s_footer;
  kmlfile_.close();
}

std::string KmlGenerator::getKmlBody()
{
  std::string s_config_header = config_header_.str();
  std::string s_body          = body_.str();
  std::string s_config_footer = config_footer_.str();

  return s_config_header + s_body + s_config_footer;
}


#endif /*KMLGENERATOR_H */
