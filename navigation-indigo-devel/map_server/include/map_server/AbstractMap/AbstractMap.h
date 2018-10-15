#ifndef _SURO_STD_ABSTRACT_MAP_taotao_20110901_H
#define _SURO_STD_ABSTRACT_MAP_taotao_20110901_H

#include <iostream>
#include <vector>
#include <boost/smart_ptr.hpp>

class AbstractMap
{
public:
  AbstractMap(){}
  virtual ~AbstractMap(){}
  virtual void LoadFromFile(std::string file_name){};
  virtual std::string GetMapTypeName() = 0;

protected:
  std::string map_name_; //地图名称
  std::string trace_name_; //地图名称

public:
  inline std::string get_map_name() {
    return map_name_;
  }
  inline std::string get_trace_name() {
      return trace_name_;
    }
};

#endif
