#ifndef FCCL_FEATURES_H
#define FCCL_FEATURES_H

#include <kdl/frames.hpp>
#include <string>

// In theory, having strings inside features, screws its realtime safety for creation! 
// (there is _no_ copy-on-write implementation for the GNU std::string) For the time being, 
// we just reserve 256 bytes for every string. For longer names, realtime may get a hickup.
#define STRING_SIZE 256

// BASE CLASS FOR ALL FEATURES
class Feature
{
public:
  Feature()
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }
 
  Feature(std::string name, KDL::Vector position, std::string frame_id) : 
      name_(name), position_(position), frame_id_(frame_id)
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  virtual ~Feature();

  // name/id of the feature given by knowledge base
  std::string name_;

  // origin/position of the feature w.r.t. to frame_id_
  KDL::Vector position_;

  // tf-frame w.r.t. which the feature is defined
  std::string frame_id_;
};

class PointFeature: public Feature
{
public:
  PointFeature()
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  PointFeature(std::string name, KDL::Vector position, std::string frame_id) : 
      Feature(name, position, frame_id)
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  virtual ~PointFeature();
};

class LineFeature: public Feature
{
public:
  LineFeature()
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  LineFeature(std::string name, KDL::Vector position, KDL::Vector direction, std::string frame_id) : 
      Feature(name, position, frame_id), direction_(direction)
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  virtual ~LineFeature();

  // direction-vector of the line feature
  KDL::Vector direction_;
};

class PlaneFeature: public Feature
{
public:
  PlaneFeature()
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  PlaneFeature(std::string name, KDL::Vector position, KDL::Vector normal, std::string frame_id) : 
      Feature(name, position, frame_id), normal_(normal)
  {
    name_.reserve(STRING_SIZE);
    frame_id_.reserve(STRING_SIZE);
  }

  virtual ~PlaneFeature();

  // normal-vector of the plane feature
  KDL::Vector normal_;
};

#endif // FCCL_FEATURES_H
