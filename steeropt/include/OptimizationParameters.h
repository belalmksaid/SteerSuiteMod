/*
* OptimizationParameters.h
*
*  Created on: 2015-12-08
*      Author: gberseth
*/

#ifndef OPTIMIZATIONPARAMETERS_H_
#define OPTIMIZATIONPARAMETERS_H_

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include "SteerOptPlugin.h"
#include "tinyxml/ticpp.h"
#include "util/Geometry.h"

namespace Graphing {

	struct STEEROPTPLUG_API OptimizationParameter
	{
		enum ParameterType {
			Translation,
			Rotation,
			WorldTranslation,
			NumParmeterTypes
		};

		OptimizationParameter() {}
		OptimizationParameter(double x0, double lb, double ub, std::vector<size_t> node_id, ParameterType p_type)
		{
			_x0 = x0;
			_lb = lb;
			_ub = ub;
			_node_ids = node_id;
			_p_type = p_type;
			// _translationM = Eigen::Matrix4d::Identity();
			_rotationnM_Y = Eigen::Matrix4d::Identity();
			_name = "";
		}

		double _x0;
		double _lb, _ub;
		/// One parameter can map to many nodes, edges
		std::vector<size_t> _node_ids;
		ParameterType _p_type;
		Eigen::Vector4d _translationV;
		Eigen::Matrix4d _rotationnM_Y;
		Eigen::Vector4d _rotationOriginV;
		std::string _name;

		virtual std::vector<double> getTranslationDirectionV()
		{
			std::vector<double> tVector;
			tVector.push_back(_translationV(0));
			tVector.push_back(_translationV(1));
			tVector.push_back(_translationV(2));

			return tVector;
		}

		virtual std::vector<double> getRotationOriginV()
		{
			std::vector<double> tOrigin;
			tOrigin.push_back(_rotationOriginV(0));
			tOrigin.push_back(_rotationOriginV(1));
			tOrigin.push_back(_rotationOriginV(2));

			return tOrigin;
		}

		virtual void setNodeIds(std::vector<size_t> in)
		{
			this->_node_ids.clear();
			this->_node_ids.insert(this->_node_ids.end(), in.begin(), in.end());
		}

		virtual std::vector<size_t> getNodeIDs()
		{
			return this->_node_ids;
		}

		// relative translation to be applied to the object
		// transDir is a unit vector in the direction of travel
		virtual void setTranslation(Eigen::Vector3d transDir)
		{
			_translationV = Eigen::Vector4d(transDir(0), transDir(1), transDir(2), 1);
		}
		virtual void setTranslation(std::vector<double> transDir)
		{
			this->setTranslation(Eigen::Vector3d(transDir[0], transDir[1], transDir[2]));
		}
		// theta should be in radians
		virtual void setRotationY(double theta)
		{
			_rotationnM_Y = Eigen::Matrix4d::Identity();
			_rotationnM_Y(0, 0) = cos(theta);
			_rotationnM_Y(0, 2) = sin(theta);
			_rotationnM_Y(2, 0) = -sin(theta);
			_rotationnM_Y(2, 2) = cos(theta);
			//TODO finish rotation setup
		}

		virtual void setRotationVector(Eigen::Vector3d transDir)
		{
			_rotationOriginV = Eigen::Vector4d(transDir(0), transDir(1), transDir(2), 1);
		}
		virtual void setRotationVector(std::vector<double> transDir)
		{
			this->setRotationVector(Eigen::Vector3d(transDir[0], transDir[1], transDir[2]));
		}

		std::vector<double> getUpdatedPos2(double parameter, std::vector<double> p)
		{
			Eigen::Vector3d pos = Eigen::Vector3d(p[0], p[1], p[2]);
			Eigen::Vector3d pos_ = this->getUpdatedPos(parameter, pos);
			std::vector<double> out;
			out.push_back(pos_(0));
			out.push_back(pos_(1));
			out.push_back(pos_(2));
			return out;
		}
		Eigen::Vector3d getUpdatedPos(double parameter, Eigen::Vector3d p)
		{
			// Applies the transformations to the given p and returns the result.
			// p;
			if (_p_type == ParameterType::Translation)
			{
				
				return getUpdatedPosTranslation(parameter, p);
			}
			else
			{ // perform rotation
				return getUpdatedPosRotation(parameter, p);
			}
		}

		Eigen::Vector3d getUpdatedPosTranslation(double parameter, Eigen::Vector3d p)
		{
			Eigen::Vector4d out = Eigen::Vector4d(0, 0, 0, 1.0);
			Eigen::Vector4d p_ = Eigen::Vector4d(p(0), p(1), p(2), 1.0);
			Eigen::Matrix4d t_translationM = Eigen::Matrix4d::Identity();
			t_translationM(0, 3) = _translationV(0)*parameter;
			t_translationM(1, 3) = _translationV(1)*parameter;
			t_translationM(2, 3) = _translationV(2)*parameter;
			out = (t_translationM * out) + p_;
			return Eigen::Vector3d(out(0), out(1), out(2));
		}

		Eigen::Vector3d getUpdatedPosRotation(double parameter, Eigen::Vector3d p)
		{
			Eigen::Vector4d out = Eigen::Vector4d(0, 0, 0, 1.0);
			Eigen::Vector4d p_ = Eigen::Vector4d(p(0),p(1),p(2),1.0);
			p_(3) = 1.0;
			Eigen::Matrix4d _roationM = Eigen::Matrix4d::Identity();
			_roationM(0, 0) = cos(parameter);
			_roationM(0, 2) = sin(parameter);
			_roationM(2, 0) = -sin(parameter);
			_roationM(2, 2) = cos(parameter);
			out = Eigen::Vector4d(_roationM * (p_ - _rotationOriginV)) + _rotationOriginV;
			// out = out - _rotationOriginV;
			return Eigen::Vector3d(out(0), out(1), out(2));
		}



		inline std::string toString() {
			std::stringstream ss;
			ss << "x0: " << _x0 << std::endl;
			ss << "lb: " << _lb << std::endl;
			ss << "ub: " << _ub << std::endl;
			ss << "name: " << _name << std::endl;
			ss << "tanslationVector: " << _translationV.transpose() << std::endl;
			ss << "rotationOriginc: " << _rotationOriginV.transpose() << std::endl;

			return ss.str();
		}


	};

/*<<<<<<< Updated upstream
=======
	/// Will be used to apply a parameter to the Graph
	virtual void applyParameter() {}
	virtual void addParameter(const OptimizationParameter &p)
	{
		this->_parameters.push_back(p);
	}
>>>>>>> Stashed changes*/


class STEEROPTPLUG_API OptimizationParameters {
	public:
		OptimizationParameters();
		virtual ~OptimizationParameters();

		virtual size_t size();

		/// Will be used to apply a parameter to the Graph
		virtual void applyParameter() {}
		virtual void addParameter(OptimizationParameter & p)
		{
			this->_parameters.push_back(p);
		}

		virtual OptimizationParameter getParameter(size_t p)
		{
			return this->_parameters.at(p);
		}
		virtual void loadFromFile(std::string file);
		virtual void saveToFile(std::string filename);
		inline std::string toString()
		{
			std::stringstream ss;
			for (size_t i = 0; i < this->_parameters.size(); i++)
			{
				ss << this->_parameters.at(i).toString() << std::endl;
			}
			return ss.str();
		}

		std::vector<OptimizationParameter> _parameters;
		std::vector<std::vector<Util::Point> > _visibilitiRegions;
		std::vector<std::vector<Util::Point> > _queryRegions;
		std::vector<std::vector<Util::Point> > _refRegions;

	private:
		/// @name Helper functions for parsing
		//@{
		/// Parses the top-level SteerSuite test case element.
		virtual void _parseDOM(const ticpp::Element * root);
		/// Parses the steering algorithm configuration element.
		virtual void _parseAIConfig(const ticpp::Element * subRoot);
		virtual void _parseAIParam(const ticpp::Element * subRoot, OptimizationParameter & param);
		/// Parses the environment configuration element.
		virtual void _parseEnvConfig(const ticpp::Element * root);
		virtual void _parseEnvParam(const ticpp::Element * root, OptimizationParameter & param);
		virtual void _parseNodeIDs(const ticpp::Element * root, std::vector<size_t> & node_ids);
		/// Parses the steering algorithm configuration element.
		virtual void _parseRegion(const ticpp::Element * subRoot, std::vector<Util::Point> & points);
		virtual void _parseRegions(const ticpp::Element * subRoot, std::vector<std::vector<Util::Point> > & regions);
		/// Parses the environment configuration element.
		virtual void _parseReferenceRegions(const ticpp::Element * root);
		virtual void _parseQueryRegions(const ticpp::Element * root);
		virtual void _parseVisibilityGraph(const ticpp::Element * root);
		/// Reads a 3 element vector from a SteerSuite test case, or indicates that it should be randomly generated.
		virtual void _getXYZFromXMLElement(const ticpp::Element * subRoot, Util::Point & xyzTuple);

		/// For constructing XML data
		virtual TiXmlElement * _composeRegions(std::vector<std::vector<Util::Point> > & regions, std::string name);
		virtual TiXmlElement * _composeParameter(OptimizationParameter & param);
	};

} /* namespace Graphing */
#endif /* OPTIMIZATIONPARAMETERS_H_ */
