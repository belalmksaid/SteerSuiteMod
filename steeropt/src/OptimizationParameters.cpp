/*
* OptimizationParameters.cpp
*
*  Created on: 2015-12-08
*      Author: gberseth
*/

#include "OptimizationParameters.h"
#include "tinyxml/ticpp.h"
#include "util/GenericException.h"
#include "testcaseio/TestCaseIO.h"
#include "util/Misc.h"

namespace Graphing {

	OptimizationParameters::OptimizationParameters() {
		// TODO Auto-generated constructor stub

	}

	OptimizationParameters::~OptimizationParameters() {
		// TODO Auto-generated destructor stub
	}

	size_t OptimizationParameters::size()
	{
		return this->_parameters.size();
	}

	void OptimizationParameters::loadFromFile(std::string fileName)
	{

		//
		// first, parse the test case and get the raw data from it
		//
		ticpp::Document doc(fileName);
		doc.LoadFile();
		ticpp::Element * root = doc.FirstChildElement();
		std::string rootTagName = root->Value();

		// if the root tag doesn't match our expected root tag, its an error.
		if (rootTagName != "parameterOptimitationExperiment" &&
			(rootTagName != "AIparameterOptimitationExperiment"))
		{
			throw Util::GenericException("XML file " + fileName + " does not seem to be a Valid Optimization Experiment test case.\n");
		}

		_parseDOM(root);
	}

	TiXmlElement * _composeXYZ(Util::Point p, std::string name)
	{
		std::stringstream ss;
		TiXmlElement * point = new TiXmlElement(name);
		TiXmlElement * x = new TiXmlElement("x");
		ss << p.x;
		TiXmlText * x_ = new TiXmlText(ss.str());
		ss.str(std::string());
		TiXmlElement * y = new TiXmlElement("y");
		ss << p.y;
		TiXmlText * y_ = new TiXmlText(ss.str());
		ss.str(std::string());
		TiXmlElement * z = new TiXmlElement("z");
		ss << p.z;
		TiXmlText * z_ = new TiXmlText(ss.str());
		ss.str(std::string());
		x->LinkEndChild(x_);
		y->LinkEndChild(y_);
		z->LinkEndChild(z_);
		point->LinkEndChild(x);
		point->LinkEndChild(y);
		point->LinkEndChild(z);
		return point;
	}

	TiXmlElement * OptimizationParameters::_composeRegions(std::vector<std::vector<Util::Point> > & regions, std::string name)
	{
		std::stringstream ss;
		TiXmlElement * visibilityGraph = new TiXmlElement(name);
		for (size_t i = 0; i < regions.size(); i++)
		{
			TiXmlElement * region = new TiXmlElement("region");
			for (size_t j = 0; j < regions[i].size(); j++)
			{
				TiXmlElement * point = _composeXYZ(regions[i][j], "point");
				
				region->LinkEndChild(point);
			}
			visibilityGraph->LinkEndChild(region);
		}
		return visibilityGraph;
	}

	TiXmlElement * OptimizationParameters::_composeParameter(OptimizationParameter & param)
	{
		std::stringstream ss;
		TiXmlElement * parameter = new TiXmlElement("parameter");

		TiXmlElement * min = new TiXmlElement("min");
		TiXmlElement * max = new TiXmlElement("max");
		TiXmlElement * original = new TiXmlElement("original");
		// TiXmlElement * discretization = new TiXmlElement("discretization"); // Not Used
		TiXmlElement * translationDirection;
		if (param._p_type == OptimizationParameter::ParameterType::Translation)
		{
			translationDirection = _composeXYZ(Util::Point(param._translationV(0), param._translationV(1), 
				param._translationV(2)), "translationDirection");
		}
		else
		{
			translationDirection = _composeXYZ(Util::Point(param._rotationOriginV(0), param._rotationOriginV(1),
				param._rotationOriginV(2)), "rotationOrigin");
		}

		TiXmlElement * node_ids = new TiXmlElement("node_ids");

		ss << param._lb;
		TiXmlText * lb = new TiXmlText(ss.str());
		min->LinkEndChild(lb);
		ss.str(std::string());

		ss << param._ub;
		TiXmlText * ub = new TiXmlText(ss.str());
		max->LinkEndChild(ub);
		ss.str(std::string());

		ss << param._x0;
		TiXmlText * x0 = new TiXmlText(ss.str());
		original->LinkEndChild(x0);
		ss.str(std::string());

		parameter->LinkEndChild(min);
		parameter->LinkEndChild(max);
		parameter->LinkEndChild(original);
		parameter->LinkEndChild(translationDirection);

		for (size_t i = 0; i < param.getNodeIDs().size(); i++)
		{
			TiXmlElement * node = new TiXmlElement("node");
			ss << param.getNodeIDs()[i];
			TiXmlText * node_ = new TiXmlText(ss.str());
			node->LinkEndChild(node_);
			ss.str(std::string());
			node_ids->LinkEndChild(node);
		}

		parameter->LinkEndChild(node_ids);
		return parameter;
	}

	void OptimizationParameters::saveToFile(std::string filename)
	{
		// Make xml: <?xml ..><Hello>World</Hello>
		std::stringstream ss;
		TiXmlDocument doc;
		TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
		TiXmlElement * element = new TiXmlElement("parameterOptimitationExperiment");
		TiXmlElement * env_config = new TiXmlElement("env_config");

		TiXmlElement * visibilityGraph = _composeRegions(this->_visibilitiRegions, "visibilityGraph");
		env_config->LinkEndChild(visibilityGraph);

		TiXmlElement * queryRegions = _composeRegions(this->_queryRegions, "queryRegions");
		env_config->LinkEndChild(queryRegions);

		TiXmlElement * refRegions = _composeRegions(this->_refRegions, "referenceRegions");
		env_config->LinkEndChild(refRegions);

		for (size_t i = 0; i < this->_parameters.size(); i++)
		{
			TiXmlElement * param = _composeParameter(this->_parameters[i]);
			env_config->LinkEndChild(param);
		}


		element->LinkEndChild(env_config);
		doc.LinkEndChild(decl);
		doc.LinkEndChild(element);
		doc.SaveFile(filename);
	}

	void OptimizationParameters::_parseDOM(const ticpp::Element * root)
	{
		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(root); child != child.end(); child++) {

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "ai_config") {
				_parseEnvConfig(&(*child));
			}
			else if (childTagName == "env_config") {
				_parseEnvConfig(&(*child));
			}
			else {
				throw GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}
	}

	void OptimizationParameters::_parseAIConfig(const ticpp::Element * subRoot)
	{
		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(subRoot); child != child.end(); child++) {

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "parameter") {
				OptimizationParameter param;
				_parseAIParam(&(*child), param);
				this->addParameter(param);
			}
			else {
				throw Util::GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}
	}

	void OptimizationParameters::_parseAIParam(const ticpp::Element * subRoot, OptimizationParameter & param)
	{


	}
	void OptimizationParameters::_parseEnvConfig(const ticpp::Element * root)
	{
		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(root); child != child.end(); child++) {

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "parameter") {
				OptimizationParameter param;
				_parseEnvParam(&(*child), param);
				this->addParameter(param);
			}
			else if (childTagName == "visibilityGraph") {
				// this->_degree_weight = atof((*optionIter).second.c_str());
				_parseVisibilityGraph(&(*child));
			}
			else if (childTagName == "queryRegions") {
				// this->_degree_weight = atof((*optionIter).second.c_str());
				_parseQueryRegions(&(*child));
			}
			else if (childTagName == "referenceRegions") {
				// std::cout << (*optionIter).first << ": " << (*optionIter).second << std::endl;
				// this->_degree_weight = atof((*optionIter).second.c_str());
				_parseReferenceRegions(&(*child));
			}

			else {
				throw Util::GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}
	}

	void OptimizationParameters::_parseEnvParam(const ticpp::Element * root, OptimizationParameter & param)
	{
		std::string name;
		root->GetAttribute("name", &name, false);
		param._name = name;

		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(root); child != child.end(); child++) {

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "min") {
				child->GetText(&param._lb);
			}
			else if (childTagName == "max") {
				child->GetText(&param._ub);
			}
			else if (childTagName == "original") {
				child->GetText(&param._x0);
			}
			else if (childTagName == "discretization") {
				// TODO skip for now.
			}
			else if (childTagName == "translationDirection") {
				Util::Point p;
				_getXYZFromXMLElement(&(*child), p);
				param.setTranslation(Eigen::Vector3d(p.x, p.y, p.z));
				param._p_type = OptimizationParameter::ParameterType::Translation;
			}
			else if (childTagName == "rotationOrigin") {
				Util::Point p;
				_getXYZFromXMLElement(&(*child), p);
				param.setRotationVector(Eigen::Vector3d(p.x, p.y, p.z));
				param._p_type = OptimizationParameter::ParameterType::Rotation;
			}
			else if (childTagName == "node_ids") {
				std::vector<size_t> node_ids;
				_parseNodeIDs(&(*child), node_ids);
				param.setNodeIds(node_ids);
			}
			else {
				throw Util::GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}

	}

	void OptimizationParameters::_getXYZFromXMLElement(const ticpp::Element * subRoot, Util::Point & xyzTuple)
	{
		bool XTagSpecified = false;
		bool YTagSpecified = false;
		bool ZTagSpecified = false;

		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(subRoot); child != child.end(); child++)
		{
			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// its the address of the iterator's current content.
			if (childTagName == "x") {
				child->GetText(&xyzTuple.x);
				XTagSpecified = true;
			}
			else if (childTagName == "y") {
				child->GetText(&xyzTuple.y);
				YTagSpecified = true;
			}
			else if (childTagName == "z") {
				child->GetText(&xyzTuple.z);
				ZTagSpecified = true;
			}
			else {
				throw GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}

		if ((XTagSpecified != YTagSpecified) || (XTagSpecified != ZTagSpecified)) {
			throw GenericException("Inside the element <" + subRoot->Value() + "> at line " + Util::toString(subRoot->Row()) + ": please specify <x>, <y> and <z> values.");
		}
	}

	void OptimizationParameters::_parseNodeIDs(const ticpp::Element * root, std::vector<size_t> & node_ids)
	{
		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(root); child != child.end(); child++)
		{

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "node") {
				size_t param;
				child->GetText(&param);
				node_ids.push_back(param);
			}
			else {
				throw Util::GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}
	}

	void OptimizationParameters::_parseRegion(const ticpp::Element * subRoot, std::vector<Util::Point> & points)
	{
		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(subRoot); child != child.end(); child++)
		{

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "point") {
				Util::Point param;
				_getXYZFromXMLElement(&(*child), param);
				points.push_back(param);
			}
			else {
				throw Util::GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}

	}

	/// Parses the environment configuration element.
	void OptimizationParameters::_parseRegions(const ticpp::Element * subRoot, std::vector<std::vector<Util::Point> > & regions)
	{
		ticpp::Iterator<ticpp::Element> child;
		for (child = child.begin(subRoot); child != child.end(); child++)
		{

			std::string childTagName = child->Value();

			// NOTE: in the following code, '&' and '*' do not cancel each other out;
			// it's the address of the iterator's current content.
			if (childTagName == "region") {
				std::vector<Util::Point> points;
				_parseRegion(&(*child), points);
				regions.push_back(points);
			}
			else {
				throw Util::GenericException("Unexpected tag <" + childTagName + "> found on line " + Util::toString(child->Row()) + "\n");
			}
		}
	}

	/// Parses the environment configuration element.
	void OptimizationParameters::_parseReferenceRegions(const ticpp::Element * root)
	{
		_parseRegions(root, _refRegions);
	}

	void OptimizationParameters::_parseQueryRegions(const ticpp::Element * root)
	{
		_parseRegions(root, _queryRegions);
	}

	void OptimizationParameters::_parseVisibilityGraph(const ticpp::Element * root)
	{
		_parseRegions(root, _visibilitiRegions);
	}


} /* namespace Graphing */
