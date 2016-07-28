#ifndef PetriNetPlans_XMLPnpPlanInstantiator_guard
#define PetriNetPlans_XMLPnpPlanInstantiator_guard


#include <string>
#include <sstream>
#include <stdexcept>
#include <map>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include "basic_plan.h"

//I use this declaration, defined  in the .cpp,
 //to remove the includes of libxml from this file.
 struct hideParams;

namespace PetriNetPlans {

	/**
	 * \brief A xml plan loader for basic PnpPlans
	 *
	 * This class parses .pnml files to fill plan objects. Just instantiate the class
	 * and call loadFromPNML().
	 * */
  class XMLPnpPlanInstantiator {
    public:

      /**
      * \brief Loads the plan from a file
      *
      * \param filePath the path of the .pnml file to load
      * \param plan the plan object that will receive the net specified in
      * 				\p filePath
      * \exception std::runtime_error thrown whenever the parsing fails
      */
      void loadFromPNML(const std::string& filePath,  PnpPlan* plan) throw(std::runtime_error);

      /**
      * \brief Loads the plan from a string
      *
      * \param filePath the path of the .pnml file to load
      * \param plan the plan object that will receive the net specified in
      * 				\p filePath
      * \exception std::runtime_error thrown whenever the parsing fails
      */
      void loadFromString(const std::string& xml,  PnpPlan* plan) throw(std::runtime_error);

	  /**
	  *\brief dtor
	  */
      virtual ~XMLPnpPlanInstantiator();

    private:
      void loadPlan(xmlDocPtr doc, xmlNodePtr cur, PnpPlan* plan) throw(std::runtime_error);
      void parseArc(const hideParams&, PnpPlan*);
      void parseComment(const hideParams&, PnpPlan*);
      void parseTransition(const hideParams&, PnpPlan*);
      void parsePlace(const hideParams&, PnpPlan*);
      std::map<std::string, PnpPlace*> placesPnmlLookup;
      std::map<std::string, PnpTransition*> transitionsPnmlLookup;


      std::string generateUUID() {
          return num_to_str<boost::uuids::uuid>(boost::uuids::random_generator()());
      }

      template<typename T>
      std::string num_to_str(T num) {
          std::stringstream ss;
          ss << num;
          return ss.str();
      }
      
      std::string extractPlanNameFromPath(const std::string& filePath);
      hideParams checkDocument(const hideParams &, const std::string& filePath);

  };
}

#endif
