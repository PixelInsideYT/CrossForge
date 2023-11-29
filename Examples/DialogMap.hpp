#ifndef __CFORGE_DIALOGMAP_HPP__
#define __CFORGE_DIALOGMAP_HPP__

#include <map>

using namespace std;

namespace CForge {

	class DialogMap {
	public:
		DialogMap(void) {

			dialogMap["plant"] = &DialogMap::getPlantName;

		}//Constructor

		~DialogMap(void) {

			dialogMap.clear();

		}//Destructor

		string lookup(string key) {

			return (this->*dialogMap[key])();
		}

		string getPlantName() {
			return "monstera";
		}

		map<string, string(DialogMap::*)()> dialogMap;

	protected:

	};//DialogMap

}//name space

#endif
