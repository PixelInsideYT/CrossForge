#ifndef __CFORGE_DIALOGGRAPH_HPP__
#define __CFORGE_DIALOGGRAPH_HPP__

#include "json/json.h"
#include <fstream>
#include <iostream>
#include "DialogMap.hpp"

using namespace std;

namespace CForge {

	class Dialoggraph {
	public:
		Dialoggraph(void) {
			
		}//Constructor

		~Dialoggraph(void) {
			
		}//Destructor

		void init(string filepath, DialogMap map) {
			// read dialog from json file
			Json::Value root;
			std::ifstream ifs;
			ifs.open(filepath, std::ifstream::in);
			if (!ifs.is_open()) cout << "File could not be opened" << endl;
			Json::Reader reader;
			reader.parse(ifs, root);
			ifs.close();

			// initialize dialog graph
			text = root["text"].asString();
			replacePlaceholder(map);
			playerSpeaking = root["playerSpeaking"].asBool();
			for (Json::Value i : root["answers"]) {
				Dialoggraph dialogInit;
				dialogInit.initAnswer(i, map);
				answers.push_back(dialogInit);				
			}		
		}

		void initAnswer(Json::Value answer, DialogMap map) {
			text = answer["text"].asString();
			replacePlaceholder(map);
			playerSpeaking = answer["playerSpeaking"].asBool();
			for (Json::Value j : answer["answers"]) {
				Dialoggraph dialogAnswer;
				dialogAnswer.initAnswer(j, map);
				answers.push_back(dialogAnswer);				
			}
		}

		void replacePlaceholder(DialogMap map) {
			int start = text.find("<");
			int end = text.find(">");
			if (start == -1) return;

			int len = end - start + 1;
			string placeholder = text.substr(start+1, len-2);
			string replacement = map.lookup(placeholder);

			text.replace(start, len, replacement);
		}

		bool playerSpeaking;
		string text;
		vector <Dialoggraph> answers;
		
	protected:		

	};//Dialoggraph

}//name space

#endif